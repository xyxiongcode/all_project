# NavDP Embedding长度问题分析与解决方案

## 问题描述

报错信息：
```
[ERROR] 模型前向传播失败: The shape of the 2D attn_mask is torch.Size([24, 132]), but should be (24, 68).
```

## 根本原因分析

### 1. 实际序列长度计算

**实际的条件embedding序列长度：68**

```python
# 在 navdp_policy.py 的 forward() 方法中
ng_cond_embed_tensor = torch.cat([
    ng_time_embed,      # 1个token
    nogoal_embed,       # 1个token
    nogoal_embed,       # 1个token
    nogoal_embed,       # 1个token
    rgbd_embed,         # 64个token（由rgbd_encoder输出）
], dim=1)
# 总计：1 + 1 + 1 + 1 + 64 = 68个token
```

### 2. 期望序列长度计算

**期望的序列长度：132**

```python
# 在 navdp_policy.py 的 __init__() 方法中（已修复前）
self.cond_critic_mask = torch.zeros((self.predict_size, 4 + self.memory_size * 16))
# 如果 predict_size=24, memory_size=8
# 形状 = (24, 4 + 8*16) = (24, 132)
```

### 3. RGBD Encoder的输出序列长度

**为什么rgbd_embed是64而不是128？**

在 `encoder/navdp_backbone.py` 中：

```python
# 第391行
query_num = min(self.memory_size * 8, 64)  # 限制查询数量

# 如果 memory_size = 8
# query_num = min(8 * 8, 64) = min(64, 64) = 64
```

**关键发现：**
- RGBD encoder使用 `memory_size * 8` 而不是 `memory_size * 16`
- 而且被限制为最大64
- 所以即使memory_size=8，输出也只有64个token，而不是期望的128个

### 4. 配置参数

在 `navdp_generate_dataset.py` 中：
```python
self.memory_size = 8  # 历史帧数
```

## 解决方案

### 方案1：动态创建Mask（已实现，推荐）

在运行时根据实际的条件embedding序列长度动态创建mask：

```python
# 在 forward() 方法中
actual_memory_seq_len = ng_cond_embeddings.shape[1]  # 获取实际的memory序列长度
if self.cond_critic_mask is None or self.cond_critic_mask.shape[1] != actual_memory_seq_len:
    self.cond_critic_mask = torch.zeros(
        (self.predict_size, actual_memory_seq_len),
        device=device,
        dtype=torch.float32
    )
    mask_prefix_len = min(4, actual_memory_seq_len)
    self.cond_critic_mask[:, 0:mask_prefix_len] = float('-inf')
```

**优点：**
- 自动适配实际序列长度
- 无需修改其他配置
- 灵活性高

### 方案2：修改RGBD Encoder的查询数量

如果希望使用 `memory_size * 16` 个token，需要修改 `encoder/navdp_backbone.py`：

```python
# 修改前
query_num = min(self.memory_size * 8, 64)  # 限制查询数量

# 修改后
query_num = min(self.memory_size * 16, 128)  # 使用16倍，并提高上限
```

**同时需要修改：**
1. `navdp_policy.py` 中的 `cond_pos_embed` 最大长度
2. `navdp_backbone.py` 中的 `former_query` 最大长度

```python
# navdp_policy.py
self.cond_pos_embed = LearnablePositionalEncoding(
    self.token_dim, 
    self.memory_size * 16 + 4  # 改为 16 倍
)

# navdp_backbone.py
self.former_query = LearnablePositionalEncoding(
    d_model=self.feature_dim, 
    max_len=self.memory_size * 16  # 改为 16 倍
)
```

**优点：**
- 与原始设计一致
- 使用更多token，可能提升性能

**缺点：**
- 需要重新训练模型
- 计算量增加

### 方案3：修改memory_size配置

如果想保持 `memory_size * 16` 的设计，可以减小 `memory_size`：

```python
# 在 navdp_generate_dataset.py 中
self.memory_size = 4  # 从8改为4

# 这样：
# rgbd_embed长度 = min(4 * 8, 64) = 32
# 条件embedding长度 = 1 + 3 + 32 = 36
# 期望长度 = 4 + 4 * 16 = 68（仍然不匹配）
```

这个方案不推荐，因为需要重新收集数据。

## 推荐的修改方案

### 如果想增加embedding长度（使用方案2）

1. **修改 `encoder/navdp_backbone.py` 第391行：**

```python
# 修改前
query_num = min(self.memory_size * 8, 64)

# 修改后
query_num = min(self.memory_size * 16, 128)  # 改为16倍，上限提高
```

2. **修改 `encoder/navdp_backbone.py` 第222行：**

```python
# 修改前
self.former_query = LearnablePositionalEncoding(d_model=self.feature_dim, max_len=self.memory_size * 16)

# 保持原样即可（已经是16倍）
```

3. **修改 `navdp_policy.py` 第131行：**

```python
# 修改前
self.cond_pos_embed = LearnablePositionalEncoding(self.token_dim, self.memory_size * 16 + 4)

# 保持原样即可（已经是16倍+4）
```

4. **重新初始化 `cond_critic_mask`：**

```python
# 在 __init__() 中恢复固定大小创建（如果采用方案2）
self.cond_critic_mask = torch.zeros((self.predict_size, 4 + self.memory_size * 16))
self.cond_critic_mask[:, 0:4] = float('-inf')
```

### 如果想保持当前长度（使用方案1，已实现）

当前代码已经实现了动态创建mask的方案，无需额外修改。只需要确保：
- `cond_critic_mask` 初始化为 `None`
- 在 `forward()` 中动态创建mask

## 当前状态

✅ **已实现方案1（动态创建mask）**：
- `cond_critic_mask` 在初始化时为 `None`
- 在 `forward()` 方法中根据实际序列长度动态创建
- 自动适配任何序列长度

## 总结

- **实际序列长度68** = 1(time) + 3(goals) + 64(rgbd)
- **期望序列长度132** = 4 + 8*16（基于原始设计）
- **差异原因**：RGBD encoder使用 `memory_size * 8` 而不是 `memory_size * 16`，且被限制为最大64
- **解决方案**：已实现动态创建mask，自动适配实际序列长度

如果需要使用更长的embedding（128个token），可以按照方案2修改RGBD encoder的查询数量。

