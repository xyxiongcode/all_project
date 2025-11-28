# 训练错误修复说明

## 错误原因

**错误信息**：
```
Expected tensor for argument #1 'indices' to have one of the following scalar types: Long, Int; but got torch.cuda.FloatTensor instead (while checking arguments for embedding)
```

**根本原因**：
1. `LearnablePositionalEncoding`类使用`nn.Embedding`层来存储位置编码
2. `nn.Embedding`期望整数类型的索引（Long/Int）
3. 但在模型调用时，传入了特征向量（FloatTensor）而不是位置索引

**具体位置**：
- `cond_pos_embed(...)` 被调用时传入特征向量
- `out_pos_embed(...)` 被调用时传入特征向量

---

## 修复方案

### 修复 `LearnablePositionalEncoding.forward()` 方法

修改后的方法可以：
1. **自动检测输入类型**：
   - 如果是浮点数特征向量（FloatTensor），根据序列长度自动生成位置索引
   - 如果是整数位置索引，直接使用

2. **生成位置索引**：
   ```python
   # 根据特征向量的序列长度生成位置索引
   position_ids = torch.arange(seq_len, dtype=torch.long)
   ```

3. **类型转换和限制**：
   - 确保位置索引是Long类型
   - 限制位置索引在有效范围内（0到max_len-1）

---

## 修复后的行为

### 输入特征向量时：
```python
# 输入: [batch_size, seq_len, embed_dim] (FloatTensor)
cond_pos_embed = self.cond_pos_embed(feature_tensor)
# 内部自动生成位置索引: [0, 1, 2, ..., seq_len-1]
# 输出: [batch_size, seq_len, d_model] (位置编码)
```

### 输入位置索引时（向后兼容）：
```python
# 输入: [batch_size, seq_len] (LongTensor)
position_ids = torch.tensor([[0, 1, 2, 3]], dtype=torch.long)
cond_pos_embed = self.cond_pos_embed(position_ids)
# 输出: [batch_size, seq_len, d_model] (位置编码)
```

---

## 测试建议

修复后，训练应该能够正常进行。如果还有问题，检查：

1. **序列长度是否超过max_len**：
   - 如果序列长度超过`max_len`，位置索引会被限制
   - 检查日志中是否有相关警告

2. **输入维度是否正确**：
   - 确保输入tensor的维度符合预期
   - `[batch_size, seq_len, ...]`格式

3. **设备一致性**：
   - 确保位置索引和embedding层在同一设备上

---

## 相关文件

- `src/data_capture/scripts/encoder/navdp_backbone.py`: 修复了`LearnablePositionalEncoding`类

