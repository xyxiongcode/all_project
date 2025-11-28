# 内存优化总结

## 问题
即使只有18个样本，仍然出现GPU内存超限。这说明**单个样本的模型内存占用过大**。

## 已实施的激进优化

### 1. 减少图像分辨率
- `image_size`: 224 → **168** (14的倍数)
- **内存节省**: 约44%的编码器内存

### 2. 减少历史帧数
- `memory_size`: 8 → **4**
- **内存节省**: 约50%的输入和编码器内存

### 3. 减少预测步数
- `predict_size`: 24 → **12**
- **内存节省**: 约50%的decoder输出内存

### 4. 减少Transformer层数
- `temporal_depth`: 8 → **4**
- **内存节省**: 约50%的Transformer内存

### 5. 减少注意力头数
- `heads`: 8 → **4**
- **内存节省**: 约30%的attention内存

### 6. 减少序列长度
- `max_seq_len`: 128 → **64**
- **内存节省**: 约50%的序列处理内存

### 7. 减少查询数量
- `query_num`: 32 → **16**
- **内存节省**: 约50%的查询内存

### 8. 减少backbone Transformer头数
- backbone `nhead`: 8 → **4**
- **内存节省**: 约30%的backbone attention内存

## 预计总内存节省
**约70-80%**的内存节省

## 其他已优化项
- ✅ `batch_size=1`
- ✅ `num_workers=0`
- ✅ `bf16=True` (混合精度)
- ✅ `gradient_accumulation_steps=8`
- ✅ 内存清理调用已添加

## 如果仍然OOM

### 进一步减少配置
```python
image_size=154,      # 14×11，进一步减少
memory_size=2,       # 从4减少到2
predict_size=6,      # 从12减少到6
temporal_depth=2,    # 从4减少到2
max_seq_len=32,      # 从64减少到32
query_num=8,         # 从16减少到8
```

### 其他选项
1. 冻结编码器：设置`finetune=False`让编码器使用`eval()`模式
2. 使用CPU offloading：将部分计算移到CPU
3. 检查GPU显存：确保有足够的显存（建议至少8GB）

## 当前配置总结
- `image_size=168`
- `memory_size=4`
- `predict_size=12`
- `temporal_depth=4`
- `heads=4`
- `max_seq_len=64`
- `query_num=16`
- `batch_size=1`
- `bf16=True`