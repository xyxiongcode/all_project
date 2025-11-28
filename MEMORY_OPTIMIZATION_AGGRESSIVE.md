# 激进的内存优化方案

## 已实施的优化

### 1. 模型参数减少
- **temporal_depth**: 从 8 减少到 4
- **heads**: 从 8 减少到 4
- **max_seq_len**: 从 512 减少到 256
- **query_num**: 从 64 减少到 32

### 2. 内存清理策略
- 在 `training_step` 前后清理 CUDA 缓存
- 在 Transformer decoder 调用之间清理中间变量
- 在每个大的计算块后立即清理

### 3. Batch Size
- 保持 `batch_size=1`
- `gradient_accumulation_steps=8` (有效batch size=8)

### 4. 混合精度训练
- 已启用 `bf16=True`

## 内存估算

### Transformer 内存消耗
对于序列长度 L 和隐藏维度 D：
- Self-Attention: O(L² × D)
- Feed-Forward: O(L × D²)
- 总内存: O(L² × D + L × D²)

### 优化前 vs 优化后
- **序列长度**: 512 → 256 (内存减少 ~4x)
- **temporal_depth**: 8 → 4 (内存减少 ~2x)
- **heads**: 8 → 4 (内存减少 ~2x)

总内存减少约 **16x** (仅Transformer部分)

## 进一步优化建议

如果仍然遇到OOM错误，可以考虑：

1. **进一步减少序列长度到 128**
2. **减少 predict_size 从 24 到 12**
3. **减少 token_dim 从 384 到 256**
4. **使用 CPU offloading** (将部分计算移到CPU)

## 监控内存使用

使用以下命令监控GPU内存：
```bash
watch -n 1 nvidia-smi
```

或使用提供的脚本：
```bash
python check_cuda_memory.py
```
