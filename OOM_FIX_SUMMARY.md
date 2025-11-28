# CUDA OOM 问题修复总结

## 问题分析

您遇到的OOM错误发生在以下阶段：
1. **RGB编码器**（DINOv2 Vision Transformer）
2. **深度编码器**（DINOv2 Vision Transformer）
3. 这些编码器对batch size非常敏感

**关键问题**：您将`batch_size`从1改为了4，这导致内存使用增加了4倍！

## 已实施的修复

### 1. 恢复batch_size=1 ✅
```python
batch_size=1,  # 必须保持为1，batch_size=4会导致OOM
```

### 2. 在编码器后立即清理内存 ✅
- RGB编码器计算后立即删除中间变量
- 深度编码器计算后立即删除中间变量
- 每次删除后调用`torch.cuda.empty_cache()`和`gc.collect()`

### 3. 之前的优化仍然有效
- `temporal_depth`: 4 (从8减少)
- `heads`: 4 (从8减少)  
- `max_seq_len`: 256 (从512减少)
- `query_num`: 32 (从64减少)
- `batch_size`: 1 (必须保持)
- `gradient_accumulation_steps`: 8 (有效batch size=8)

## 为什么batch_size必须为1？

Vision Transformer（如DINOv2）的内存复杂度：
- **Self-Attention**: O(batch_size × seq_len² × hidden_dim)
- **Feed-Forward**: O(batch_size × seq_len × hidden_dim²)

对于图像编码器：
- 输入形状: `[batch_size, 8, 3, 224, 224]` (8帧历史)
- batch_size=1时：处理1×8=8张图像
- batch_size=4时：处理4×8=32张图像 (**4倍内存**)

## 如果仍然OOM

如果使用`batch_size=1`仍然遇到OOM，可以考虑：

### 选项1：减少memory_size
```python
memory_size=4,  # 从8减少到4（处理更少的历史帧）
```

### 选项2：进一步减少序列长度
```python
max_seq_len = 128  # 从256进一步减少到128
```

### 选项3：减少图像分辨率
```python
image_size=160,  # 从224减少到160
```

### 选项4：使用CPU offloading
将部分计算移到CPU（会显著降低速度）

## 验证修复

运行训练脚本并监控内存：
```bash
# 在另一个终端运行
watch -n 1 nvidia-smi

# 或使用脚本
python check_cuda_memory.py
```

预期内存使用：约10-12 GiB（而不是16+ GiB）

## 重要提醒

⚠️ **永远不要将batch_size增加到1以上**，除非：
1. 您有更大的GPU（>40GB）
2. 您进一步减少了其他内存占用（memory_size、序列长度等）
3. 您实施了CPU offloading
