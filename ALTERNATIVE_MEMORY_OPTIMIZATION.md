# 替代内存优化方案（保持模型容量）

## 已实施的优化（不减少模型容量）

### 1. 减少图像分辨率 ✅
- `image_size`: 224 → **160**
- **内存节省**: 约50%的编码器内存
- **优势**: 不影响模型结构，只减少输入尺寸

### 2. 禁用多进程数据加载 ✅
- `num_workers`: 2 → **0**
- **优势**: 减少数据加载的内存开销

### 3. 更早的序列压缩 ✅
- `max_seq_len`: 256 → **128**
- **优势**: 减少Transformer attention的内存占用

## 其他可选优化方案

### 方案1: 使用环境变量优化内存分配

创建启动脚本 `run_train.sh`:

```bash
#!/bin/bash
# 设置PyTorch CUDA内存分配优化
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True

# 限制CUDA缓存分配
export CUDA_LAUNCH_BLOCKING=0

# 运行训练
python src/data_capture/scripts/train.py
```

这样可以：
- 减少内存碎片化
- 提高内存利用率
- 避免不必要的内存保留

### 方案2: 分块处理图像（如果可能）

将图像序列分成小块处理，处理完一块后清理再处理下一块。

### 方案3: CPU Offloading（部分计算移到CPU）

将某些中间结果移到CPU，需要时再移回GPU。

### 方案4: 更激进的序列长度压缩

即使保持memory_size=8，可以在编码后立即将序列长度限制到更小（如64或128）。

### 方案5: 优化数据加载

- 禁用数据预加载
- 使用更小的图像缓存
- 即时加载数据而不是批量加载

## 当前配置总结

保持模型容量：
- `memory_size=8` ✅
- `predict_size=24` ✅
- `temporal_depth=8` ✅
- `heads=8` ✅

已优化的参数：
- `image_size=160` (减少编码器内存)
- `num_workers=0` (减少数据加载内存)
- `max_seq_len=128` (减少Transformer内存)
- `batch_size=1` (必须)
- `bf16=True` (混合精度)

## 如果仍然OOM

可以尝试：

1. **进一步减少图像分辨率到128**
   ```python
   image_size=128,  # 从160再减少到128
   ```

2. **使用环境变量**
   ```bash
   export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
   ```

3. **进一步减少序列长度到64**
   ```python
   max_seq_len = 64  # 在navdp_backbone.py中
   ```

4. **减少query_num**
   ```python
   query_num = 16  # 减少查询数量
   ```