# 最终内存优化方案

## 当前问题

即使batch_size=2，仍然出现CUDA内存不足错误。这说明模型本身的内存占用非常大。

## 已实施的优化

1. ✅ **batch_size**: 16 → 8 → 2 → **1** (最小化内存占用)
2. ✅ **temporal_depth**: 16 → 8 (减少模型层数，参数从82M → 63M)
3. ✅ **混合精度训练**: 启用bf16 (减少约50%内存)
4. ✅ **梯度累积**: 增加到8 (保持有效batch_size=8)
5. ✅ **工作进程**: 8 → 4 (减少CPU内存)
6. ✅ **CUDA缓存清理**: 在training_step中添加

## 当前配置

```python
# navdp_configs.py
batch_size = 1  # 最小批次大小
temporal_depth = 8  # 减少的模型深度
num_workers = 4

# train.py
per_device_train_batch_size = 1
gradient_accumulation_steps = 8  # 有效batch_size = 1 × 8 = 8
bf16 = True
```

## 如果仍然内存不足

### 选项1: 进一步减小模型容量
```python
# navdp_configs.py
temporal_depth = 6  # 进一步减少
heads = 4  # 从8减少到4
token_dim = 256  # 从384减少到256
```

### 选项2: 减小图像尺寸
```python
# navdp_configs.py
image_size = 192  # 从224减少到192
```

### 选项3: 设置环境变量（运行前）
```bash
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
```

### 选项4: 检查是否有其他进程占用GPU
```bash
nvidia-smi
```

## 训练速度影响

- **batch_size=1**: 每个step的时间会增加约2倍
- **gradient_accumulation_steps=8**: 需要8个step才能更新一次，但内存使用减少
- **总体**: 训练速度会变慢，但可以正常运行

## 建议

如果仍然遇到内存问题，考虑：
1. 检查是否有其他进程占用GPU内存
2. 重启Python进程以清理内存碎片
3. 考虑使用更小的模型配置

