# CUDA内存不足问题分析和解决方案

## 问题分析

### 当前状态
- **GPU总容量**: 23.55 GiB
- **已使用内存**: 22.63 GiB
- **PyTorch分配**: 21.89 GiB
- **空闲内存**: 131.75 MiB
- **错误**: 尝试分配20 MiB时失败

### 内存使用分析
从DEBUG输出可以看到模型正在处理：
- **批次大小**: 16
- **特征维度**: [16, 8, 256, 384] (RGB + Depth)
- **Transformer序列长度**: 512 (下采样后)
- **预测序列长度**: 24
- **模型参数**: 82,190,602 个参数

模型已经使用了接近GPU的全部容量，在第一个训练步骤时就达到了内存上限。

## 解决方案（按优先级排序）

### 方案1: 启用混合精度训练 + 梯度检查点（推荐）
**优点**: 保持batch_size不变，内存减少约60-70%
**实施**:
- 启用 `bf16=True` (如果GPU支持) 或 `fp16=True`
- 启用 `gradient_checkpointing=True`

### 方案2: 减小batch_size
**优点**: 简单直接
**缺点**: 训练速度变慢，可能需要调整学习率
**建议**: 从16降到8或4

### 方案3: 增加梯度累积
**优点**: 保持有效batch_size，减小实际batch_size
**建议**: batch_size=8, gradient_accumulation_steps=2 (有效batch_size仍为16)

### 方案4: 减小num_workers
**优点**: 减少CPU内存使用（不影响GPU内存，但可能有助于系统稳定性）
**建议**: 从8降到4

### 方案5: 优化数据加载
**优点**: 减少内存占用
**建议**: 
- 设置 `dataloader_pin_memory=False` (已设置)
- 设置 `persistent_workers=False`

## 推荐的组合方案

### 组合A: 最大内存节省（推荐）
```python
- gradient_checkpointing=True
- bf16=True (如果GPU支持) 或 fp16=True
- batch_size=8
- gradient_accumulation_steps=2
- num_workers=4
```

### 组合B: 平衡方案
```python
- gradient_checkpointing=True
- bf16=True (如果GPU支持) 或 fp16=True
- batch_size=12
- gradient_accumulation_steps=1
- num_workers=4
```

### 组合C: 保守方案（如果GPU不支持bf16）
```python
- gradient_checkpointing=True
- batch_size=4
- gradient_accumulation_steps=4 (保持有效batch_size=16)
- num_workers=4
```

## 实施步骤

### 步骤1: 修改配置文件 (`navdp_configs.py`)
```python
batch_size=8  # 从16降到8
num_workers=4  # 从8降到4
```

### 步骤2: 修改训练参数 (`train.py`)
```python
gradient_checkpointing=True,  # 从False改为True
bf16=True,  # 从False改为True（如果GPU支持）或使用fp16=True
gradient_accumulation_steps=2,  # 从1改为2
```

### 步骤3: 检查GPU是否支持bf16
```bash
python -c "import torch; print(torch.cuda.is_bf16_supported())"
```

## 预期效果

| 方案 | 内存节省 | Batch Size | 训练速度影响 |
|------|---------|-----------|-------------|
| 混合精度 | ~50% | 不变 | 轻微提升 |
| 梯度检查点 | ~30% | 不变 | 约10-20%变慢 |
| 减小batch_size | ~50% | 减半 | 约2倍变慢 |
| 组合A | ~70% | 减半 | 约1.5-2倍变慢 |

## 注意事项

1. **混合精度训练**: 如果GPU不支持bf16，使用fp16可能会导致数值不稳定
2. **梯度累积**: 需要确保有效batch_size足够大以保证训练稳定性
3. **学习率调整**: 如果减小了batch_size，可能需要相应调整学习率
4. **内存碎片**: 如果仍然遇到内存问题，尝试设置环境变量：
   ```bash
   export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
   ```

