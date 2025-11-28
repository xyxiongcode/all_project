# 最终OOM修复 - 最激进的优化

## 已实施的优化

### 1. 模型参数大幅减少
- **memory_size**: 8 → **4** (减少处理的图像帧数，直接减少编码器内存)
- **predict_size**: 24 → **12** (减少预测序列长度)
- **temporal_depth**: 4 → **2** (Transformer层数减半)
- **heads**: 4 (保持不变)
- **max_seq_len**: 256 → **128** (序列长度减半)
- **query_num**: 32 → **16** (查询数量减半)

### 2. 内存清理策略
- 在RGBD编码器后立即清理输入
- 在goal编码器后立即清理输入
- 在每个Transformer decoder调用后清理中间变量
- 在forward函数的多个关键点清理缓存

### 3. 其他优化
- **batch_size**: 1 (必须保持)
- **num_workers**: 4 → **2** (减少数据加载内存)
- **gradient_accumulation_steps**: 8 (有效batch size=8)
- **bf16**: True (混合精度)

## 预期内存减少

### 编码器部分 (RGB/深度)
- memory_size 8→4: **2倍减少** (处理的图像数减半)

### Transformer部分
- temporal_depth 4→2: **2倍减少**
- max_seq_len 256→128: **4倍减少** (注意力复杂度O(L²))
- query_num 32→16: **2倍减少**

**总内存减少估算**: 约 **32倍** (仅模型部分)

## 如果仍然OOM

如果这些优化仍然不够，可以考虑：

### 最后手段选项

1. **减少图像分辨率**
   ```python
   image_size=160,  # 从224减少到160
   ```

2. **使用梯度检查点** (如果模型支持)
   ```python
   gradient_checkpointing=True
   ```

3. **进一步减少memory_size到2**
   ```python
   memory_size=2,  # 只使用2帧历史
   ```

4. **减少token_dim**
   ```python
   token_dim=256,  # 从384减少到256
   ```

5. **使用CPU offloading** (将部分计算移到CPU)

## 重要提醒

⚠️ **这些优化会影响模型性能**：
- memory_size减少会降低历史信息利用
- temporal_depth减少会降低模型容量
- predict_size减少会降低长期预测能力

需要在内存和性能之间找到平衡。
