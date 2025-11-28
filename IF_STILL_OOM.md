# 如果仍然出现OOM错误的解决方案

## 当前已实施的优化
- ✅ `image_size=168` (减少44%编码器内存)
- ✅ `memory_size=4` (减少50%输入内存)
- ✅ `predict_size=12` (减少50%输出内存)
- ✅ `temporal_depth=4` (减少50%Transformer内存)
- ✅ `heads=4` (减少30%attention内存)
- ✅ `max_seq_len=64` (减少50%序列内存)
- ✅ `query_num=16` (减少50%查询内存)
- ✅ `batch_size=1`
- ✅ `num_workers=0`
- ✅ `bf16=True`
- ✅ `finetune=False` (编码器使用eval模式，不存储梯度)

## 进一步优化方案

### 方案1: 进一步减少配置参数（推荐）
修改 `navdp_configs.py`:
```python
image_size=154,      # 14×11，从168进一步减少
memory_size=2,       # 从4减少到2（最小可用值）
predict_size=6,      # 从12减少到6
temporal_depth=2,    # 从4减少到2（最小可用值）
max_seq_len=32,      # 从64减少到32
query_num=8,         # 从16减少到8
```

### 方案2: 冻结更多层
在训练开始前，可以冻结RGB和深度编码器：
```python
# 在train.py中，创建模型后添加
if not config.il.finetune:
    model.rgbd_encoder.rgb_model.eval()
    model.rgbd_encoder.depth_model.eval()
    for param in model.rgbd_encoder.rgb_model.parameters():
        param.requires_grad = False
    for param in model.rgbd_encoder.depth_model.parameters():
        param.requires_grad = False
```

### 方案3: 减少梯度累积步数
修改 `train.py`:
```python
gradient_accumulation_steps=4,  # 从8减少到4（但仍然可以累积）
```

### 方案4: 检查GPU显存
运行以下命令查看实际可用显存：
```bash
nvidia-smi
python check_cuda_memory.py
```

如果显存小于8GB，可能需要：
1. 使用更小的模型配置
2. 考虑使用CPU训练（虽然会很慢）
3. 升级GPU硬件

### 方案5: 分阶段训练
先冻结编码器训练decoder，然后再微调编码器。

### 方案6: 使用DeepSpeed ZeRO
如果有多GPU，可以使用DeepSpeed ZeRO来分片优化器状态和梯度。

## 调试步骤

1. **检查实际使用的内存**:
   ```python
   import torch
   print(f"Allocated: {torch.cuda.memory_allocated()/1024**3:.2f} GB")
   print(f"Reserved: {torch.cuda.memory_reserved()/1024**3:.2f} GB")
   ```

2. **在forward之前清理缓存**:
   ```python
   torch.cuda.empty_cache()
   ```

3. **检查是否有内存泄漏**:
   - 观察内存是否持续增长
   - 检查是否有未释放的大张量

## 最小配置（如果上述都不行）
```python
image_size=154,      # 14×11
memory_size=2,       # 最小
predict_size=6,      # 最小
temporal_depth=2,    # 最小
heads=2,             # 最小（需要能被token_dim=384整除）
max_seq_len=32,      # 最小
query_num=8,         # 最小
batch_size=1,
num_workers=0,
bf16=True,
finetune=False,
```
