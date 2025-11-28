# 内存泄漏修复说明

## 问题
每个epoch，GPU显存增加约2GB，这是严重的内存泄漏。

## 根本原因

### 1. **compute_loss返回的outputs包含GPU张量**
- `pred_ng`, `pred_mg`, `critic_pred`, `augment_pred`, `noise`等都是GPU张量
- 这些张量保存在outputs字典中，没有被及时释放
- **每个step累积2GB张量 × 数千个steps = 数GB内存泄漏**

### 2. **TensorBoard writer缓存事件**
- SummaryWriter会缓存事件直到flush
- 如果没有定期flush，事件会累积在内存中
- **每个epoch可能有数千个事件未写入磁盘**

### 3. **中间变量未及时删除**
- `inputs`字典包含大量GPU张量
- `outputs`字典包含GPU张量
- 这些变量在training_step结束后没有被删除

## 修复措施

### 1. 修改compute_loss返回值
- ✅ **只返回标量值**，不返回GPU张量
- ✅ **立即删除中间GPU张量**（pred_ng, pred_mg等）
- ✅ 使用`.item()`提取标量值

### 2. 在training_step中及时清理
- ✅ **立即删除outputs和inputs**
- ✅ 在每次step后清理CUDA缓存

### 3. TensorBoard定期flush
- ✅ **每个epoch结束时flush**
- ✅ **每100步flush一次**（防止事件累积过多）

### 4. Epoch结束时全面清理
- ✅ **flush TensorBoard writer**
- ✅ **清理CUDA缓存**
- ✅ **调用gc.collect()**

## 修复后的代码

### compute_loss修改
```python
# 之前：outputs包含GPU张量
outputs = {
    'pred_ng': pred_ng,  # GPU张量！
    'pred_mg': pred_mg,  # GPU张量！
    ...
}

# 之后：只返回标量值
outputs = {
    'ng_action_loss': ng_action_loss.item(),  # 标量
    'mg_action_loss': mg_action_loss.item(),  # 标量
    ...
}
del pred_ng, pred_mg, ...  # 立即删除GPU张量
```

### training_step修改
```python
loss, outputs = self.compute_loss(...)
# 保存标量值
self._last_outputs = {...}
# 立即删除
del outputs
del inputs
torch.cuda.empty_cache()
```

### TensorBoard flush
```python
# 每100步flush
if state.global_step % 100 == 0:
    self.writer.flush()

# 每个epoch结束时flush
def on_epoch_end(...):
    self.writer.flush()
    torch.cuda.empty_cache()
    gc.collect()
```

## 预期效果
- ✅ **每个epoch的内存增长应该接近0**
- ✅ GPU显存保持稳定，不再累积增长
- ✅ 训练可以正常运行多个epoch

## 验证方法
运行训练，观察：
```python
# 每个epoch开始时记录显存
print(f"Epoch {epoch} start: {torch.cuda.memory_allocated()/1024**3:.2f} GB")
# 每个epoch结束时记录显存
print(f"Epoch {epoch} end: {torch.cuda.memory_allocated()/1024**3:.2f} GB")
```

如果显存增长接近0，说明修复成功！
