# 训练错误修复总结

## 错误1: JSON序列化错误

### 错误信息
```
TypeError: Object of type ExpCfg is not JSON serializable
```

### 原因
- `NavDPModelConfig`包含`model_cfg`字段，它是一个`ExpCfg`对象（Pydantic模型）
- 保存检查点时，`save_pretrained()`会尝试将配置序列化为JSON
- `ExpCfg`对象无法直接JSON序列化

### 修复
在`NavDPModelConfig`类中添加了`to_dict()`方法：
- 检测`model_cfg`是否为Pydantic模型
- 如果是，转换为字典（使用`model_dump()`或`dict()`方法）
- 确保配置可以JSON序列化

### 修复位置
`src/data_capture/scripts/navdp_policy.py` - `NavDPModelConfig.to_dict()`方法

---

## 错误2: Embedding层类型错误

### 错误信息
```
Expected tensor for argument #1 'indices' to have one of the following scalar types: Long, Int; 
but got torch.cuda.FloatTensor instead (while checking arguments for embedding)
```

### 原因
- `LearnablePositionalEncoding`使用`nn.Embedding`层存储位置编码
- `nn.Embedding`期望整数类型的索引（Long/Int）
- 但模型传入的是浮点数特征向量（FloatTensor）

### 修复
修改了`LearnablePositionalEncoding.forward()`方法：
- 自动检测输入类型（特征向量 vs 位置索引）
- 如果是特征向量，根据序列长度自动生成位置索引
- 如果是位置索引，直接使用并确保是Long类型

### 修复位置
`src/data_capture/scripts/encoder/navdp_backbone.py` - `LearnablePositionalEncoding.forward()`方法

---

## 修复后的预期效果

### 1. JSON序列化 ✅
- ✅ 保存检查点时不再出现JSON序列化错误
- ✅ `ExpCfg`对象会被正确转换为字典

### 2. Embedding错误 ✅
- ✅ 位置编码层可以接收特征向量输入
- ✅ 自动生成正确的位置索引
- ✅ 确保所有索引都是Long类型

### 3. 训练稳定性 ⚠️
- ✅ 训练应该可以正常进行
- ⚠️ 如果仍有embedding错误，可能需要检查其他位置

---

## 测试建议

1. **重新运行训练**：
   ```bash
   python navdp_train.py
   ```

2. **观察输出**：
   - 检查是否还有embedding错误
   - 检查是否还有JSON序列化错误
   - 检查训练是否可以正常进行

3. **如果仍有问题**：
   - 检查错误堆栈跟踪，定位具体位置
   - 可能需要检查其他embedding层的使用

---

## 相关文件

1. ✅ `src/data_capture/scripts/navdp_policy.py` - 添加了`to_dict()`方法
2. ✅ `src/data_capture/scripts/encoder/navdp_backbone.py` - 修复了`LearnablePositionalEncoding.forward()`

