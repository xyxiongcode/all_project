# NavDP模型训练指南

## 一、代码说明

`navdp_train.py` 是用于训练NavDP导航策略模型的完整训练脚本。

### 主要组件

1. **RealNavigationDataset类**
   - 负责从磁盘加载训练样本
   - 处理RGB图像、深度图像、目标表示等
   - 将数据转换为PyTorch张量格式

2. **NavDPTrainer类**
   - 实现训练逻辑
   - 计算损失函数（action loss + critic loss + aux loss）
   - 处理模型前向传播和反向传播

3. **数据加载流程**
   - 加载metadata.json获取样本信息
   - 加载历史RGB图像序列（8帧）
   - 加载深度序列
   - 加载目标表示（point_goal, image_goal, pixel_goal）
   - 加载动作序列（24步未来动作）
   - 转换为张量并应用归一化

---

## 二、数据集结构要求

### 目录结构

```
{DATA_ROOT}/
  train/
    samples/
      sample_0_0000/
        rgb_00.png              # 历史RGB图像第0帧 (224x224)
        rgb_01.png              # 历史RGB图像第1帧
        ...
        rgb_07.png              # 历史RGB图像第7帧
        depth_sequence.npy      # 深度序列 (8, 224, 224)
        action_sequence.npy     # 未来动作序列 (24, 3)
        image_goal.png          # 图像目标（可选，224x224）
        pixel_goal.npy          # 像素目标热力图（可选，224x224）
        metadata.json           # 样本元数据
      sample_0_0001/
        ...
  test/
    samples/
      ...（相同结构）
```

### metadata.json格式

```json
{
  "episode_id": 0,
  "step_index": 10,
  "timestamp": 1234567890,
  "point_goal": [1.5, 0.5, 0.1],  // 相对于机器人的目标 [dx, dy, dtheta]
  "robot_pose": {
    "x": 10.5,
    "y": 20.3,
    "theta": 0.5
  },
  "velocity": {
    "linear_x": 0.5,
    "linear_y": 0.0,
    "angular_z": 0.1
  },
  "nav_semantics": {
    "has_goal": true,
    "distance_to_goal": 5.2,
    "heading_error": 0.1,
    "min_lidar_distance": 2.0,
    "safety_score": 0.8
  },
  "move_base_status": "SUCCEEDED",  // 或 "ABORTED", "ACTIVE"等
  "split_type": "train"
}
```

---

## 三、如何使用数据集训练

### 步骤1: 准备数据集

确保数据集已按照上述结构保存：
```bash
ls {DATA_ROOT}/train/samples/sample_0_0000/
# 应该看到: rgb_00.png, rgb_01.png, ..., depth_sequence.npy, action_sequence.npy, metadata.json
```

### 步骤2: 配置参数

编辑配置文件或直接修改代码中的配置：
```python
config.dataset_root_dir = "/path/to/your/dataset"
config.batch_size = 32
config.epochs = 100
config.lr = 1e-4
config.memory_size = 8  # 历史帧数
config.predict_size = 24  # 预测步数
config.image_size = 224  # 图像大小
```

### 步骤3: 运行训练

```bash
cd /home/gr-agv-x9xy/isaac_sim_ws/src/data_capture/scripts
python navdp_train.py
```

### 步骤4: 监控训练进度

训练过程中会输出：
- 每个epoch的训练损失
- 验证损失（如果有验证集）
- 模型检查点保存位置

---

## 四、代码修复说明

### 修复1: RGB图像加载和处理

**问题**：原代码在应用transform前转置了图像维度，导致格式不匹配

**修复**：
- 保持图像为[H, W, C]格式加载
- 使用PIL Image进行转换（transform会自动处理维度）
- 移除了调试print语句

### 修复2: 目标表示加载

**问题**：原代码总是创建空白图像/热力图，不尝试加载实际数据

**修复**：
- **image_goal**: 先尝试加载`image_goal.png`，如果不存在才创建空白图像
- **pixel_goal**: 先尝试加载`pixel_goal.npy`，如果不存在才创建空白热力图
- **point_goal**: 添加了格式验证和错误处理

### 修复3: 深度序列处理

**问题**：深度序列的维度处理不够健壮

**修复**：
- 改进了深度序列的维度检查和调整
- 添加了归一化处理

### 修复4: 错误处理

**问题**：缺少对文件不存在等异常情况的处理

**修复**：
- 添加了文件存在性检查
- 添加了格式验证
- 改进了错误日志

---

## 五、数据格式说明

### 1. RGB图像
- **格式**: PNG图像文件
- **大小**: 224x224像素
- **颜色空间**: RGB（通过cv2转换为RGB）
- **处理**: 归一化到[0,1]，然后应用ImageNet均值和标准差

### 2. 深度序列
- **格式**: NumPy数组文件 (.npy)
- **形状**: (8, 224, 224) - 8帧深度图像
- **处理**: 归一化到[0,1]（除以最大值）

### 3. 点目标 (point_goal)
- **格式**: 列表 [dx, dy, dtheta]
- **单位**: 米（dx, dy），弧度（dtheta）
- **坐标系**: 相对于机器人坐标系
- **处理**: 乘以scene_scale（场景缩放因子）

### 4. 图像目标 (image_goal)
- **格式**: PNG图像文件（可选）
- **大小**: 224x224像素
- **处理**: 如果不存在，创建空白图像并在中心绘制绿点

### 5. 像素目标 (pixel_goal)
- **格式**: NumPy数组文件 (.npy)（可选）
- **形状**: (224, 224)
- **处理**: 如果不存在，创建空白热力图并在中心绘制圆点

### 6. 动作序列
- **格式**: NumPy数组文件 (.npy)
- **形状**: (24, 3) - 24步未来动作，每步3个值[linear_x, linear_y, angular_z]
- **单位**: 米/秒（线性速度），弧度/秒（角速度）
- **处理**: linear_x和linear_y乘以scene_scale

---

## 六、训练输出

### 模型检查点

训练过程中会定期保存检查点：
- **位置**: `config.final_model_dir`
- **格式**: PyTorch模型文件 (.pth)

### 训练日志

训练过程中会输出：
- Epoch进度
- 训练损失（action_loss, critic_loss, aux_loss）
- 验证损失（如果有验证集）
- 样本形状信息（用于调试）

---

## 七、常见问题

### Q1: 数据集加载失败

**检查**：
- 数据集目录路径是否正确
- samples目录是否存在
- 每个sample目录是否包含必要的文件

### Q2: 图像加载错误

**检查**：
- RGB图像是否为有效的PNG文件
- 图像尺寸是否为224x224
- 图像是否损坏

### Q3: 维度不匹配错误

**检查**：
- depth_sequence.npy的维度是否为(8, 224, 224)
- action_sequence.npy的维度是否为(24, 3)
- point_goal是否为长度为3的列表

### Q4: 内存不足

**解决方案**：
- 减小batch_size
- 减少max_samples（限制加载的样本数量）
- 使用更少的验证样本

---

## 八、下一步

训练完成后：
1. 在验证集上评估模型性能
2. 测试模型在实际导航任务中的表现
3. 根据结果调整超参数或重新训练

