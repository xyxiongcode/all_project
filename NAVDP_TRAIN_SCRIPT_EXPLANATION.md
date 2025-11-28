# NavDP训练脚本适配说明

## 一、修改概述

已将`navdp_train.py`修改为适配从`navdp_generate_dataset.py`收集的真实世界数据集，并与NavDP模型架构完全兼容。

## 二、主要组件

### 1. RealNavigationDataset类

**功能**: 加载从`navdp_generate_dataset.py`生成的训练样本

**数据格式适配**:
- 从`samples/sample_<episode_id>_<index>/`目录加载数据
- 加载RGB序列（`rgb_00.png`到`rgb_07.png`）
- 加载深度序列（`depth_sequence.npy`）
- 加载动作序列（`action_sequence.npy`）
- 加载目标表示（`point_goal`, `image_goal.png`, `pixel_goal.npy`）
- 从`metadata.json`加载元数据和critic值计算所需信息

**关键特性**:
- 支持train/test分割
- 自动处理缺失文件（使用默认值）
- 图像预处理（归一化、resize）
- 动作序列缩放（使用scene_scale）

### 2. navdp_collate_fn函数

**功能**: 将单个样本整理成批次

**处理**:
- 堆叠所有张量成批次维度
- 处理critic值的维度（标量压缩）
- 返回字典格式，键名与模型接口匹配

### 3. NavDPTrainer类

**关键修改**:

#### compute_loss方法
- **输入**: 与模型forward方法完全匹配
  - `goal_point`, `goal_image`, `goal_pixel`
  - `input_images`, `input_depths`
  - `output_actions`, `augment_actions`
  - `batch_label_critic`, `batch_augment_critic`

- **模型调用**: 
  ```python
  pred_ng, pred_mg, critic_pred, augment_pred, noise, aux_pred = model(
      goal_point, goal_image, goal_pixel, 
      input_images, input_depths, 
      output_actions, augment_actions
  )
  ```

- **损失计算**:
  - 动作损失: `0.5 * ng_action_loss + 0.5 * mg_action_loss`
  - Critic损失: 基于真实critic值和预测值的MSE
  - 总损失: `0.8 * action_loss + 0.2 * critic_loss + 0.1 * aux_loss`

#### get_train_dataloader方法
- 使用`navdp_collate_fn`作为collate函数
- 支持分布式训练（DistributedSampler）
- 配置了正确的batch_size和num_workers

## 三、数据集到模型的映射

### 数据加载流程

1. **数据集格式** (navdp_generate_dataset.py保存):
   ```
   sample_<episode_id>_<index>/
   ├── metadata.json          # 包含point_goal, nav_semantics等
   ├── rgb_00.png ~ rgb_07.png
   ├── depth_sequence.npy
   ├── action_sequence.npy
   ├── image_goal.png (可选)
   └── pixel_goal.npy (可选)
   ```

2. **RealNavigationDataset加载**:
   - 读取文件并转换为numpy数组
   - 转换为PyTorch张量
   - 应用预处理（归一化、resize等）
   - 计算critic值（基于min_lidar_distance）

3. **navdp_collate_fn整理**:
   - 堆叠成批次
   - 返回字典，键名匹配模型接口

4. **NavDPTrainer.compute_loss**:
   - 提取输入并移动到模型设备
   - 调用模型forward
   - 计算损失并返回

## 四、Critic值计算

由于没有ESDF地图，使用简化的critic值计算（基于LiDAR数据）：

```python
if min_lidar_distance < 0.1:
    base_critic = -5.0  # 非常危险
elif min_lidar_distance < 0.5:
    base_critic = -2.0 + (min_lidar_distance - 0.1) * 5.0  # 危险
else:
    base_critic = min(2.0, 0.5 + (min_lidar_distance - 0.5) / 2.0)  # 相对安全
```

- `batch_label_critic`: 真实轨迹的critic值
- `batch_augment_critic`: 增强轨迹的critic值（0.9倍）

## 五、使用方式

### 初始化数据集

```python
train_dataset = RealNavigationDataset(
    dataset_root="/path/to/navdp_episode_data",
    split='train',
    memory_size=8,
    predict_size=24,
    image_size=(224, 224),
    scene_scale=1.0
)
```

### 训练配置

确保config中包含：
- `config.il.batch_size`: 批次大小
- `config.il.num_workers`: 数据加载器工作进程数
- `config.il.lr`: 学习率

### 模型接口

模型forward方法签名：
```python
def forward(self, goal_point, goal_image, goal_pixel, 
            input_images, input_depths, 
            output_actions, augment_actions)
```

返回：
```python
(noise_pred_ng, noise_pred_mg, cr_label_pred, cr_augment_pred, 
 [ng_noise, mg_noise], [imagegoal_aux_pred, pixelgoal_aux_pred])
```

## 六、注意事项

1. **数据路径**: 确保数据集目录结构正确（`train/samples/`和`test/samples/`）
2. **内存使用**: 大数据集建议使用适当的`num_workers`
3. **设备管理**: 所有张量会自动移动到模型设备
4. **缺失文件**: 如果样本文件缺失，会使用默认值（零张量）
5. **Critic值**: 当前使用简化计算，可根据实际需求调整公式

## 七、与原始NavDP的差异

1. **Critic值计算**: 使用LiDAR数据而非ESDF地图
2. **数据集格式**: 适配真实世界数据收集格式
3. **目标表示**: 支持image_goal和pixel_goal（如果没有则使用默认值）
4. **增强动作**: 使用0.9倍真实动作（而非从ESDF生成）

这些修改使训练脚本能够在没有ESDF地图的情况下，使用真实世界收集的数据训练NavDP模型。

