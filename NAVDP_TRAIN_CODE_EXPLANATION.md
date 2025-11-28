# NavDP训练代码说明

## 一、代码整体结构

`navdp_train.py` 是用于训练NavDP导航策略模型的脚本，包含以下主要组件：

### 1. **RealNavigationDataset类**（第15-232行）

负责从磁盘加载训练数据并转换为模型可用的格式。

**主要功能**：
- 加载数据集：从指定目录加载训练/验证样本
- 数据预处理：将RGB图像、深度图像、目标表示等转换为PyTorch张量
- 数据增强：可选的图像变换（transform）

**关键方法**：
- `__init__()`: 初始化数据集，设置路径、参数等
- `__getitem__()`: 加载单个样本并转换为张量
- `_create_empty_sample()`: 创建空样本（用于错误处理）

### 2. **data_collator函数**（第234-270行）

将多个样本批次化，处理不同形状的张量。

**主要功能**：
- 将列表中的样本堆叠成批次张量
- 处理metadata等非张量数据
- 处理形状不匹配的情况

### 3. **NavDPTrainer类**（第278-401行）

继承自`BaseTrainer`，实现NavDP特定的训练逻辑。

**主要功能**：
- 计算损失函数（action loss + critic loss + aux loss）
- 处理模型输入输出格式转换
- 处理设备（CPU/GPU）转移

**关键方法**：
- `compute_loss()`: 计算训练损失，调用模型前向传播

### 4. **main函数**（第404-505行）

训练入口，负责：
- 加载配置
- 创建模型
- 加载数据集
- 创建训练器
- 开始训练

---

## 二、数据加载流程

### 1. 数据集目录结构

```
{DATA_ROOT}/
  train/
    samples/
      sample_{episode_id}_{sample_index:04d}/
        rgb_00.png              # 历史RGB图像第0帧
        rgb_01.png              # 历史RGB图像第1帧
        ...
        rgb_07.png              # 历史RGB图像第7帧（memory_size=8）
        depth_sequence.npy      # 深度序列（8x224x224）
        action_sequence.npy     # 未来动作序列（24x3）
        image_goal.png          # 图像目标（可选）
        pixel_goal.npy          # 像素目标（可选）
        metadata.json           # 样本元数据
  test/
    samples/
      ...（相同结构）
```

### 2. 样本加载步骤（`__getitem__`方法）

1. **加载metadata.json**
   - 读取样本元数据
   - 获取point_goal、robot_pose等信息

2. **加载历史RGB图像序列**
   - 从`rgb_{i:02d}.png`加载8帧图像
   - 调整大小到224x224
   - 转换为RGB格式

3. **加载深度序列**
   - 从`depth_sequence.npy`加载
   - 确保维度为[8, 224, 224]
   - 归一化深度值

4. **加载目标表示**
   - point_goal: 从metadata.json读取
   - image_goal: 从`image_goal.png`加载，如果没有则创建空白图像
   - pixel_goal: 从`pixel_goal.npy`加载，如果没有则创建空白热力图

5. **加载动作序列**
   - 从`action_sequence.npy`加载
   - 确保长度为24步
   - 应用场景缩放

6. **转换为张量**
   - RGB序列: [memory_size, 3, H, W]
   - 深度序列: [memory_size, 1, H, W]
   - 目标表示: 各种格式
   - 动作序列: [predict_size, 3]

7. **创建价值标签**
   - 基于move_base_status判断成功/失败
   - batch_label_critic: 1.0（成功）或0.0（失败）
   - batch_augment_critic: 0.8（成功）或0.2（失败）

---

## 三、训练流程

### 1. 初始化阶段

```python
# 1. 加载配置
config = navdp_exp_cfg.il

# 2. 创建模型
model = NavDPNet(model_config)

# 3. 加载预训练模型（可选）
if pretrained_path:
    model = load_pretrained_model(model, pretrained_path)

# 4. 创建数据集
train_dataset = RealNavigationDataset(config, root_dir, split='train')
val_dataset = RealNavigationDataset(config, root_dir, split='test')

# 5. 创建训练器
trainer = NavDPTrainer(config, model, train_dataset, data_collator)
```

### 2. 训练循环

每个epoch：
1. 遍历训练集批次
2. 对每个批次：
   - 调用`compute_loss()`计算损失
   - 反向传播更新模型参数
   - 记录损失值
3. 在验证集上评估模型性能
4. 保存检查点

### 3. 损失计算

```python
# Action Loss（动作预测损失）
ng_action_loss = (noise_pred_ng - ng_noise).square().mean()
mg_action_loss = (noise_pred_mg - mg_noise).square().mean()
action_loss = 0.5 * mg_action_loss + 0.5 * ng_action_loss

# Critic Loss（价值函数损失）
critic_loss = (
    (cr_label_pred - batch_label_critic).square().mean() +
    (cr_augment_pred - batch_augment_critic).square().mean()
)

# Aux Loss（辅助损失，目标预测）
aux_loss = (
    0.5 * (goal_point - imagegoal_aux_pred).square().mean() +
    0.5 * (goal_point - pixelgoal_aux_pred).square().mean()
)

# 总损失
loss = alpha * action_loss + (1 - alpha) * critic_loss + 0.5 * aux_loss
```

---

## 四、如何训练模型

### 1. 准备数据集

确保数据集目录结构正确：
```bash
{DATA_ROOT}/
  train/samples/  # 训练样本
  test/samples/   # 测试样本（可选）
```

### 2. 配置参数

在配置文件中设置：
- `dataset_root_dir`: 数据集根目录路径
- `batch_size`: 批次大小
- `epochs`: 训练轮数
- `lr`: 学习率
- `memory_size`: 历史帧数（默认8）
- `predict_size`: 预测步数（默认24）
- `image_size`: 图像大小（默认224）

### 3. 运行训练

```bash
cd /home/gr-agv-x9xy/isaac_sim_ws/src/data_capture/scripts
python navdp_train.py
```

### 4. 监控训练

训练过程中会输出：
- 每个epoch的训练损失
- 验证损失（如果有验证集）
- 模型检查点保存路径

---

## 五、当前代码存在的问题

### 1. RGB图像处理问题
- 代码在应用transform前先转置图像维度，但transform期望[H,W,C]格式
- 需要修复图像转换流程

### 2. 目标加载问题
- 应该先尝试加载image_goal.png和pixel_goal.npy
- 如果不存在才创建空白图像

### 3. 调试代码
- 存在调试print语句（第160行），应该移除

### 4. 错误处理
- 缺少对文件不存在情况的完整处理
- 需要更好的错误日志

---

## 六、修复计划

1. ✅ 修复RGB图像转换逻辑
2. ✅ 添加image_goal.png和pixel_goal.npy的加载
3. ✅ 移除调试print语句
4. ✅ 改进错误处理
5. ✅ 确保point_goal格式正确

