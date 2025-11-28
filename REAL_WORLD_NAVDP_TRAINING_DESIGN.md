# 真实世界NavDP训练方案设计

## 一、NavDP论文 vs 真实数据对比

### NavDP论文的设计（仿真）
- **Diffusion Policy**: 生成轨迹分布
- **Critic Function**: 从ESDF地图评估轨迹安全性（特权信息）
- **训练数据**: 从仿真环境收集，有ESDF地图

### 真实数据的特点
- ✅ 有RGB/深度图像序列
- ✅ 有动作序列（cmd_vel）
- ✅ 有LiDAR数据（可计算障碍物距离）
- ✅ 有机器人位姿
- ❌ **没有ESDF地图**（这是关键差异）

## 二、简化方案设计

### 方案A：基于轨迹重建的Critic计算（推荐）

在训练时，从动作序列重建轨迹点，然后使用LiDAR数据计算critic值。

**优势**：
- 更接近NavDP原始设计
- Critic值反映轨迹的实际安全性

**实现**：
1. 从动作序列重建轨迹点（在机器人坐标系）
2. 使用episode中的LiDAR数据，提取障碍物点
3. 计算每个轨迹点到障碍物的距离
4. 使用NavDP公式计算critic值

### 方案B：基于点安全性的Critic计算（简化版）

直接使用`nav_semantics`中的`min_lidar_distance`计算critic值。

**优势**：
- 实现简单
- 不需要重建轨迹

**劣势**：
- 只反映当前点安全性，不是整个轨迹

### 方案C：混合方案（推荐用于训练）

结合方案A和B：
- 训练初期：使用方案B（简单快速）
- 训练后期：使用方案A（更准确）

## 三、推荐实现方案

### 1. 在数据生成时添加轨迹重建和Critic计算

在`navdp_generate_dataset.py`的`_create_training_sample_from_data`中：
- 从action_sequence重建轨迹点
- 使用当前episode的LiDAR数据计算障碍物距离
- 计算critic值并保存到样本中

### 2. 在训练时使用Critic值

在`navdp_train.py`中：
- 如果样本中有critic值，直接使用
- 如果没有，使用简化的基于`min_lidar_distance`的计算

