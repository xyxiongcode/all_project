# NavDP数据收集器采集的数据类型总结

## 概述

`navdp_generate_dataset.py` 脚本用于收集机器人导航任务的训练数据，采用基于episode的数据组织结构。每个episode对应一次完整的导航任务（从接收目标到到达目标或失败）。

---

## 一、数据源（ROS Topics订阅）

### 1. 传感器数据（时间同步）
- **`/rgb_left`** (Image) - 左RGB相机图像
- **`/depth_left`** (Image) - 左深度相机图像
- **`/odom`** (Odometry) - 里程计数据（位姿和速度）
- **`/scan`** (LaserScan) - LiDAR扫描数据
- **`/imu`** (Imu) - 惯性测量单元数据

### 2. 控制命令
- **`/cmd_vel`** (Twist) - 速度控制命令

### 3. 导航目标
- **`/move_base/goal`** (MoveBaseActionGoal) - 通过Action Client发送的导航目标

### 4. 其他信息
- **`/camera_info_left`** (CameraInfo) - 相机内参（用于图像处理）
- **`/map`** (OccupancyGrid) - 占用栅格地图（用于视线检查和路径规划验证）

---

## 二、单个数据点（Data Point）结构

每个时间步采集的数据点包含以下内容：

### 1. 时间戳和索引
```python
{
    'timestamp': int,          # ROS时间戳（纳秒）
    'step_index': int          # 在当前episode中的步数索引
}
```

### 2. 图像数据
- **`rgb`**: 处理后的RGB图像 (224x224, BGR格式, numpy array)
- **`rgb_original`**: 原始RGB图像（完整尺寸，用于goal_image捕获）
- **`depth`**: 处理后的深度图像 (224x224, 深度值范围[0.1, 3.0]米, numpy array)

### 3. 机器人状态
```python
{
    'robot_pose': {
        'x': float,            # map坐标系中的x坐标
        'y': float,            # map坐标系中的y坐标
        'theta': float,        # 朝向角度（弧度）
        'timestamp': int       # 位姿时间戳
    },
    'odom_msg': Odometry,      # 完整的里程计消息
    'lidar_msg': LaserScan,    # 完整的LiDAR扫描消息
    'imu_msg': Imu             # 完整的IMU消息
}
```

### 4. 动作数据（控制命令）
```python
{
    'action': {
        'linear_x': float,     # 线速度x分量（m/s）
        'linear_y': float,     # 线速度y分量（m/s）
        'angular_z': float     # 角速度z分量（rad/s）
    }
}
```

### 5. 目标表示（Goal Representation）
支持三种目标表示方式：

#### 5.1 点目标（Point Goal）
```python
{
    'point': [
        robot_dx,              # 目标在机器人坐标系中的x坐标（米）
        robot_dy,              # 目标在机器人坐标系中的y坐标（米）
        dtheta                 # 目标相对于机器人的朝向差（弧度）
    ]
}
```

#### 5.2 像素目标（Pixel Goal）
```python
{
    'pixel': numpy.array       # 224x224的热力图，目标位置标记为1.0
}
```

#### 5.3 图像目标（Image Goal）
```python
{
    'image': numpy.array       # 目标图像（BGR格式，224x224）
}
```

### 6. 导航语义信息（Navigation Semantics）
```python
{
    'nav_semantics': {
        'has_goal': bool,                          # 是否有目标
        'distance_to_goal': float,                 # 到目标的欧几里得距离（米）
        'heading_error': float,                    # 朝向误差（弧度）
        'min_lidar_distance': float,               # 最近的障碍物距离（米）
        'safety_score': float                      # 安全分数（0-1，基于最近障碍物距离）
    }
}
```

### 7. 其他元数据
```python
{
    'has_sufficient_history': bool,    # 是否有足够的历史数据（≥memory_size帧）
    'move_base_status': str,           # move_base状态（PENDING/ACTIVE/SUCCEEDED等）
    'split_type': str                  # 数据分割类型（'train'或'test'）
}
```

---

## 三、Episode级别的数据

每个episode包含：

### 1. Episode元数据（metadata.json）
```json
{
    "episode_id": int,
    "start_time": float,               # 系统时间戳（秒）
    "end_time": float,                 # 系统时间戳（秒）
    "duration": float,                 # 持续时间（秒）
    "goal_type": str,                  # 'point', 'image', 'pixel'
    "goal_data": {                     # 目标数据
        "type": str,
        "x": float,                    # map坐标系
        "y": float,                    # map坐标系
        "theta": float,
        "stamp": int,
        "original_frame": str
    },
    "total_steps": int,                # 总步数
    "success": bool,                   # 是否成功到达目标
    "termination_reason": str,         # 终止原因
    "move_base_status": str,           # move_base最终状态
    "split_type": str,                 # 'train'或'test'
    "camera_info": {
        "K": [float, ...],             # 相机内参矩阵
        "D": [float, ...]              # 畸变系数
    },
    "time_info": {
        "ros_start_time": float,
        "system_start_time": float,
        "system_end_time": float
    },
    "goal_image": {
        "captured": bool,              # 是否捕获了goal_image
        "path": str,                   # goal_image路径
        "capture_pose": {              # 捕获时的机器人位姿
            "x": float,
            "y": float,
            "theta": float,
            "distance_to_goal": float,
            "capture_reason": str
        },
        "capture_distance_threshold": float
    }
}
```

### 2. Episode目录结构
```
episode_<episode_id>/
├── metadata.json              # Episode元数据
├── goal_image.png             # 目标图像（如果捕获成功）
├── step_0000/
│   ├── rgb.png                # RGB图像（224x224）
│   ├── depth.npy              # 深度图像（224x224）
│   └── data.json              # 该步的所有数据（JSON格式）
├── step_0001/
│   ├── rgb.png
│   ├── depth.npy
│   └── data.json
└── ...
```

### 3. 每个step的数据（data.json）
```json
{
    "timestamp": int,
    "step_index": int,
    "robot_pose": {
        "x": float,
        "y": float,
        "theta": float,
        "timestamp": int
    },
    "action": {
        "linear_x": float,
        "linear_y": float,
        "angular_z": float
    },
    "goal_representation": {
        "point": [float, float, float],
        "pixel": [[...]],              # 224x224数组
        "image": null
    },
    "nav_semantics": {
        "has_goal": bool,
        "distance_to_goal": float,
        "heading_error": float,
        "min_lidar_distance": float,
        "safety_score": float
    },
    "has_sufficient_history": bool,
    "move_base_status": str,
    "split_type": str
}
```

---

## 四、训练样本（Training Samples）结构

训练样本是从episode数据中生成的，用于NavDP模型训练：

### 1. 训练样本结构
```python
{
    'episode_id': int,
    'step_index': int,                 # 生成样本的时间步索引
    'timestamp': int,
    
    # 历史序列（memory_size帧，默认8帧）
    'rgb_sequence': [                  # 历史RGB图像序列
        numpy.array,                   # 每帧224x224
        ...
    ],
    'depth_sequence': [                # 历史深度图像序列
        numpy.array,                   # 每帧224x224
        ...
    ],
    
    # 目标表示
    'point_goal': [float, float, float],
    'image_goal': numpy.array or None,
    'pixel_goal': numpy.array or None,
    
    # 机器人状态
    'robot_pose': {
        'x': float,
        'y': float,
        'theta': float
    },
    'velocity': {
        'linear_x': float,
        'linear_y': float,
        'angular_z': float
    },
    
    # 监督信号（未来动作序列，predict_size步，默认24步）
    'action_sequence': [
        [linear_x, linear_y, angular_z],
        ...
    ],
    
    # 语义信息
    'nav_semantics': {
        'has_goal': bool,
        'distance_to_goal': float,
        'heading_error': float,
        'min_lidar_distance': float,
        'safety_score': float
    },
    
    'move_base_status': str,
    'split_type': str
}
```

### 2. 训练样本目录结构
```
samples/
├── sample_<episode_id>_<sample_index>/
│   ├── metadata.json                  # 样本元数据
│   ├── rgb_00.png                     # 历史序列第0帧
│   ├── rgb_01.png                     # 历史序列第1帧
│   ├── ...
│   ├── rgb_07.png                     # 历史序列第7帧（memory_size-1）
│   ├── depth_sequence.npy             # 深度序列（8x224x224）
│   ├── image_goal.png                 # 图像目标（如果有）
│   ├── pixel_goal.npy                 # 像素目标热力图
│   └── action_sequence.npy            # 未来动作序列（24x3）
```

---

## 五、CSV索引文件

### 1. Episode CSV（train_episodes.csv / test_episodes.csv）
列名：
- `episode_id` - Episode ID
- `start_time` - 开始时间（系统时间戳）
- `end_time` - 结束时间（系统时间戳）
- `duration` - 持续时间（秒）
- `goal_type` - 目标类型
- `total_steps` - 总步数
- `success` - 是否成功
- `failure_reason` - 失败原因
- `min_obstacle_distance` - 最小障碍物距离
- `average_velocity` - 平均速度
- `goal_distance_start` - 起始时距离目标
- `goal_distance_end` - 结束时距离目标
- `episode_path` - Episode目录路径
- `move_base_status` - move_base状态
- `split_type` - 分割类型

### 2. Sample CSV（train_samples.csv / test_samples.csv）
列名：
- `episode_id` - Episode ID
- `step_index` - 时间步索引
- `timestamp` - 时间戳
- `rgb_path` - RGB图像路径
- `depth_path` - 深度序列路径
- `point_goal_x`, `point_goal_y`, `point_goal_theta` - 点目标坐标
- `image_goal_path` - 图像目标路径
- `pixel_goal_path` - 像素目标路径
- `robot_x`, `robot_y`, `robot_theta` - 机器人位姿
- `velocity_x`, `velocity_y`, `velocity_theta` - 速度
- `action_linear_x`, `action_linear_y`, `action_angular_z` - 动作（当前步）
- `has_goal` - 是否有目标
- `distance_to_goal` - 距离目标
- `heading_error` - 朝向误差
- `min_lidar_distance` - 最小LiDAR距离
- `safety_score` - 安全分数
- `move_base_status` - move_base状态
- `split_type` - 分割类型

---

## 六、数据处理参数

### 1. 图像处理
- **RGB图像尺寸**: 224x224像素
- **深度图像尺寸**: 224x224像素
- **深度范围**: [0.1, 3.0]米（裁剪和归一化）

### 2. 历史数据
- **memory_size**: 8帧（历史序列长度）
- **predict_size**: 24步（未来动作预测长度）

### 3. Goal Image捕获
- **捕获距离阈值**: 默认3.0米（可配置）
- **Fallback阈值**: 4.0米（episode结束时）
- **视线检查**: 使用Bresenham算法检查地图上的遮挡

---

## 七、数据收集流程

1. **接收目标** → 通过`/move_base/goal`订阅接收到新目标
2. **开始Episode** → 创建episode目录，重置状态
3. **数据采集循环**：
   - 时间同步接收传感器数据（RGB、深度、里程计、LiDAR、IMU）
   - 数据质量检查（时间同步、深度质量、LiDAR质量）
   - 提取机器人位姿（通过TF查询map坐标系）
   - 计算目标表示（点/图像/像素）
   - 计算导航语义信息（距离、朝向误差、障碍物等）
   - 检查是否需要捕获goal_image
   - 保存数据点到episode_data列表
4. **Episode终止** → 检测到达目标、失败、超时等条件
5. **数据保存** → 保存所有step数据、元数据、goal_image
6. **生成训练样本** → 从episode数据生成NavDP训练样本
7. **更新CSV索引** → 更新episode和sample的CSV文件

---

## 八、数据质量检查

### 1. 时间同步
- RGB和深度图像时间戳差异 < 0.1秒

### 2. 深度数据质量
- 有效深度像素比例 ≥ 30%

### 3. LiDAR数据质量
- 有效LiDAR扫描点比例 ≥ 30%

### 4. 最小数据要求
- 最小episode步数：5步
- 生成训练样本需要：至少 `memory_size + predict_size` 步数据

---

## 九、坐标系统

### 使用的坐标系
- **map**: 全局地图坐标系（用于机器人位姿和目标位置）
- **base_link**: 机器人本体中心（通过TF查询 map->base_link 变换）

### 距离计算
- **当前实现**: 欧几里得距离（直线距离）
- **注意**: 这不考虑路径规划中的绕路情况，实际距离可能更长

---

## 十、数据组织

### 目录结构
```
<save_dir>/
├── train/
│   ├── episodes/
│   │   ├── episode_<id1>/
│   │   ├── episode_<id2>/
│   │   └── ...
│   └── samples/
│       ├── sample_<episode_id>_<index>/
│       └── ...
├── test/
│   ├── episodes/
│   └── samples/
├── train_episodes.csv
├── test_episodes.csv
├── train_samples.csv
├── test_samples.csv
└── dataset_statistics.json
```

---

## 总结

`navdp_generate_dataset.py` 采集了以下核心数据：

1. **多模态传感器数据**: RGB图像、深度图像、LiDAR、IMU、里程计
2. **机器人状态**: 位姿（map坐标系）、速度、朝向
3. **控制动作**: 速度命令（线速度和角速度）
4. **目标表示**: 点目标（相对坐标）、像素目标（热力图）、图像目标
5. **导航语义**: 距离、朝向误差、障碍物距离、安全分数
6. **Episode元数据**: 成功/失败、持续时间、终止原因等
7. **训练样本**: 历史序列 + 未来动作序列（用于监督学习）

这些数据可用于训练NavDP（Navigation Diffusion Policy）等端到端导航策略模型。

