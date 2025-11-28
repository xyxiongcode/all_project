# 目标超时检查修复说明

## 问题描述

**现象**：
- 机器人发布目标后会生成轨迹
- 机器人在沿轨迹行走过程中，有时候ROS time过了30s，有时候过了60s，会发布新的目标点
- 导致机器人在路中间不得不重新规划到新目标点的轨迹

**根本原因**：
1. 定时器每15秒检查一次超时
2. 超时检查只判断时间是否超过阈值，没有充分考虑机器人是否还在正常导航
3. 移动检测逻辑不够完善，只检查最近一次位移，可能误判

---

## 修复方案

### 1. 改进的移动检测机制

#### 新增状态变量
```python
self.last_movement_time = rospy.Time(0)  # 最后一次移动的时间
self.goal_start_pose = None  # 发布目标时的机器人位置
self.goal_start_time = rospy.Time(0)  # 发布目标时的时间
```

#### 在里程计回调中记录移动时间
```python
# 在 odom_callback 中
if dist > 0.05:  # 移动超过5cm
    self.last_movement_time = now
```

#### 在发布目标时记录起始状态
```python
# 在 sample_and_publish_goal 中
if self.current_pose_map is not None:
    self.goal_start_pose = ...  # 记录起始位置
    self.goal_start_time = rospy.Time.now()
    self.last_movement_time = rospy.Time.now()
```

---

### 2. 三重超时检查机制

在 `generate_goal_timer_cb()` 中，判定超时前会进行**三重检查**：

#### 检查1：最近移动时间
```python
time_since_last_movement = (now - self.last_movement_time).to_sec()
recent_movement_threshold = 10.0  # 10秒内有移动

if time_since_last_movement < recent_movement_threshold:
    # 机器人最近有移动，延长超时检查
    return
```

**说明**：如果机器人最近10秒内有移动（>5cm），认为还在正常导航，不判定为超时。

---

#### 检查2：总移动距离
```python
if self.goal_start_pose is not None:
    total_dx = current_x - goal_start_x
    total_dy = current_y - goal_start_y
    total_distance_moved = np.hypot(total_dx, total_dy)
    
    if total_distance_moved > 1.0:  # 移动超过1米
        # 机器人在正常导航，延长超时检查
        return
```

**说明**：如果从目标发布到现在，机器人总共移动了超过1米，说明在正常导航，不判定为超时。

---

#### 检查3：剩余距离变化
```python
if self.goal_start_pose is not None:
    initial_distance = 从起始位置到目标的距离
    current_distance = 从当前位置到目标的距离
    
    if current_distance < initial_distance * 0.9:  # 至少减少了10%
        # 机器人正在接近目标，延长超时检查
        return
```

**说明**：如果剩余距离比初始距离减少了至少10%，说明机器人在接近目标，不判定为超时。

---

### 3. 动态超时时间计算

根据目标距离动态计算超时时间：

```python
def _calculate_goal_timeout(self, goal_distance):
    # 基础时间 = 距离 / 平均速度
    base_time = goal_distance / self.avg_robot_speed  # 默认0.5 m/s
    
    # 加上缓冲时间（50%）
    timeout = base_time * 1.5
    
    # 限制在最小和最大超时时间之间
    timeout = max(30.0, min(120.0, timeout))
    
    return timeout
```

**示例**：
- 目标距离 10米：超时时间 = (10/0.5) × 1.5 = 30秒（最小30秒）
- 目标距离 30米：超时时间 = (30/0.5) × 1.5 = 90秒
- 目标距离 100米：超时时间 = (100/0.5) × 1.5 = 300秒 → 限制为120秒（最大120秒）

---

## 修复效果

### 修复前
- 固定30秒超时，无论目标距离
- 只检查最近一次位移，容易误判
- 机器人在正常导航时可能被判定为超时

### 修复后
- 根据目标距离动态计算超时时间
- 三重检查机制，充分判断机器人是否在正常导航
- 只有在机器人真正卡死或目标不可达时才判定为超时

---

## 参数配置

### 可调参数

```python
# 超时时间参数
goal_timeout_min = 30.0  # 最小超时时间（秒）
goal_timeout_max = 120.0  # 最大超时时间（秒）
avg_robot_speed = 0.5  # 机器人平均速度（m/s）

# 移动检测参数（代码中硬编码，可根据需要调整）
recent_movement_threshold = 10.0  # 最近移动时间阈值（秒）
total_movement_threshold = 1.0  # 总移动距离阈值（米）
distance_reduction_threshold = 0.9  # 距离减少比例阈值（90%）
```

### 调整建议

如果机器人速度较慢：
```bash
rosparam set /optimized_random_goal_generator/avg_robot_speed 0.3
```

如果机器人速度较快：
```bash
rosparam set /optimized_random_goal_generator/avg_robot_speed 0.8
```

如果需要更长的超时时间：
```bash
rosparam set /optimized_random_goal_generator/goal_timeout_max 180.0
```

---

## 调试日志

修复后的代码会输出详细的超时检查信息：

```
[目标生成器] 机器人最近5.2s内有移动，剩余距离=15.32m，延长超时检查
```

或者：

```
============================================================
[目标生成器] 目标超时，重新采样
[目标生成器] 已用时间: 95.3s，动态超时阈值: 90.0s
[目标生成器] 剩余距离: 12.45m
[目标生成器] 距离上次移动: 25.6s
============================================================
```

这些日志可以帮助诊断为什么会在中途发布新目标。

---

## 注意事项

1. **三重检查是"或"的关系**：只要满足任一条件，就不判定为超时
2. **动态超时时间**：根据目标距离计算，但受最小/最大限制
3. **移动检测精度**：移动检测基于里程计数据，如果里程计不准确，可能影响判断
4. **卡死检测**：超时检查与卡死检测（`stuck_check_cb`）是独立的，卡死检测仍然有效

---

## 测试建议

1. **观察日志**：查看超时检查的日志输出，确认逻辑是否正确
2. **调整参数**：根据实际机器人速度调整 `avg_robot_speed`
3. **监控行为**：观察机器人是否还会在正常导航时被中断

如果问题仍然存在，可以：
- 增加 `goal_timeout_max` 的值
- 调整 `recent_movement_threshold`（在代码中修改）
- 检查里程计数据是否准确

