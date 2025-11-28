# 机器人到目标点距离的计算方法

## 概述

在 `navdp_generate_dataset.py` 中，机器人到目标点的距离会在运动过程中**实时计算并输出到终端**。本文档说明距离是如何计算的。

---

## 距离计算公式

### 基本原理

使用**欧几里得距离（Euclidean Distance）**公式，计算二维平面上两点之间的直线距离：

```
distance = √[(goal_x - robot_x)² + (goal_y - robot_y)²]
```

### 代码实现

在 `navdp_generate_dataset.py` 中有两处计算距离：

#### 1. **定期进度输出**（每20个数据点输出一次）

**位置**：`sync_callback()` 函数，第1114-1120行

```python
# 定期输出进度
if len(self.episode_data) % 20 == 0:
    if self.current_goal_type == 'point' and self.episode_goal:
        distance = math.hypot(
            robot_pose['x'] - self.episode_goal['x'],
            robot_pose['y'] - self.episode_goal['y']
        )
        rospy.loginfo(f"[数据收集器] Episode进度: {len(self.episode_data)}步, 目标距离: {distance:.3f}m, 当前位置: ({robot_pose['x']:.3f}, {robot_pose['y']:.3f})")
```

**说明**：
- `robot_pose['x']`, `robot_pose['y']`：机器人当前位置（通过TF查询map坐标系）
- `self.episode_goal['x']`, `self.episode_goal['y']`：目标点位置（在map坐标系中）
- `math.hypot()`：Python的数学函数，计算两点之间的欧几里得距离（等价于 `√(dx² + dy²)`）

#### 2. **导航语义计算**（每次数据收集时计算）

**位置**：`_compute_navigation_semantics()` 函数，第1302-1304行

```python
if self.current_goal_type == 'point' and self.episode_goal:
    dx = self.episode_goal['x'] - robot_pose['x']
    dy = self.episode_goal['y'] - robot_pose['y']
    semantics['distance_to_goal'] = float(math.hypot(dx, dy))
```

**说明**：
- 这个距离被存储在 `nav_semantics['distance_to_goal']` 中
- 会保存到每个数据点的元数据中
- 用于后续的数据分析和训练

---

## 坐标系统

### 统一使用 map 坐标系

两个脚本都使用 **map 坐标系**进行计算：

1. **机器人位置**：
   - 通过 TF 查询 `map -> base_link` 变换获取
   - 在 `_extract_robot_pose()` 函数中实现（第1130-1154行）
   - 使用 `UnifiedPoseGetter` 工具类

2. **目标位置**：
   - 从 `/move_base/goal` (MoveBaseActionGoal) topic 接收
   - 如果目标不在 map 坐标系，会通过 TF 转换到 map 坐标系
   - 在 `point_goal_callback()` 函数中处理（第492-519行）

### 为什么使用 map 坐标系？

- **全局一致性**：map 坐标系是全局固定的，不会漂移
- **准确性**：避免了 odom 坐标系的累积误差
- **统一性**：机器人和目标都在同一坐标系中，计算距离更准确

---

## 输出频率和格式

### 输出时机

1. **定期输出**（主要输出）：
   - **频率**：每收集 **20个数据点** 输出一次
   - **位置**：`sync_callback()` 函数
   - **格式**：`[数据收集器] Episode进度: {步数}步, 目标距离: {距离:.3f}m, 当前位置: ({x:.3f}, {y:.3f})`

2. **第一个数据点**：
   - 输出一次，显示初始距离
   - **格式**：`[数据收集器] 距离目标: {距离:.3f}m`

3. **Episode结束时**：
   - 显示最终距离
   - 保存在episode元数据中

### 示例输出

```
[数据收集器] Episode进度: 20步, 目标距离: 15.234m, 当前位置: (10.123, 20.456)
[数据收集器] Episode进度: 40步, 目标距离: 12.567m, 当前位置: (11.234, 21.567)
[数据收集器] Episode进度: 60步, 目标距离: 9.123m, 当前位置: (12.345, 22.678)
...
```

---

## 调整输出频率

### 方法1：修改输出间隔

**位置**：`navdp_generate_dataset.py` 第1114行

```python
# 当前：每20个数据点输出一次
if len(self.episode_data) % 20 == 0:
    # ... 输出距离

# 改为每10个数据点输出一次：
if len(self.episode_data) % 10 == 0:
    # ... 输出距离

# 改为每5个数据点输出一次：
if len(self.episode_data) % 5 == 0:
    # ... 输出距离
```

### 方法2：基于时间间隔输出

可以添加基于时间的输出（例如每秒输出一次）：

```python
# 在 __init__() 中添加
self.last_distance_print_time = time.time()
self.distance_print_interval = 1.0  # 每秒输出一次

# 在 sync_callback() 中修改
current_time = time.time()
if current_time - self.last_distance_print_time > self.distance_print_interval:
    if self.current_goal_type == 'point' and self.episode_goal:
        distance = math.hypot(
            robot_pose['x'] - self.episode_goal['x'],
            robot_pose['y'] - self.episode_goal['y']
        )
        rospy.loginfo(f"[数据收集器] 目标距离: {distance:.3f}m, 当前位置: ({robot_pose['x']:.3f}, {robot_pose['y']:.3f})")
        self.last_distance_print_time = current_time
```

---

## 距离计算的关键代码

### 完整的计算流程

```python
# 1. 获取机器人当前位置（map坐标系）
robot_pose = self._extract_robot_pose(odom_msg)
# robot_pose = {'x': float, 'y': float, 'theta': float, 'timestamp': int}

# 2. 获取目标位置（map坐标系）
goal_x = self.episode_goal['x']  # 目标点x坐标
goal_y = self.episode_goal['y']  # 目标点y坐标

# 3. 计算距离
dx = goal_x - robot_pose['x']  # x方向差值
dy = goal_y - robot_pose['y']  # y方向差值
distance = math.hypot(dx, dy)  # 欧几里得距离

# 4. 输出或保存
rospy.loginfo(f"目标距离: {distance:.3f}m")
```

### math.hypot() 函数

`math.hypot()` 是 Python 标准库函数，用于计算欧几里得距离：

```python
import math

# 等价于：
distance = math.hypot(dx, dy)
# 等价于：
distance = math.sqrt(dx*dx + dy*dy)
```

**优势**：
- 更高效（避免溢出）
- 更精确（数值稳定性更好）
- 代码更简洁

---

## 其他相关距离计算

### 1. 目标生成器中的距离计算

在 `random_generate_goal.py` 中也有距离计算：

**位置**：`check_goal_reached_or_timeout()` 和 `generate_goal_timer_cb()` 函数

```python
# 计算到目标的距离（用于超时检查）
cx = self.current_pose_map.position.x
cy = self.current_pose_map.position.y
gx, gy = self.current_goal
goal_distance = np.hypot(cx - gx, cy - gy)
```

### 2. Goal Image 捕获时的距离检查

**位置**：`_check_and_capture_goal_image()` 函数，第1852-1855行

```python
# 计算到目标点的距离
dx = self.episode_goal['x'] - robot_pose['x']
dy = self.episode_goal['y'] - robot_pose['y']
distance = math.hypot(dx, dy)

# 检查距离是否在捕获范围内
if distance > self.goal_image_capture_distance:  # 默认3.0m
    return  # 距离太远，不捕获
```

---

## 验证距离计算

### 方法1：检查终端输出

运行脚本后，查看终端输出：

```bash
[数据收集器] Episode进度: 20步, 目标距离: 15.234m, 当前位置: (10.123, 20.456)
```

### 方法2：检查保存的数据

每个数据点中都保存了 `distance_to_goal`：

```python
# 在数据点的 nav_semantics 中
data_point['nav_semantics']['distance_to_goal']  # 距离值（米）
```

### 方法3：使用 ROS topic

可以订阅相关 topic 查看机器人位置和目标位置：

```bash
# 查看机器人位置（通过TF）
rosrun tf tf_echo map base_link

# 查看当前目标
rostopic echo /move_base/goal
```

---

## 常见问题

### Q1: 为什么距离有时候会先增加再减少？

**原因**：
- 机器人可能先绕行障碍物，导致距离暂时增加
- 路径规划可能选择了更长的路径
- 这是正常现象，最终距离会减少

### Q2: 距离计算的精度如何？

**精度**：
- 使用 `float` 类型，精度约为 7 位有效数字
- 输出格式：`.3f`（保留3位小数），约1毫米精度
- 实际精度取决于地图分辨率和TF变换精度

### Q3: 距离单位是什么？

**单位**：**米（m）**

所有坐标和距离都使用国际单位制（SI）：
- 位置坐标：米
- 距离：米
- 速度：米/秒

### Q4: 如何实时监控距离变化？

**方法**：
1. 查看终端日志输出（每20个数据点输出一次）
2. 使用 ROS 工具查看保存的数据
3. 修改代码增加输出频率（见上文"调整输出频率"）

---

## 总结

### 距离计算要点

1. ✅ **公式**：欧几里得距离 `distance = √[(x₁-x₂)² + (y₁-y₂)²]`
2. ✅ **实现**：`math.hypot(dx, dy)`
3. ✅ **坐标系**：统一使用 map 坐标系
4. ✅ **输出频率**：每20个数据点输出一次
5. ✅ **精度**：保留3位小数（约1毫米精度）

### 代码位置

- **主要输出**：`navdp_generate_dataset.py` 第1114-1120行
- **距离计算**：`_compute_navigation_semantics()` 第1302-1304行
- **机器人位置**：`_extract_robot_pose()` 第1130-1154行

---

**文档创建时间**：2024年  
**相关文档**：
- `COMPLETE_WORKFLOW_EXPLANATION.md`
- `BASE_LINK_COORDINATE_UPDATE.md`
- `GET_ROBOT_POSE_ROS1.md`

