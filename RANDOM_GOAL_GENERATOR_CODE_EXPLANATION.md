# random_generate_goal.py 脚本逻辑详解（带原代码）

本文档按照代码运行顺序，附带原代码片段，详细解释 `random_generate_goal.py` 脚本的逻辑。

---

## 一、程序入口和导入

### 1.1 脚本头部和导入

```python
#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
import random
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs

# === 新增：导入 IMU 消息类型 ===
from sensor_msgs.msg import Imu

# === 可选：Isaac Sim 的 ResetPoses.action（如果没有这个包，会自动回退 /initialpose） ===
try:
    from isaac_sim.msg import ResetPosesAction, ResetPosesGoal
    import actionlib
    HAS_RESET_ACTION = True
except Exception:
    ResetPosesAction = ResetPosesGoal = None
    actionlib = None
    HAS_RESET_ACTION = False

from tf.transformations import euler_from_quaternion, quaternion_from_euler
```

**解释**：
- 导入ROS核心库（rospy）和消息类型
- 导入数学库（numpy）和随机数生成
- 尝试导入Isaac Sim的ResetPoses Action，如果失败则设置标志为False，后续会回退到使用 `/initialpose` 话题

### 1.2 主程序入口

```python
if __name__ == "__main__":
    try:
        OptimizedRandomGoalGenerator().run()
    except rospy.ROSInterruptException:
        pass
```

**执行顺序**：
1. 创建 `OptimizedRandomGoalGenerator` 对象 → 触发 `__init__()` 方法
2. 调用 `run()` 方法 → 进入ROS主循环

---

## 二、初始化阶段（`__init__` 方法）

### 2.1 ROS节点初始化

```python
def __init__(self):
    rospy.init_node('optimized_random_goal_generator')
    self.make_plan = None
    self.map_ready = None
    self.first_goal_sent = False
```

**解释**：
- `rospy.init_node()` 创建ROS节点，节点名为 `optimized_random_goal_generator`
- 初始化一些状态变量为 `None` 或 `False`

### 2.2 加载ROS参数

```python
# ===== 参数 =====
self.map_topic = rospy.get_param('~map_topic', '/map')
self.goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')
self.odom_topic = rospy.get_param('~odom_topic', '/odom')

# ===== 新增：IMU 话题参数 =====
self.imu_topic = rospy.get_param('~imu_topic', '/imu')

# 距离参数
self.min_distance = rospy.get_param('~min_distance', 0.5)  # m
self.max_distance = rospy.get_param('~max_distance', 200.0)  # m

# 障碍物距离偏好
self.min_obstacle_distance = rospy.get_param('~min_obstacle_distance', 0.5)  # m
self.max_obstacle_distance = rospy.get_param('~max_obstacle_distance', 1.0)  # m
self.empty_space_discard_prob = rospy.get_param('~empty_space_discard_prob', 0.8)

self.goal_period = rospy.get_param('~goal_period', 15.0)  # s
self.goal_timeout = rospy.get_param('~goal_timeout', 30.0)  # s
self.reach_tolerance = rospy.get_param('~reach_tolerance', 0.5)  # m

# 可达性检测
self.make_plan_srv_name = rospy.get_param('~make_plan_srv', '/move_base/make_plan')
self.plan_tolerance = rospy.get_param('~plan_tolerance', 0.2)  # m
self.min_plan_length = rospy.get_param('~min_plan_length', 0.5)  # m

# TF 设置
self.use_tf = rospy.get_param('~use_tf', True)
self.map_frame = rospy.get_param('~map_frame', 'map')

# ===== 新增：卡死/侧翻检测 & 恢复相关参数 =====
self.enable_stuck_check = rospy.get_param('~enable_stuck_check', True)
self.stuck_distance_threshold = rospy.get_param('~stuck_distance_threshold', 0.05)  # m
self.stuck_time_threshold = rospy.get_param('~stuck_time_threshold', 45.0)  # s
self.rollover_angle_deg = rospy.get_param('~rollover_angle_deg', 30.0)  # 度
self.recovery_cooldown = rospy.get_param('~recovery_cooldown', 10.0)  # s
self.stuck_goal_grace = rospy.get_param('~stuck_goal_grace', 5.0)  # s
self.initial_pose_topic = rospy.get_param('~initial_pose_topic', '/initialpose')
self.reset_action_name = rospy.get_param('~reset_action', '/reset')
```

**解释**：
- 使用 `rospy.get_param()` 从ROS参数服务器加载配置，如果参数不存在则使用默认值
- `~` 前缀表示私有参数（相对于节点命名空间）
- 参数包括：话题名称、距离范围、障碍物偏好、时间阈值、卡死检测参数等

### 2.3 创建订阅者和发布者

```python
# ===== 订阅 / 发布 =====
self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=50)

# ===== 新增：IMU 订阅 =====
self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=50)

self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
self.initial_pose_pub = rospy.Publisher(self.initial_pose_topic, PoseWithCovarianceStamped, queue_size=1)
```

**解释**：
- **订阅者**：订阅地图、里程计、IMU数据，当收到消息时触发对应的回调函数
- **发布者**：发布目标点到 `/move_base_simple/goal`，发布初始位姿到 `/initialpose`

### 2.4 初始化状态变量

```python
# ===== 变量 =====
self.map_data = None
self.map_info = None
self.current_pose_map = None
self.current_goal = None
self.last_goal_time = rospy.Time(0)

# ===== 新增：卡死监测状态 =====
self.last_pose = None
self.last_pose_time = rospy.Time(0)
self.stuck_start_time = None
self.is_stuck = False
self._last_recovery_ts = rospy.Time(0)
self._last_goal_publish_ts = rospy.Time(0)

# ===== 新增：IMU 侧翻状态 =====
self.rollover_detected = False
self.last_imu_time = rospy.Time(0)
```

**解释**：
- 初始化所有状态变量，包括地图数据、当前位姿、目标点、卡死检测相关状态等

### 2.5 初始化TF变换

```python
# TF
self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
```

**解释**：
- 创建TF缓冲区（缓存10秒内的变换）和监听器
- 用于将里程计坐标系（odom）下的位姿转换为地图坐标系（map）下的位姿

### 2.6 连接ResetPoses Action（如果可用）

```python
# ===== 新增：ResetPoses Action 客户端（如果可用）=====
self.reset_client = None
if HAS_RESET_ACTION:
    try:
        self.reset_client = actionlib.SimpleActionClient(self.reset_action_name, ResetPosesAction)
        if self.reset_client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo(f"[reset] 已连接 ResetPoses action: {self.reset_action_name}")
        else:
            rospy.logwarn(f"[reset] 未连接到 {self.reset_action_name}，将回退 /initialpose")
            self.reset_client = None
    except Exception as e:
        rospy.logwarn(f"[reset] 连接 {self.reset_action_name} 异常：{e}")
        self.reset_client = None
else:
    rospy.logwarn("[reset] 未找到 isaac_sim/ResetPosesAction，复位将使用 /initialpose")
```

**解释**：
- 如果Isaac Sim支持ResetPoses Action，尝试连接
- 如果连接失败或不可用，后续会使用 `/initialpose` 话题进行复位

### 2.7 创建定时器

```python
# 定时器
self.timer = rospy.Timer(rospy.Duration(self.goal_period), self.generate_goal_timer_cb)
# 新增：卡死周期检查（不会影响你的采样逻辑）
self.stuck_timer = rospy.Timer(rospy.Duration(1.0), self.stuck_check_cb)
```

**解释**：
- **目标生成定时器**：每15秒（`goal_period`）触发一次 `generate_goal_timer_cb`
- **卡死检测定时器**：每1秒触发一次 `stuck_check_cb`

### 2.8 输出启动日志

```python
rospy.loginfo("=" * 60)
rospy.loginfo("[目标生成器] 优化版随机目标生成器已启动")
rospy.loginfo(f"[目标生成器] 目标距离范围: {self.min_distance}-{self.max_distance}m")
rospy.loginfo(f"[目标生成器] 障碍物偏好: {self.min_obstacle_distance}-{self.max_obstacle_distance}m范围内")
rospy.loginfo(f"[目标生成器] 空旷区域丢弃概率: {self.empty_space_discard_prob * 100}%")
rospy.loginfo(f"[目标生成器] 卡死检测: 距离<{self.stuck_distance_threshold}m 且时间>{self.stuck_time_threshold}s；侧翻阈值≈{self.rollover_angle_deg}°")
rospy.loginfo(f"[目标生成器] 使用IMU话题进行侧翻检测: {self.imu_topic}")
rospy.loginfo(f"[目标生成器] 目标发布话题: {self.goal_topic}")
rospy.loginfo("[目标生成器] 等待地图和位姿数据，准备生成目标...")
rospy.loginfo("=" * 60)
```

**解释**：
- 输出配置信息，方便调试和确认参数设置

---

## 三、运行时回调函数

### 3.1 地图回调函数（`map_callback`）

```python
def map_callback(self, msg: OccupancyGrid):
    self.map_info = msg.info
    self.map_data = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
    self.map_ready = True
    rospy.loginfo_once(
        "收到地图：{}x{}，分辨率={:.3f} m".format(msg.info.width, msg.info.height, msg.info.resolution))
```

**执行流程**：
1. 保存地图信息（分辨率、原点、尺寸等）到 `self.map_info`
2. 将地图数据（一维数组）转换为二维numpy数组，形状为 `(height, width)`
3. 设置 `self.map_ready = True`，表示地图已准备好
4. 输出一次日志，显示地图尺寸和分辨率

**触发时机**：收到 `/map` 话题消息时（通常只在启动时或地图更新时）

### 3.2 IMU回调函数（`imu_callback`）

```python
def imu_callback(self, msg: Imu):
    """使用IMU数据进行更准确的侧翻检测"""
    self.last_imu_time = rospy.Time.now()

    # 从IMU消息中提取四元数
    orientation_q = msg.orientation
    quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    try:
        # 将四元数转换为欧拉角（roll, pitch, yaw）
        roll, pitch, _ = euler_from_quaternion(quaternion)

        # 转换为角度
        roll_deg = abs(np.degrees(roll))
        pitch_deg = abs(np.degrees(pitch))

        # 侧翻检测
        if roll_deg > self.rollover_angle_deg or pitch_deg > self.rollover_angle_deg:
            if not self.rollover_detected:
                rospy.logwarn(f"IMU检测到侧翻! Roll: {roll_deg:.1f}°, Pitch: {pitch_deg:.1f}°")
                self.rollover_detected = True
        else:
            if self.rollover_detected:
                rospy.loginfo("IMU姿态恢复正常")
            self.rollover_detected = False

    except Exception as e:
        rospy.logwarn_throttle(5.0, f"IMU数据处理异常: {e}")
```

**执行流程**：
1. 记录当前时间到 `self.last_imu_time`
2. 从IMU消息中提取四元数（姿态）
3. 将四元数转换为欧拉角（roll, pitch, yaw）
4. 计算 roll 和 pitch 的绝对值（度）
5. **侧翻检测**：
   - 如果 roll 或 pitch > 30°，且之前未检测到侧翻，则标记为侧翻并输出警告
   - 如果角度恢复正常，则清除侧翻标志

**触发时机**：收到 `/imu` 话题消息时（通常10-100Hz）

### 3.3 里程计回调函数（`odom_callback`）

```python
def odom_callback(self, msg: Odometry):
    pose_stamped = PoseStamped()
    pose_stamped.header = msg.header
    pose_stamped.pose = msg.pose.pose

    if self.use_tf:
        try:
            # 更稳：用最新变换（Time(0)），避免时间外推问题
            tfm = self.tf_buffer.lookup_transform(self.map_frame,
                                                  pose_stamped.header.frame_id,
                                                  rospy.Time(0),
                                                  rospy.Duration(0.2))
            pose_map = tf2_geometry_msgs.do_transform_pose(pose_stamped, tfm).pose
            self.current_pose_map = pose_map
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"TF 变换失败：{e}")
            self.current_pose_map = msg.pose.pose
    else:
        self.current_pose_map = msg.pose.pose
```

**步骤1：坐标变换**
- 将里程计坐标系（odom）下的位姿转换为地图坐标系（map）下的位姿
- 使用 `tf2_ros.Buffer.lookup_transform()` 查找变换
- `rospy.Time(0)` 表示使用最新的变换（避免时间外推问题）

```python
    # ===== 修改：位移检测（移除原有的姿态检测，改用IMU数据）=====
    now = rospy.Time.now()
    if self.enable_stuck_check:
        if self.last_pose is not None:
            dx = self.current_pose_map.position.x - self.last_pose.position.x
            dy = self.current_pose_map.position.y - self.last_pose.position.y
            dist = np.hypot(dx, dy)
            # 距离阈值（未移动）
            if dist < self.stuck_distance_threshold:
                if self.stuck_start_time is None:
                    self.stuck_start_time = now
                elif (now - self.stuck_start_time).to_sec() > self.stuck_time_threshold:
                    self.is_stuck = True
            else:
                self.stuck_start_time = None
                self.is_stuck = False
```

**步骤2：卡死检测**
- 计算当前位置与上次位置的位移距离
- 如果位移 < 0.05m：
  - 如果 `stuck_start_time` 为 `None`，记录开始时间
  - 如果持续时间 > 45s，标记为卡死（`self.is_stuck = True`）
- 如果位移 >= 0.05m，清除卡死状态

```python
    self.last_pose = self.current_pose_map
    self.last_pose_time = now

    self.check_goal_reached_or_timeout()
```

**步骤3：更新状态并检查目标到达**
- 更新 `last_pose` 和 `last_pose_time`
- 调用 `check_goal_reached_or_timeout()` 检查是否到达目标

**触发时机**：收到 `/odom` 话题消息时（通常10-20Hz）

---

## 四、定时器回调函数

### 4.1 目标生成定时器（`generate_goal_timer_cb`）

```python
def generate_goal_timer_cb(self, _):
    if self.current_pose_map is None or self.map_data is None:
        return

    need_new = (self.current_goal is None)
    if not need_new and (rospy.Time.now() - self.last_goal_time).to_sec() > self.goal_timeout:
        rospy.logwarn("[目标生成器] 目标超时，重新采样")
        rospy.logwarn(f"[目标生成器] 超时时间: {self.goal_timeout:.1f}s")
        need_new = True

    if need_new:
        self.sample_and_publish_goal()
```

**执行流程**：
1. **检查前置条件**：如果地图或位姿未准备好，直接返回
2. **判断是否需要新目标**：
   - 如果没有当前目标（`self.current_goal is None`），需要新目标
   - 如果目标超时（发布时间超过30秒），需要新目标
3. **生成并发布目标**：如果需要新目标，调用 `sample_and_publish_goal()`

**触发时机**：每15秒（`goal_period`）

### 4.2 卡死检测定时器（`stuck_check_cb`）

```python
def stuck_check_cb(self, _):
    if not self.enable_stuck_check:
        return
    now = rospy.Time.now()
    # 复位冷却
    if (now - self._last_recovery_ts).to_sec() < self.recovery_cooldown:
        return
    # 发目标后的宽限：给局部规划一点时间起步
    if (now - self._last_goal_publish_ts).to_sec() < self.stuck_goal_grace:
        return
    # 里程计长时间不更新（通信/仿真暂停）
    if (now - self.last_pose_time).to_sec() > max(2 * self.stuck_time_threshold, 10.0):
        self.is_stuck = True

    # ===== 新增：检查IMU侧翻状态 =====
    if self.rollover_detected:
        self.is_stuck = True
        rospy.logwarn("IMU检测到侧翻，触发复位")

    if self.is_stuck:
        rospy.logwarn("检测到侧翻/卡死，执行复位")
        self.perform_reset()
```

**执行流程**：
1. **检查是否启用**：如果未启用卡死检测，直接返回
2. **检查冷却时间**：如果距离上次恢复 < 10秒，跳过（避免频繁复位）
3. **检查目标发布宽限时间**：如果刚发布目标 < 5秒，跳过（给机器人时间起步）
4. **检查里程计超时**：如果里程计长时间不更新（>90秒），标记为卡死
5. **检查IMU侧翻**：如果IMU检测到侧翻，标记为卡死
6. **执行复位**：如果检测到卡死，调用 `perform_reset()`

**触发时机**：每1秒

---

## 五、核心功能函数

### 5.1 目标到达检测（`check_goal_reached_or_timeout`）

```python
def check_goal_reached_or_timeout(self):
    if self.current_goal is None or self.current_pose_map is None:
        return
    cx = self.current_pose_map.position.x
    cy = self.current_pose_map.position.y
    gx, gy = self.current_goal
    dist = np.hypot(cx - gx, cy - gy)

    if dist < self.reach_tolerance:
        rospy.loginfo("=" * 60)
        rospy.loginfo("[目标生成器] 机器人已到达目标点")
        rospy.loginfo(f"[目标生成器] 当前位置: ({cx:.3f}, {cy:.3f})")
        rospy.loginfo(f"[目标生成器] 目标位置: ({gx:.3f}, {gy:.3f})")
        rospy.loginfo(f"[目标生成器] 距离误差: {dist:.3f}m (容忍度: {self.reach_tolerance:.3f}m)")
        rospy.loginfo("[目标生成器] Episode 完成，数据收集器将完成当前episode文件夹的保存")
        rospy.loginfo("[目标生成器] 准备生成下一个目标点，开始新的episode...")
        rospy.loginfo("=" * 60)
        self.current_goal = None
        # 给数据收集器一点时间完成当前episode的保存
        rospy.sleep(5)
        self.sample_and_publish_goal()
```

**执行流程**：
1. **检查前置条件**：如果没有目标或位姿，直接返回
2. **计算距离**：计算当前位置到目标点的欧氏距离
3. **判断是否到达**：如果距离 < 0.5m（`reach_tolerance`）：
   - 输出到达日志
   - 清空当前目标（`self.current_goal = None`）
   - 等待5秒，让数据收集器完成保存
   - 生成并发布新目标

**调用时机**：每次 `odom_callback` 时调用

### 5.2 随机目标生成（`generate_random_goal`）

```python
def generate_random_goal(self):
    if self.current_pose_map is None or self.map_data is None or self.map_info is None:
        return None

    cx = self.current_pose_map.position.x
    cy = self.current_pose_map.position.y

    for attempt in range(300):
        angle = random.uniform(0.0, 2.0 * np.pi)
        dist = random.uniform(self.min_distance, self.max_distance)

        gx = cx + dist * np.cos(angle)
        gy = cy + dist * np.sin(angle)

        if not self.is_within_map(gx, gy):
            continue

        obstacle_distance = self.get_min_obstacle_distance(gx, gy)
        if obstacle_distance is None:
            continue

        if obstacle_distance < self.min_obstacle_distance:
            continue
        elif obstacle_distance <= self.max_obstacle_distance:
            pass
        else:
            if random.random() < self.empty_space_discard_prob:
                continue

        if not self.is_reachable_from_current(gx, gy):
            continue

        rospy.logdebug(f"找到目标点: ({gx:.2f}, {gy:.2f}), 障碍物距离: {obstacle_distance:.2f}m")
        return gx, gy

    rospy.logwarn("在300次尝试后未找到合适目标")
    return None
```

**执行流程**：
1. **检查前置条件**：如果地图或位姿未准备好，返回 `None`
2. **获取当前位置**：`cx, cy`
3. **随机采样循环**（最多300次尝试）：
   - **生成随机候选点**：
     ```python
     angle = random.uniform(0.0, 2.0 * np.pi)  # 随机角度 [0, 2π]
     dist = random.uniform(self.min_distance, self.max_distance)  # 随机距离 [0.5, 200]m
     gx = cx + dist * np.cos(angle)  # 目标x坐标
     gy = cy + dist * np.sin(angle)  # 目标y坐标
     ```
   - **检查是否在地图内**：如果不在，跳过
   - **检查障碍物距离**：
     - 如果距离障碍物 < 0.5m，跳过（太靠近障碍物）
     - 如果距离障碍物在 0.5-1.0m，接受（理想范围）
     - 如果距离障碍物 > 1.0m，80%概率跳过（过于空旷）
   - **检查可达性**：调用路径规划服务，验证目标是否可达
   - **找到合适目标**：如果通过所有检查，返回目标坐标
4. **尝试失败**：如果300次尝试都失败，返回 `None`

### 5.3 辅助函数：地图边界检查（`is_within_map`）

```python
def is_within_map(self, x, y):
    if self.map_info is None:
        return False
    mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
    my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
    return (0 <= mx < self.map_info.width and 0 <= my < self.map_info.height)
```

**解释**：
- 将世界坐标 `(x, y)` 转换为地图坐标 `(mx, my)`
- 检查地图坐标是否在地图范围内

### 5.4 辅助函数：障碍物距离计算（`get_min_obstacle_distance`）

```python
def get_min_obstacle_distance(self, x, y):
    if self.map_info is None or self.map_data is None:
        return None
    mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
    my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
    if mx < 0 or my < 0 or mx >= self.map_info.width or my >= self.map_info.height:
        return None
    search_radius_cells = int(2.0 / self.map_info.resolution)
    min_distance = float('inf')
    for dx in range(-search_radius_cells, search_radius_cells + 1):
        for dy in range(-search_radius_cells, search_radius_cells + 1):
            check_x = mx + dx
            check_y = my + dy
            if (0 <= check_x < self.map_info.width and 0 <= check_y < self.map_info.height):
                if self.map_data[check_y, check_x] > 0:
                    obstacle_x = self.map_info.origin.position.x + check_x * self.map_info.resolution
                    obstacle_y = self.map_info.origin.position.y + check_y * self.map_info.resolution
                    distance = np.hypot(x - obstacle_x, y - obstacle_y)
                    if distance < min_distance:
                        min_distance = distance
    return min_distance if min_distance != float('inf') else 100.0
```

**执行流程**：
1. 将世界坐标转换为地图坐标
2. 计算搜索半径（2米范围内的单元格数）
3. 遍历周围单元格，查找障碍物（`map_data > 0`）
4. 计算目标点到每个障碍物的距离，返回最小距离
5. 如果没有障碍物，返回100.0（表示很安全）

### 5.5 辅助函数：可达性检查（`is_reachable_from_current`）

```python
def is_reachable_from_current(self, gx, gy):
    if self.make_plan is None:
        try:
            rospy.wait_for_service(self.make_plan_srv_name, timeout=15.0)
            self.make_plan = rospy.ServiceProxy(self.make_plan_srv_name, GetPlan)
        except:
            rospy.logwarn(f"无法连接服务: {self.make_plan_srv_name}")
            return False

    start = PoseStamped()
    start.header.stamp = rospy.Time.now()
    start.header.frame_id = self.map_frame
    start.pose = self.current_pose_map

    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = self.map_frame
    goal.pose.position.x = gx
    goal.pose.position.y = gy
    goal.pose.position.z = 0.0
    goal.pose.orientation.w = 1.0

    try:
        resp = self.make_plan(start=start, goal=goal, tolerance=self.plan_tolerance)
        if not resp.plan.poses:
            return False
        length = 0.0
        prev = None
        for ps in resp.plan.poses:
            if prev is not None:
                length += np.hypot(ps.pose.position.x - prev.pose.position.x,
                                   ps.pose.position.y - prev.pose.position.y)
            prev = ps
        return length >= self.min_plan_length
    except Exception as e:
        rospy.logwarn_throttle(2.0, f"路径规划失败: {e}")
        return False
```

**执行流程**：
1. **连接路径规划服务**：如果未连接，等待服务可用并创建代理
2. **构建规划请求**：
   - 起始点：当前位置
   - 目标点：候选目标点
3. **调用规划服务**：调用 `/move_base/make_plan` 服务
4. **验证路径**：
   - 如果路径为空，返回 `False`
   - 计算路径总长度
   - 如果路径长度 >= 0.5m，返回 `True`（目标可达）

### 5.6 目标发布（`sample_and_publish_goal`）

```python
def sample_and_publish_goal(self):
    goal = self.generate_random_goal()
    if goal is None:
        rospy.logwarn("[目标生成器] 无法找到合适的随机目标")
        return

    goal_x, goal_y = goal
    yaw = np.arctan2(goal_y - self.current_pose_map.position.y,
                     goal_x - self.current_pose_map.position.x)
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)

    goal_msg = PoseStamped()
    goal_msg.header = Header(stamp=rospy.Time.now(), frame_id=self.map_frame)
    goal_msg.pose.position.x = goal_x
    goal_msg.pose.position.y = goal_y
    goal_msg.pose.position.z = 0.0
    goal_msg.pose.orientation.x = qx
    goal_msg.pose.orientation.y = qy
    goal_msg.pose.orientation.z = qz
    goal_msg.pose.orientation.w = qw

    # 获取起始位置信息
    start_x = self.current_pose_map.position.x
    start_y = self.current_pose_map.position.y
    distance_to_goal = np.hypot(goal_x - start_x, goal_y - start_y)

    self.goal_pub.publish(goal_msg)
    self.current_goal = (goal_x, goal_y)
    self.last_goal_time = rospy.Time.now()
    self._last_goal_publish_ts = self.last_goal_time

    obstacle_distance = self.get_min_obstacle_distance(goal_x, goal_y)
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("[目标生成器] 发布新目标点 - 开始新的Episode")
    rospy.loginfo(f"[目标生成器] 起始位置: ({start_x:.3f}, {start_y:.3f})")
    rospy.loginfo(f"[目标生成器] 目标位置: ({goal_x:.3f}, {goal_y:.3f})")
    rospy.loginfo(f"[目标生成器] 目标距离: {distance_to_goal:.3f}m")
    rospy.loginfo(f"[目标生成器] 障碍物距离: {obstacle_distance:.3f}m")
    rospy.loginfo(f"[目标生成器] 目标朝向: {np.degrees(yaw):.1f}°")
    rospy.loginfo("[目标生成器] 数据收集器将创建新的episode文件夹并开始记录数据")
    rospy.loginfo("=" * 60)
```

**执行流程**：
1. **生成随机目标**：调用 `generate_random_goal()`
2. **计算目标朝向**：计算从当前位置指向目标的角度，转换为四元数
3. **构建目标消息**：创建 `PoseStamped` 消息，包含位置和朝向
4. **发布目标**：发布到 `/move_base_simple/goal` 话题
5. **更新状态**：
   - 保存当前目标坐标
   - 记录发布时间
6. **输出日志**：输出目标详细信息

**调用时机**：
- 定时器回调（每15秒，如果需要新目标）
- 目标到达后（等待5秒后）
- 复位后

### 5.7 复位操作（`perform_reset`）

```python
def perform_reset(self):
    px = self.current_pose_map.position.x if self.current_pose_map else 0.0
    py = self.current_pose_map.position.y if self.current_pose_map else 0.0
    yaw = random.uniform(0, 2 * np.pi)

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = self.map_frame
    pose.pose.position.x = px
    pose.pose.position.y = py
    pose.pose.position.z = 0.0
    qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    did_reset = False

    if self.reset_client is not None:
        try:
            goal = ResetPosesGoal()
            if hasattr(goal, 'target'):
                goal.target = pose
            elif hasattr(goal, 'pose'):
                goal.pose = pose
            elif hasattr(goal, 'poses'):
                goal.poses = [pose]
            else:
                slots = getattr(goal, '__slots__', [])
                assigned = False
                for s in slots:
                    try:
                        setattr(goal, s, pose)
                        assigned = True
                        break
                    except Exception:
                        pass
                if not assigned:
                    rospy.logwarn("[reset] 无法识别 ResetPosesGoal 字段，跳过 action")

            if hasattr(goal, 'target') or hasattr(goal, 'pose') or hasattr(goal, 'poses') or 'assigned' in locals():
                self.reset_client.send_goal(goal)
                ok = self.reset_client.wait_for_result(rospy.Duration(5.0))
                if ok:
                    rospy.loginfo("[reset] ResetPoses.action 已返回")
                    did_reset = True
                else:
                    self.reset_client.cancel_goal()
                    rospy.logwarn("[reset] ResetPoses.action 超时")
        except Exception as e:
            rospy.logwarn(f"[reset] ResetPoses.action 失败：{e}")

    if not did_reset:
        init = PoseWithCovarianceStamped()
        init.header.stamp = rospy.Time.now()
        init.header.frame_id = self.map_frame
        init.pose.pose = pose.pose
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.2 ** 2
        cov[35] = (np.deg2rad(5.0)) ** 2
        init.pose.covariance = cov
        self.initial_pose_pub.publish(init)
        rospy.loginfo("[reset] 已发布 /initialpose 进行复位")

    # ===== 新增：复位后清除IMU侧翻状态 =====
    self.rollover_detected = False
    self.is_stuck = False
    self.stuck_start_time = None
    self._last_recovery_ts = rospy.Time.now()
    self.current_goal = None
    rospy.sleep(0.3)
    self.sample_and_publish_goal()
```

**执行流程**：
1. **确定复位位置**：
   - 位置：当前位置（不改变）
   - 朝向：随机角度 [0, 2π]
2. **构建复位消息**：创建 `PoseStamped` 消息
3. **尝试使用ResetPoses Action**（如果可用）：
   - 尝试设置目标位姿（兼容不同的Action定义）
   - 发送目标并等待结果（最多5秒）
4. **回退到/initialpose**（如果Action不可用或失败）：
   - 创建 `PoseWithCovarianceStamped` 消息
   - 设置协方差矩阵（位置不确定性0.2m，角度不确定性5°）
   - 发布到 `/initialpose` 话题
5. **重置状态**：
   - 清除侧翻和卡死标志
   - 清空当前目标
   - 记录恢复时间
6. **生成新目标**：等待0.3秒后，生成并发布新目标

**调用时机**：检测到卡死或侧翻时

---

## 六、主循环（`run` 方法）

```python
def run(self):
    rospy.spin()
```

**解释**：
- `rospy.spin()` 进入ROS主循环，保持节点运行
- 等待ROS消息和定时器触发回调函数

---

## 七、完整执行流程图

```
程序启动
    ↓
__init__() 初始化
    ├─ 创建ROS节点
    ├─ 加载参数
    ├─ 创建订阅者（地图、里程计、IMU）
    ├─ 创建发布者（目标、初始位姿）
    ├─ 初始化TF监听器
    ├─ 连接ResetPoses Action（如果可用）
    └─ 启动定时器（目标生成、卡死检测）
    ↓
run() → rospy.spin() 进入主循环
    ↓
┌─────────────────────────────────────┐
│  并行运行的异步回调函数              │
├─────────────────────────────────────┤
│                                     │
│  1. map_callback()                  │
│     └─ 接收地图数据                 │
│                                     │
│  2. odom_callback()                 │
│     ├─ 坐标变换（odom→map）         │
│     ├─ 卡死检测                     │
│     └─ check_goal_reached()        │
│        └─ 如果到达 → 生成新目标      │
│                                     │
│  3. imu_callback()                  │
│     └─ 侧翻检测                     │
│                                     │
│  4. generate_goal_timer_cb()        │
│     └─ 每15秒检查是否需要新目标    │
│        └─ 如果需要 → sample_and_    │
│           publish_goal()            │
│                                     │
│  5. stuck_check_cb()                │
│     └─ 每秒检查是否卡死/侧翻        │
│        └─ 如果卡死 → perform_reset()│
│                                     │
└─────────────────────────────────────┘
```

---

## 八、关键逻辑总结

### 8.1 目标生成策略
1. **随机采样**：从当前位置随机方向和距离生成候选点
2. **障碍物偏好**：优先选择距离障碍物0.5-1.0米的目标
3. **可达性验证**：使用路径规划服务验证目标是否可达
4. **空旷区域过滤**：80%概率丢弃过于空旷的目标

### 8.2 安全机制
1. **卡死检测**：位移 < 0.05m 且持续 > 45s
2. **侧翻检测**：IMU检测到 roll/pitch > 30°
3. **自动恢复**：检测到问题后自动复位机器人
4. **目标超时**：目标发布后30秒未到达，重新生成

### 8.3 与数据收集器的协作
1. **目标发布**：发布到 `/move_base_simple/goal`，触发数据收集器创建episode
2. **到达检测**：检测到到达后等待5秒，让数据收集器完成保存
3. **连续生成**：到达后自动生成下一个目标，形成连续的数据收集流程

