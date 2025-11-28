# ROS1中获取AGV小车位置坐标的方法（统一使用TF查询）

## 方法概述

本文档统一使用**TF（Transform）查询**方法获取AGV小车位置坐标。TF是ROS中标准的坐标系变换系统，可以查询任意坐标系中的机器人位姿。

**优势**：
- **统一接口**：所有坐标系使用相同的查询方法
- **灵活性强**：可以查询任意坐标系（odom、map等）
- **标准方法**：ROS推荐的坐标查询方式
- **全局定位**：使用map坐标系可获得全局一致的位置

---

## 核心方法：使用TF查询

### 特点
- **坐标系灵活**：可以查询任意坐标系中的位姿
- **全局定位**：如果查询 `map->base_link`，得到全局位置
- **不累积误差**：如果使用map坐标系
- **标准方法**：ROS推荐的坐标查询方式

### 步骤

#### 1. 检查TF树
```bash
# 查看TF树
rosrun tf view_frames
# 生成 frames.pdf 文件，显示TF树结构

# 查看特定变换
rosrun tf tf_echo map base_link
# 或
rosrun tf tf_echo odom base_link

# 查看所有可用坐标系
rosrun tf tf_monitor
```

#### 2. Python代码示例（基础版）

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion

class RobotPoseGetterTF:
    def __init__(self):
        rospy.init_node('robot_pose_getter_tf')
        
        # 初始化TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 坐标系名称（可通过参数配置）
        self.base_frame = rospy.get_param('~base_frame', 'base_link')  # 机器人本体坐标系
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')  # 里程计坐标系
        self.map_frame = rospy.get_param('~map_frame', 'map')  # 地图坐标系
        
        rospy.loginfo(f"TF查询器已初始化")
        rospy.loginfo(f"基础坐标系: {self.base_frame}")
        rospy.loginfo(f"目标坐标系: {self.odom_frame} 或 {self.map_frame}")
    
    def get_pose_in_frame(self, target_frame='odom', source_frame='base_link'):
        """
        获取机器人在指定坐标系中的位姿
        
        参数:
            target_frame: 目标坐标系（如 'odom', 'map'）
            source_frame: 源坐标系（通常是 'base_link'）
        
        返回:
            dict: {'x': float, 'y': float, 'theta': float, 'frame': str} 或 None
        """
        try:
            # 查询变换
            transform = self.tf_buffer.lookup_transform(
                target_frame,      # 目标坐标系
                source_frame,      # 源坐标系
                rospy.Time(0),     # 使用最新变换
                rospy.Duration(1.0)  # 超时时间
            )
            
            # 提取位置
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # 提取姿态（四元数转欧拉角）
            orientation = transform.transform.rotation
            quaternion = [
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w
            ]
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            
            return {
                'x': float(x),
                'y': float(y),
                'z': float(z),
                'theta': float(yaw),
                'roll': float(roll),
                'pitch': float(pitch),
                'frame': target_frame
            }
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, f"TF查询失败 ({source_frame} -> {target_frame}): {e}")
            return None
    
    def get_pose_odom(self):
        """获取机器人在odom坐标系中的位姿"""
        return self.get_pose_in_frame(self.odom_frame, self.base_frame)
    
    def get_pose_map(self):
        """获取机器人在map坐标系中的位姿"""
        return self.get_pose_in_frame(self.map_frame, self.base_frame)
    
    def wait_for_tf(self, target_frame='odom', source_frame='base_link', timeout=5.0):
        """等待TF变换可用"""
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if self.tf_buffer.can_transform(target_frame, source_frame, rospy.Time(0)):
                return True
            rospy.sleep(0.1)
        return False
    
    def run(self, rate_hz=1.0):
        """持续查询并打印位姿"""
        rate = rospy.Rate(rate_hz)
        
        # 等待TF可用
        rospy.loginfo("等待TF变换可用...")
        if not self.wait_for_tf(self.odom_frame, self.base_frame):
            rospy.logerr(f"无法获取TF变换: {self.base_frame} -> {self.odom_frame}")
            return
        
        rospy.loginfo("TF变换可用，开始查询位姿...")
        
        while not rospy.is_shutdown():
            # 获取odom坐标系中的位姿
            pose_odom = self.get_pose_odom()
            if pose_odom:
                rospy.loginfo(f"[Odom] x={pose_odom['x']:.3f}, y={pose_odom['y']:.3f}, theta={pose_odom['theta']:.3f}rad ({pose_odom['theta']*180/3.14159:.1f}°)")
            
            # 获取map坐标系中的位姿（如果可用）
            pose_map = self.get_pose_map()
            if pose_map:
                rospy.loginfo(f"[Map]  x={pose_map['x']:.3f}, y={pose_map['y']:.3f}, theta={pose_map['theta']:.3f}rad ({pose_map['theta']*180/3.14159:.1f}°)")
            elif pose_odom:
                rospy.logdebug("Map坐标系不可用，仅显示Odom坐标系")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        getter = RobotPoseGetterTF()
        rospy.sleep(1.0)  # 等待TF初始化
        getter.run(rate_hz=1.0)  # 1Hz查询频率
    except rospy.ROSInterruptException:
        pass
```

---

## 完整示例：统一TF查询接口

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
import math

class UnifiedRobotPoseGetter:
    """
    统一的机器人位姿获取器（使用TF查询）
    支持查询任意坐标系中的机器人位姿
    """
    def __init__(self):
        rospy.init_node('unified_robot_pose_getter')
        
        # 初始化TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 坐标系名称（可通过ROS参数配置）
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        
        # 存储最新位姿（可选，用于缓存）
        self.latest_pose_odom = None
        self.latest_pose_map = None
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("统一机器人位姿获取器已初始化（使用TF查询）")
        rospy.loginfo(f"基础坐标系: {self.base_frame}")
        rospy.loginfo(f"可查询坐标系: {self.odom_frame}, {self.map_frame}")
        rospy.loginfo("=" * 60)
    
    def get_pose(self, target_frame='odom', source_frame=None):
        """
        获取机器人在指定坐标系中的位姿（统一接口）
        
        参数:
            target_frame: 目标坐标系（'odom', 'map'等）
            source_frame: 源坐标系（默认使用base_frame）
        
        返回:
            dict: {
                'x': float, 'y': float, 'z': float,
                'theta': float, 'roll': float, 'pitch': float,
                'frame': str, 'timestamp': rospy.Time
            } 或 None
        """
        if source_frame is None:
            source_frame = self.base_frame
        
        try:
            # 查询变换
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            
            # 提取位置
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # 提取姿态
            orientation = transform.transform.rotation
            quaternion = [
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w
            ]
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            
            pose = {
                'x': float(x),
                'y': float(y),
                'z': float(z),
                'theta': float(yaw),
                'roll': float(roll),
                'pitch': float(pitch),
                'frame': target_frame,
                'timestamp': transform.header.stamp
            }
            
            # 更新缓存
            if target_frame == self.odom_frame:
                self.latest_pose_odom = pose
            elif target_frame == self.map_frame:
                self.latest_pose_map = pose
            
            return pose
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, 
                f"TF查询失败 ({source_frame} -> {target_frame}): {e}")
            return None
    
    def get_pose_odom(self):
        """获取odom坐标系中的位姿"""
        return self.get_pose(self.odom_frame)
    
    def get_pose_map(self):
        """获取map坐标系中的位姿"""
        return self.get_pose(self.map_frame)
    
    def calculate_distance(self, pose1, pose2):
        """
        计算两个位姿之间的距离
        
        参数:
            pose1, pose2: 位姿字典（包含'x'和'y'）
        
        返回:
            float: 欧几里得距离
        """
        if pose1 is None or pose2 is None:
            return None
        
        dx = pose1['x'] - pose2['x']
        dy = pose1['y'] - pose2['y']
        return math.hypot(dx, dy)
    
    def calculate_angle_to_goal(self, current_pose, goal_x, goal_y):
        """
        计算从当前位置到目标点的角度
        
        参数:
            current_pose: 当前位姿字典
            goal_x, goal_y: 目标点坐标
        
        返回:
            float: 角度（弧度）
        """
        if current_pose is None:
            return None
        
        dx = goal_x - current_pose['x']
        dy = goal_y - current_pose['y']
        return math.atan2(dy, dx)
    
    def print_pose(self, pose, label=""):
        """格式化打印位姿"""
        if pose is None:
            rospy.logwarn(f"{label}: 位姿不可用")
            return
        
        rospy.loginfo(f"{label}[{pose['frame']}]: "
                     f"x={pose['x']:.3f}, y={pose['y']:.3f}, "
                     f"theta={pose['theta']:.3f}rad ({pose['theta']*180/3.14159:.1f}°)")
    
    def run(self, rate_hz=1.0, print_odom=True, print_map=True):
        """持续查询并打印位姿"""
        rate = rospy.Rate(rate_hz)
        
        # 等待TF可用
        rospy.loginfo("等待TF变换可用...")
        rospy.sleep(1.0)
        
        if not self.tf_buffer.can_transform(self.odom_frame, self.base_frame, rospy.Time(0)):
            rospy.logerr(f"无法获取TF变换: {self.base_frame} -> {self.odom_frame}")
            rospy.logerr("请检查TF树是否正常发布")
            return
        
        rospy.loginfo("TF变换可用，开始查询位姿...")
        rospy.loginfo("=" * 60)
        
        while not rospy.is_shutdown():
            # 查询odom坐标系
            if print_odom:
                pose_odom = self.get_pose_odom()
                self.print_pose(pose_odom, "Odom")
            
            # 查询map坐标系
            if print_map:
                pose_map = self.get_pose_map()
                self.print_pose(pose_map, "Map")
            
            rospy.loginfo("-" * 60)
            rate.sleep()

if __name__ == '__main__':
    try:
        getter = UnifiedRobotPoseGetter()
        getter.run(rate_hz=1.0, print_odom=True, print_map=True)
    except rospy.ROSInterruptException:
        pass
```

---

## 使用示例

### 示例1：基本使用

```python
import rospy
from get_robot_pose_tf import RobotPoseGetterTF

rospy.init_node('example')
getter = RobotPoseGetterTF()
rospy.sleep(1.0)  # 等待TF初始化

# 获取odom坐标系中的位姿
pose_odom = getter.get_pose_odom()
if pose_odom:
    print(f"Odom位置: ({pose_odom['x']:.3f}, {pose_odom['y']:.3f})")
    print(f"航向角: {pose_odom['theta']:.3f}rad")

# 获取map坐标系中的位姿
pose_map = getter.get_pose_map()
if pose_map:
    print(f"Map位置: ({pose_map['x']:.3f}, {pose_map['y']:.3f})")
```

### 示例2：计算到目标点的距离

```python
import rospy
from get_robot_pose_tf import UnifiedRobotPoseGetter

rospy.init_node('example')
getter = UnifiedRobotPoseGetter()
rospy.sleep(1.0)

# 获取当前位置
current_pose = getter.get_pose_map()  # 使用map坐标系
goal_x, goal_y = 5.0, 3.0  # 目标点坐标

if current_pose:
    # 计算距离
    distance = getter.calculate_distance(
        current_pose,
        {'x': goal_x, 'y': goal_y}
    )
    print(f"到目标点距离: {distance:.3f}m")
    
    # 计算角度
    angle = getter.calculate_angle_to_goal(current_pose, goal_x, goal_y)
    print(f"到目标点角度: {angle:.3f}rad ({angle*180/3.14159:.1f}°)")
```

### 示例3：定时查询

```python
import rospy
from get_robot_pose_tf import UnifiedRobotPoseGetter

rospy.init_node('example')
getter = UnifiedRobotPoseGetter()
rospy.sleep(1.0)

rate = rospy.Rate(10.0)  # 10Hz
while not rospy.is_shutdown():
    pose = getter.get_pose_odom()
    if pose:
        # 处理位姿数据
        print(f"位置: ({pose['x']:.3f}, {pose['y']:.3f})")
    rate.sleep()
```

---

## 命令行工具

### 1. 查看TF变换
```bash
# 查看map->base_link变换
rosrun tf tf_echo map base_link

# 查看odom->base_link变换
rosrun tf tf_echo odom base_link

# 查看TF树
rosrun tf view_frames

# 监控TF发布频率
rosrun tf tf_monitor
```

### 2. 检查坐标系
```bash
# 查看所有可用坐标系
rosrun tf tf_monitor

# 检查特定变换是否存在
rosrun tf tf_echo map base_link
# 如果成功，会持续输出变换信息
# 如果失败，会显示错误信息
```

---

## 坐标系说明

### `base_link` 坐标系
- **定义**：机器人本体坐标系
- **位置**：通常位于机器人中心或底盘中心
- **用途**：作为查询的源坐标系

### `odom` 坐标系
- **定义**：里程计坐标系
- **特点**：相对于机器人启动位置，连续平滑
- **优点**：不会跳变，适合局部导航
- **缺点**：会累积误差

### `map` 坐标系
- **定义**：地图坐标系
- **特点**：全局坐标系，相对于地图
- **优点**：全局一致，不累积误差
- **缺点**：可能因定位更新而跳变

### 坐标系关系
```
map (全局坐标系)
  ↓ (map->odom变换，由定位系统维护)
odom (里程计坐标系)
  ↓ (odom->base_link变换，由里程计维护)
base_link (机器人本体坐标系)
```

---

## 常见问题

### 1. TF查询失败

**错误信息**：
```
TF查询失败: LookupException / ConnectivityException / ExtrapolationException
```

**解决方法**：
```bash
# 检查TF树
rosrun tf view_frames

# 检查特定变换
rosrun tf tf_echo map base_link

# 检查TF发布频率
rosrun tf tf_monitor

# 在代码中增加等待时间
rospy.sleep(2.0)  # 等待TF初始化
```

### 2. 坐标系名称不同

**问题**：不同机器人可能使用不同的坐标系名称

**解决方法**：
```python
# 通过ROS参数配置
rospy.set_param('~base_frame', 'your_base_frame')
rospy.set_param('~odom_frame', 'your_odom_frame')
rospy.set_param('~map_frame', 'your_map_frame')

# 或在launch文件中设置
<param name="base_frame" value="your_base_frame"/>
<param name="odom_frame" value="your_odom_frame"/>
<param name="map_frame" value="your_map_frame"/>
```

### 3. TF变换延迟

**问题**：查询的位姿可能不是最新的

**解决方法**：
```python
# 使用rospy.Time(0)获取最新变换
transform = self.tf_buffer.lookup_transform(
    target_frame,
    source_frame,
    rospy.Time(0),  # 使用最新变换
    rospy.Duration(1.0)
)
```

### 4. 坐标系不存在

**问题**：某些坐标系可能不存在（如map坐标系）

**解决方法**：
```python
# 检查坐标系是否存在
if self.tf_buffer.can_transform('map', 'base_link', rospy.Time(0)):
    pose_map = self.get_pose_map()
else:
    rospy.logwarn("Map坐标系不可用，使用Odom坐标系")
    pose_odom = self.get_pose_odom()
```

---

## 选择建议

### 使用 `odom` 坐标系如果：
- 需要连续、平滑的位置更新
- 进行局部导航（不需要全局定位）
- 机器人运动范围较小
- 不需要地图信息

### 使用 `map` 坐标系如果：
- 需要全局定位
- 需要在地图中导航
- 需要计算到地图中目标点的距离
- 使用SLAM或AMCL定位

---

## 总结

**统一方法**：使用TF查询获取机器人位姿

**优势**：
- ✅ 统一接口，代码简洁
- ✅ 支持任意坐标系
- ✅ ROS标准方法
- ✅ 全局定位支持

**关键点**：
- 确保TF树完整且正常发布
- 注意坐标系名称（可通过参数配置）
- 处理异常情况（TF查询失败等）
- 根据需求选择合适的坐标系（odom或map）

**推荐使用**：
- **日常使用**：查询 `odom` 坐标系（连续、平滑）
- **全局导航**：查询 `map` 坐标系（全局一致）
