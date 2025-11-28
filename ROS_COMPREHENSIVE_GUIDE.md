# ROS（Robot Operating System）全面学习指南

## 目录

1. [ROS基础概念](#ros基础概念)
2. [ROS核心组件](#ros核心组件)
3. [常用命令速查](#常用命令速查)
4. [ROS编程基础](#ros编程基础)
5. [实际应用示例](#实际应用示例)
6. [高级主题](#高级主题)
7. [调试与故障排除](#调试与故障排除)
8. [最佳实践](#最佳实践)

---

## ROS基础概念

### 什么是ROS？

**ROS (Robot Operating System)** 是一个开源的机器人操作系统框架，提供：
- **通信机制**：节点间消息传递
- **工具集**：可视化、调试、仿真工具
- **软件库**：常用算法和功能包
- **生态系统**：丰富的第三方包

### ROS版本

- **ROS 1 (Noetic)**：当前稳定版本，使用Python 2/3
- **ROS 2 (Humble/Foxy)**：下一代ROS，使用DDS通信

本文档主要针对 **ROS 1 (Noetic)**。

### ROS核心概念

#### 1. 节点 (Node)
- **定义**：执行特定任务的进程
- **作用**：机器人系统的功能单元
- **特点**：可以独立运行、相互通信

#### 2. 话题 (Topic)
- **定义**：节点间异步通信的通道
- **特点**：发布/订阅模式，一对多通信
- **用途**：传感器数据、控制命令等

#### 3. 服务 (Service)
- **定义**：节点间同步通信机制
- **特点**：请求/响应模式，一对一通信
- **用途**：查询状态、执行特定操作

#### 4. 参数服务器 (Parameter Server)
- **定义**：存储配置参数的共享字典
- **特点**：全局可访问，键值对存储
- **用途**：配置参数、系统设置

#### 5. 消息 (Message)
- **定义**：节点间传递的数据结构
- **格式**：`.msg` 文件定义
- **类型**：标准消息、自定义消息

#### 6. 工作空间 (Workspace)
- **定义**：ROS项目的目录结构
- **结构**：
  ```
  workspace/
    src/          # 源代码
    build/        # 编译文件
    devel/        # 开发环境
    install/      # 安装文件
  ```

---

## ROS核心组件

### 1. 节点 (Node)

#### 节点的作用
- 执行特定功能（如传感器驱动、控制算法）
- 通过话题/服务与其他节点通信
- 可以独立启动和停止

#### 节点命名规则
- 使用小写字母、数字、下划线
- 不能以数字开头
- 示例：`robot_controller`, `sensor_node`

### 2. 话题 (Topic)

#### 话题通信模式
```
发布者 (Publisher)  --[Topic]-->  订阅者 (Subscriber)
```

#### 话题特点
- **异步**：发布者不等待订阅者
- **多对多**：多个发布者/订阅者
- **类型安全**：消息类型必须匹配

### 3. 服务 (Service)

#### 服务通信模式
```
客户端 (Client)  --[Request]-->  服务器 (Server)
客户端 (Client)  <--[Response]--  服务器 (Server)
```

#### 服务特点
- **同步**：客户端等待响应
- **一对一**：一个请求对应一个响应
- **阻塞**：调用时阻塞直到响应

### 4. 动作 (Action)

#### 动作通信模式
```
客户端 (Client)  --[Goal]-->  服务器 (Server)
客户端 (Client)  <--[Feedback]--  服务器 (Server)
客户端 (Client)  <--[Result]--  服务器 (Server)
```

#### 动作特点
- **异步**：支持长时间运行的任务
- **反馈**：可以发送进度更新
- **可取消**：可以取消正在执行的任务

### 5. 参数服务器 (Parameter Server)

#### 参数类型
- 字符串 (string)
- 整数 (int)
- 浮点数 (double)
- 布尔值 (bool)
- 列表 (list)
- 字典 (dict)

### 6. TF (Transform)

#### TF的作用
- 管理坐标系之间的变换关系
- 提供坐标转换功能
- 维护坐标系树结构

#### 坐标系关系
```
map -> odom -> base_link -> sensor_link
```

---

## 常用命令速查

### 工作空间命令

```bash
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

# 编译工作空间
cd ~/catkin_ws
catkin_make

# 设置环境变量
source ~/catkin_ws/devel/setup.bash

# 添加到.bashrc（永久生效）
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### 节点命令

```bash
# 列出所有运行中的节点
rosnode list

# 查看节点信息
rosnode info /node_name

# 查看节点机器
rosnode machine /node_name

# 杀死节点
rosnode kill /node_name

# 清理无效节点
rosnode cleanup

# 运行节点
rosrun package_name node_name
```

### 话题命令

```bash
# 列出所有话题
rostopic list

# 查看话题信息
rostopic info /topic_name

# 查看话题类型
rostopic type /topic_name

# 查看话题消息格式
rostopic type /topic_name | rosmsg show

# 查看话题数据（实时）
rostopic echo /topic_name

# 查看话题数据（一次）
rostopic echo /topic_name -n 1

# 查看话题发布频率
rostopic hz /topic_name

# 查看话题带宽
rostopic bw /topic_name

# 发布话题数据
rostopic pub /topic_name message_type "data"

# 发布话题数据（持续）
rostopic pub -r 10 /topic_name message_type "data"
```

### 服务命令

```bash
# 列出所有服务
rosservice list

# 查看服务信息
rosservice info /service_name

# 查看服务类型
rosservice type /service_name

# 查看服务参数格式
rosservice type /service_name | rossrv show

# 调用服务
rosservice call /service_name "request_data"

# 查看服务URI
rosservice uri /service_name
```

### 参数命令

```bash
# 列出所有参数
rosparam list

# 获取参数值
rosparam get /param_name

# 设置参数值
rosparam set /param_name value

# 删除参数
rosparam delete /param_name

# 从文件加载参数
rosparam load file.yaml

# 保存参数到文件
rosparam dump file.yaml

# 获取参数命名空间
rosparam get /namespace/
```

### 消息命令

```bash
# 列出所有消息类型
rosmsg list

# 查看消息定义
rosmsg show message_type

# 查看消息包
rosmsg package message_type

# 查找消息
rosmsg find message_type
```

### 服务消息命令

```bash
# 列出所有服务类型
rossrv list

# 查看服务定义
rossrv show service_type

# 查看服务包
rossrv package service_type
```

### TF命令

```bash
# 查看TF树
rosrun tf view_frames
# 生成 frames.pdf

# 查看特定变换
rosrun tf tf_echo frame1 frame2

# 监控TF
rosrun tf tf_monitor

# 查看TF发布频率
rosrun tf tf_monitor frame1 frame2
```

### 包管理命令

```bash
# 查找包
rospack find package_name

# 列出包
rospack list

# 列出包依赖
rospack depends package_name

# 列出包依赖树
rospack depends1 package_name
```

### 启动文件命令

```bash
# 运行launch文件
roslaunch package_name launch_file.launch

# 检查launch文件语法
roslaunch --check package_name launch_file.launch

# 调试launch文件
roslaunch --screen package_name launch_file.launch
```

### 数据记录命令

```bash
# 录制所有话题
rosbag record -a

# 录制指定话题
rosbag record /topic1 /topic2

# 录制到指定文件
rosbag record -O output.bag /topic1

# 播放bag文件
rosbag play file.bag

# 播放bag文件（循环）
rosbag play -l file.bag

# 查看bag文件信息
rosbag info file.bag

# 压缩bag文件
rosbag compress file.bag
```

---

## ROS编程基础

### Python编程

#### 1. 创建发布者节点

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

def talker():
    # 初始化节点
    rospy.init_node('talker', anonymous=True)
    
    # 创建发布者
    pub = rospy.Publisher('chatter', String, queue_size=10)
    
    # 设置发布频率
    rate = rospy.Rate(10)  # 10Hz
    
    while not rospy.is_shutdown():
        # 创建消息
        msg = String()
        msg.data = "Hello ROS!"
        
        # 发布消息
        pub.publish(msg)
        rospy.loginfo(f"Published: {msg.data}")
        
        # 等待
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

#### 2. 创建订阅者节点

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

def callback(msg):
    """话题回调函数"""
    rospy.loginfo(f"Received: {msg.data}")

def listener():
    # 初始化节点
    rospy.init_node('listener', anonymous=True)
    
    # 创建订阅者
    rospy.Subscriber('chatter', String, callback)
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    listener()
```

#### 3. 创建服务服务器

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import AddTwoInts, AddTwoIntsResponse

def handle_add_two_ints(req):
    """服务处理函数"""
    rospy.loginfo(f"Request: {req.a} + {req.b}")
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    # 初始化节点
    rospy.init_node('add_two_ints_server')
    
    # 创建服务服务器
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    
    rospy.loginfo("Ready to add two ints.")
    rospy.spin()

if __name__ == '__main__':
    add_two_ints_server()
```

#### 4. 创建服务客户端

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import AddTwoInts

def add_two_ints_client(x, y):
    # 等待服务可用
    rospy.wait_for_service('add_two_ints')
    
    try:
        # 创建服务客户端
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        
        # 调用服务
        resp = add_two_ints(x, y)
        return resp.sum
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('add_two_ints_client')
    
    x = 1
    y = 2
    rospy.loginfo(f"Requesting {x} + {y}")
    result = add_two_ints_client(x, y)
    rospy.loginfo(f"Result: {result}")
```

#### 5. 使用参数服务器

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

def param_example():
    rospy.init_node('param_example')
    
    # 获取参数（带默认值）
    param1 = rospy.get_param('~param1', 'default_value')
    param2 = rospy.get_param('~param2', 10)
    param3 = rospy.get_param('~param3', 3.14)
    
    # 设置参数
    rospy.set_param('~my_param', 'my_value')
    
    # 检查参数是否存在
    if rospy.has_param('~param1'):
        value = rospy.get_param('~param1')
    
    rospy.loginfo(f"param1: {param1}, param2: {param2}, param3: {param3}")

if __name__ == '__main__':
    param_example()
```

#### 6. 使用TF查询坐标

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion

def tf_example():
    rospy.init_node('tf_example')
    
    # 初始化TF
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            # 查询变换
            transform = tf_buffer.lookup_transform(
                'map',      # 目标坐标系
                'base_link', # 源坐标系
                rospy.Time(0)
            )
            
            # 提取位置
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # 提取姿态
            orientation = transform.transform.rotation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            
            rospy.loginfo(f"Position: ({x:.3f}, {y:.3f}, {z:.3f})")
            rospy.loginfo(f"Orientation: ({roll:.3f}, {pitch:.3f}, {yaw:.3f})")
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF lookup failed: {e}")
        
        rate.sleep()

if __name__ == '__main__':
    tf_example()
```

### C++编程

#### 1. 创建发布者节点

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    
    // 创建发布者
    ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 1000);
    
    // 设置发布频率
    ros::Rate loop_rate(10);
    
    int count = 0;
    while (ros::ok())
    {
        // 创建消息
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Hello ROS! " << count;
        msg.data = ss.str();
        
        // 发布消息
        pub.publish(msg);
        ROS_INFO("%s", msg.data.c_str());
        
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    
    return 0;
}
```

#### 2. 创建订阅者节点

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    
    // 创建订阅者
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    
    // 保持节点运行
    ros::spin();
    
    return 0;
}
```

---

## 实际应用示例

### 示例1：机器人位置监控

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class RobotMonitor:
    def __init__(self):
        rospy.init_node('robot_monitor')
        
        # 订阅里程计
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # 存储位置
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # 目标点
        self.goal_x = 5.0
        self.goal_y = 3.0
        
    def odom_callback(self, msg):
        # 提取位置
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # 提取姿态
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.theta = yaw
        
        # 计算到目标点的距离
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = math.hypot(dx, dy)
        
        # 计算到目标点的角度
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = angle_to_goal - self.theta
        
        rospy.loginfo(f"Position: ({self.x:.3f}, {self.y:.3f})")
        rospy.loginfo(f"Distance to goal: {distance:.3f}m")
        rospy.loginfo(f"Angle to goal: {angle_diff:.3f}rad")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    monitor = RobotMonitor()
    monitor.run()
```

### 示例2：速度控制节点

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class VelocityController:
    def __init__(self):
        rospy.init_node('velocity_controller')
        
        # 发布速度命令
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 订阅位置
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # 控制参数
        self.kp_linear = 0.5
        self.kp_angular = 1.0
        self.max_linear_vel = 0.5
        self.max_angular_vel = 1.0
        
        # 目标点
        self.goal_x = 5.0
        self.goal_y = 3.0
        
        # 当前位置
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
    def odom_callback(self, msg):
        # 更新位置
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.theta = yaw
        
        # 计算控制命令
        self.compute_control()
    
    def compute_control(self):
        # 计算到目标点的距离和角度
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.theta
        
        # 归一化角度误差到[-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # PID控制
        linear_vel = self.kp_linear * distance
        angular_vel = self.kp_angular * angle_error
        
        # 限制速度
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        # 创建速度命令
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        
        # 发布命令
        self.cmd_vel_pub.publish(cmd)
        
        rospy.loginfo(f"Distance: {distance:.3f}m, Angle error: {angle_error:.3f}rad")
        rospy.loginfo(f"Linear vel: {linear_vel:.3f}, Angular vel: {angular_vel:.3f}")
    
    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    controller = VelocityController()
    controller.run()
```

### 示例3：Launch文件示例

```xml
<?xml version="1.0"?>
<launch>
    <!-- 参数定义 -->
    <param name="robot_name" value="my_robot" />
    <param name="max_velocity" value="1.0" />
    
    <!-- 从文件加载参数 -->
    <rosparam file="$(find my_package)/config/params.yaml" command="load" />
    
    <!-- 节点启动 -->
    <node name="robot_controller" pkg="my_package" type="controller_node" output="screen">
        <param name="kp" value="0.5" />
        <remap from="/cmd_vel" to="/robot/cmd_vel" />
    </node>
    
    <node name="robot_monitor" pkg="my_package" type="monitor_node" output="screen">
        <param name="update_rate" value="10.0" />
    </node>
    
    <!-- 包含其他launch文件 -->
    <include file="$(find another_package)/launch/another.launch" />
    
    <!-- 条件启动 -->
    <node name="optional_node" pkg="my_package" type="optional_node" if="$(arg enable_optional)" />
    
    <!-- 组命名空间 -->
    <group ns="robot1">
        <node name="controller" pkg="my_package" type="controller_node" />
    </group>
</launch>
```

---

## 高级主题

### 1. 自定义消息

#### 创建消息文件

```bash
# 在package/msg/目录下创建
# MyMessage.msg
string name
int32 id
float64 value
geometry_msgs/Point position
```

#### 使用自定义消息

```python
from my_package.msg import MyMessage

msg = MyMessage()
msg.name = "test"
msg.id = 1
msg.value = 3.14
msg.position.x = 1.0
msg.position.y = 2.0
msg.position.z = 3.0
```

### 2. 自定义服务

#### 创建服务文件

```bash
# 在package/srv/目录下创建
# MyService.srv
# 请求部分
int32 a
int32 b
---
# 响应部分
int32 sum
```

### 3. 动态参数配置

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from dynamic_reconfigure.server import Server
from my_package.cfg import MyConfig

def callback(config, level):
    rospy.loginfo("Reconfigure Request: {config}")
    return config

if __name__ == '__main__':
    rospy.init_node('dynamic_reconfigure_node')
    srv = Server(MyConfig, callback)
    rospy.spin()
```

### 4. 时间同步

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import message_filters
from sensor_msgs.msg import Image, LaserScan

def callback(image, scan):
    # 处理同步的数据
    rospy.loginfo("Received synchronized data")

if __name__ == '__main__':
    rospy.init_node('sync_node')
    
    # 创建订阅者
    image_sub = message_filters.Subscriber('/camera/image', Image)
    scan_sub = message_filters.Subscriber('/scan', LaserScan)
    
    # 创建时间同步器
    ts = message_filters.ApproximateTimeSynchronizer(
        [image_sub, scan_sub], 10, 0.1
    )
    ts.registerCallback(callback)
    
    rospy.spin()
```

---

## 调试与故障排除

### 常用调试工具

#### 1. RViz（可视化工具）
```bash
# 启动RViz
rosrun rviz rviz

# 常用功能：
# - 显示TF树
# - 显示点云
# - 显示地图
# - 显示路径
```

#### 2. rqt（图形化工具集）
```bash
# 启动rqt
rqt

# 常用插件：
# - rqt_graph: 查看节点和话题图
# - rqt_topic: 监控话题
# - rqt_service: 调用服务
# - rqt_reconfigure: 动态参数配置
```

#### 3. 日志级别
```python
# 设置日志级别
rospy.logdebug("Debug message")
rospy.loginfo("Info message")
rospy.logwarn("Warning message")
rospy.logerr("Error message")
rospy.logfatal("Fatal message")
```

### 常见问题

#### 1. 节点无法启动
```bash
# 检查节点是否存在
rosnode list

# 检查包是否正确编译
catkin_make

# 检查环境变量
echo $ROS_PACKAGE_PATH
```

#### 2. 话题无数据
```bash
# 检查话题是否存在
rostopic list

# 检查话题发布频率
rostopic hz /topic_name

# 检查话题类型
rostopic type /topic_name
```

#### 3. TF查询失败
```bash
# 检查TF树
rosrun tf view_frames

# 检查特定变换
rosrun tf tf_echo frame1 frame2

# 检查TF发布频率
rosrun tf tf_monitor
```

---

## 最佳实践

### 1. 代码组织
- 使用命名空间避免冲突
- 合理使用参数服务器
- 遵循ROS命名规范

### 2. 性能优化
- 合理设置队列大小
- 使用消息过滤器减少处理
- 避免在回调中执行耗时操作

### 3. 错误处理
- 始终检查返回值
- 使用异常处理
- 添加日志记录

### 4. 文档编写
- 编写README文件
- 添加代码注释
- 提供使用示例

---

## 总结

### 核心要点
1. **节点**：功能单元
2. **话题**：异步通信
3. **服务**：同步通信
4. **参数**：配置管理
5. **TF**：坐标变换

### 学习路径
1. 掌握基本概念
2. 熟悉常用命令
3. 编写简单节点
4. 理解通信机制
5. 实践项目应用

### 推荐资源
- ROS官方Wiki: http://wiki.ros.org/
- ROS教程: http://wiki.ros.org/ROS/Tutorials
- ROS问答: https://answers.ros.org/
- GitHub: 搜索ROS相关项目

---

## 附录：常用消息类型

### 标准消息
- `std_msgs/String`: 字符串
- `std_msgs/Int32`: 整数
- `std_msgs/Float64`: 浮点数
- `std_msgs/Bool`: 布尔值
- `std_msgs/Header`: 消息头（时间戳、坐标系）

### 几何消息
- `geometry_msgs/Point`: 点（x, y, z）
- `geometry_msgs/Pose`: 位姿（位置+姿态）
- `geometry_msgs/PoseStamped`: 带时间戳和坐标系的位姿
- `geometry_msgs/Twist`: 速度（线速度+角速度）
- `geometry_msgs/Quaternion`: 四元数（x, y, z, w）
- `geometry_msgs/Vector3`: 三维向量

### 导航消息
- `nav_msgs/Odometry`: 里程计数据
- `nav_msgs/Path`: 路径（位姿序列）
- `nav_msgs/OccupancyGrid`: 占用栅格地图
- `nav_msgs/MapMetaData`: 地图元数据
- `nav_msgs/GetPlan`: 路径规划服务

### 传感器消息
- `sensor_msgs/Image`: 图像数据
- `sensor_msgs/CompressedImage`: 压缩图像
- `sensor_msgs/LaserScan`: 激光扫描数据
- `sensor_msgs/PointCloud2`: 点云数据
- `sensor_msgs/Imu`: IMU数据（加速度、角速度、姿态）
- `sensor_msgs/CameraInfo`: 相机标定信息
- `sensor_msgs/JointState`: 关节状态

### 动作消息
- `actionlib_msgs/GoalStatusArray`: 动作状态数组
- `actionlib_msgs/GoalStatus`: 单个动作状态
- `actionlib_msgs/GoalID`: 动作目标ID

### 控制消息
- `move_base_msgs/MoveBaseAction`: move_base动作
- `move_base_msgs/MoveBaseGoal`: move_base目标
- `move_base_msgs/MoveBaseResult`: move_base结果

---

## 常见ROS节点和话题

### 导航相关

#### move_base 节点
- **功能**：全局路径规划和局部路径跟踪
- **输入话题**：
  - `/move_base_simple/goal` (geometry_msgs/PoseStamped): 简单目标
  - `/move_base/goal` (move_base_msgs/MoveBaseActionGoal): 动作目标
  - `/odom` (nav_msgs/Odometry): 里程计
  - `/map` (nav_msgs/OccupancyGrid): 地图
  - `/scan` (sensor_msgs/LaserScan): 激光扫描
- **输出话题**：
  - `/cmd_vel` (geometry_msgs/Twist): 速度命令
  - `/move_base/status` (actionlib_msgs/GoalStatusArray): 状态
  - `/move_base/feedback` (move_base_msgs/MoveBaseActionFeedback): 反馈
- **服务**：
  - `/move_base/make_plan` (nav_msgs/GetPlan): 路径规划服务
  - `/move_base/clear_costmaps` (std_srvs/Empty): 清除代价地图

#### amcl 节点
- **功能**：自适应蒙特卡洛定位
- **输入话题**：
  - `/scan` (sensor_msgs/LaserScan): 激光扫描
  - `/map` (nav_msgs/OccupancyGrid): 地图
  - `/initialpose` (geometry_msgs/PoseWithCovarianceStamped): 初始位姿
- **输出话题**：
  - `/amcl_pose` (geometry_msgs/PoseWithCovarianceStamped): 定位结果
  - `/particlecloud` (geometry_msgs/PoseArray): 粒子云

#### map_server 节点
- **功能**：地图服务器
- **输出话题**：
  - `/map` (nav_msgs/OccupancyGrid): 地图数据
  - `/map_metadata` (nav_msgs/MapMetaData): 地图元数据

### 传感器相关

#### 激光雷达节点（通常由驱动包提供）
- **输出话题**：
  - `/scan` (sensor_msgs/LaserScan): 激光扫描数据
- **消息字段**：
  - `angle_min`, `angle_max`: 角度范围
  - `angle_increment`: 角度增量
  - `range_min`, `range_max`: 距离范围
  - `ranges[]`: 距离数组
  - `intensities[]`: 强度数组

#### 相机节点（通常由驱动包提供）
- **输出话题**：
  - `/camera/image_raw` (sensor_msgs/Image): 原始图像
  - `/camera/image_compressed` (sensor_msgs/CompressedImage): 压缩图像
  - `/camera/camera_info` (sensor_msgs/CameraInfo): 相机信息
  - `/camera/depth/image_raw` (sensor_msgs/Image): 深度图像

#### IMU节点（通常由驱动包提供）
- **输出话题**：
  - `/imu/data` (sensor_msgs/Imu): IMU数据
- **消息字段**：
  - `linear_acceleration`: 线性加速度
  - `angular_velocity`: 角速度
  - `orientation`: 姿态（四元数）

### 里程计相关

#### 里程计节点（通常由驱动包提供）
- **输出话题**：
  - `/odom` (nav_msgs/Odometry): 里程计数据
- **消息字段**：
  - `pose.pose`: 位姿（位置+姿态）
  - `pose.covariance`: 位姿协方差
  - `twist.twist`: 速度（线速度+角速度）
  - `twist.covariance`: 速度协方差

### TF相关

#### robot_state_publisher 节点
- **功能**：发布机器人TF变换
- **输入话题**：
  - `/joint_states` (sensor_msgs/JointState): 关节状态
- **输出**：TF变换（通过TF系统）

#### static_transform_publisher 节点
- **功能**：发布静态TF变换
- **用法**：
  ```bash
  rosrun tf static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_ms
  ```

### 控制相关

#### cmd_vel 话题
- **类型**：`geometry_msgs/Twist`
- **用途**：速度控制命令
- **字段**：
  - `linear.x`: 线速度（前进/后退，m/s）
  - `linear.y`: 侧向速度（通常为0，m/s）
  - `linear.z`: 垂直速度（通常为0，m/s）
  - `angular.x`: 滚转角速度（通常为0，rad/s）
  - `angular.y`: 俯仰角速度（通常为0，rad/s）
  - `angular.z`: 偏航角速度（转向，rad/s）

### 常用服务

#### 导航服务
- `/move_base/make_plan` (nav_msgs/GetPlan): 生成路径规划
- `/move_base/clear_costmaps` (std_srvs/Empty): 清除代价地图
- `/global_localization` (std_srvs/Empty): 全局定位（AMCL）

#### 系统服务
- `/rosout/get_loggers` (roscpp/GetLoggers): 获取日志器
- `/rosout/set_logger_level` (roscpp/SetLoggerLevel): 设置日志级别

---

## 常见消息详细说明

### geometry_msgs/Twist（速度命令）

```python
from geometry_msgs.msg import Twist

cmd = Twist()
cmd.linear.x = 0.5   # 前进速度 0.5 m/s
cmd.linear.y = 0.0   # 侧向速度（差速机器人通常为0）
cmd.linear.z = 0.0   # 垂直速度（地面机器人通常为0）
cmd.angular.x = 0.0  # 滚转角速度
cmd.angular.y = 0.0  # 俯仰角速度
cmd.angular.z = 0.3  # 转向角速度 0.3 rad/s
```

**常用场景**：
- 差速驱动机器人：只使用 `linear.x` 和 `angular.z`
- 全向驱动机器人：使用 `linear.x`, `linear.y`, `angular.z`

### nav_msgs/Odometry（里程计）

```python
from nav_msgs.msg import Odometry

# 订阅
def odom_callback(msg):
    # 位置
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    
    # 姿态（四元数）
    orientation = msg.pose.pose.orientation
    
    # 速度
    linear_x = msg.twist.twist.linear.x
    angular_z = msg.twist.twist.angular.z
```

**常用字段**：
- `header.frame_id`: 坐标系（通常是 `odom`）
- `child_frame_id`: 子坐标系（通常是 `base_link`）
- `pose.pose`: 位姿
- `twist.twist`: 速度

### sensor_msgs/LaserScan（激光扫描）

```python
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    # 角度范围
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    angle_increment = msg.angle_increment
    
    # 距离数据
    ranges = msg.ranges  # 距离数组（米）
    intensities = msg.intensities  # 强度数组
    
    # 计算每个点的角度
    angles = []
    for i in range(len(ranges)):
        angle = angle_min + i * angle_increment
        angles.append(angle)
```

**常用字段**：
- `angle_min`, `angle_max`: 扫描角度范围（弧度）
- `angle_increment`: 角度增量（弧度）
- `time_increment`: 时间增量（秒）
- `scan_time`: 扫描时间（秒）
- `range_min`, `range_max`: 有效距离范围（米）
- `ranges[]`: 距离数组（米，inf表示无效）
- `intensities[]`: 强度数组

### sensor_msgs/Image（图像）

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

def image_callback(msg):
    # 转换为OpenCV格式
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    
    # 图像信息
    height = msg.height
    width = msg.width
    encoding = msg.encoding  # 如 "bgr8", "rgb8", "mono8"
    
    # 处理图像
    cv2.imshow("Image", cv_image)
    cv2.waitKey(1)
```

**常用编码**：
- `bgr8`: BGR彩色图像（OpenCV默认）
- `rgb8`: RGB彩色图像
- `mono8`: 灰度图像
- `32FC1`: 32位浮点单通道（深度图）

### nav_msgs/OccupancyGrid（地图）

```python
from nav_msgs.msg import OccupancyGrid
import numpy as np

def map_callback(msg):
    # 地图信息
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution  # 米/像素
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y
    
    # 地图数据（转换为numpy数组）
    map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
    
    # 值含义：
    # -1: 未知区域
    # 0: 自由空间
    # 100: 障碍物
    # 1-99: 代价（值越大，代价越高）
```

**常用操作**：
- 世界坐标转地图坐标：
  ```python
  mx = int((world_x - origin_x) / resolution)
  my = int((world_y - origin_y) / resolution)
  ```
- 地图坐标转世界坐标：
  ```python
  world_x = origin_x + mx * resolution
  world_y = origin_y + my * resolution
  ```

### geometry_msgs/PoseStamped（带时间戳的位姿）

```python
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# 创建位姿
pose = PoseStamped()
pose.header.stamp = rospy.Time.now()
pose.header.frame_id = "map"
pose.pose.position.x = 1.0
pose.pose.position.y = 2.0
pose.pose.position.z = 0.0

# 设置姿态（欧拉角转四元数）
qx, qy, qz, qw = quaternion_from_euler(0, 0, 1.57)  # 90度
pose.pose.orientation.x = qx
pose.pose.orientation.y = qy
pose.pose.orientation.z = qz
pose.pose.orientation.w = qw

# 提取姿态（四元数转欧拉角）
orientation = pose.pose.orientation
quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
roll, pitch, yaw = euler_from_quaternion(quaternion)
```

---

## 常见节点启动命令

### 导航节点
```bash
# move_base
rosrun move_base move_base

# amcl
rosrun amcl amcl

# map_server
rosrun map_server map_server map.yaml
```

### 传感器节点
```bash
# 激光雷达（示例）
rosrun hokuyo_node hokuyo_node

# 相机（示例）
rosrun usb_cam usb_cam_node

# IMU（示例）
rosrun razor_imu_9dof razor_imu_9dof_node
```

### TF节点
```bash
# robot_state_publisher
rosrun robot_state_publisher robot_state_publisher

# static_transform_publisher
rosrun tf static_transform_publisher 0 0 0 0 0 0 map odom 100
```

---

## 话题通信模式总结

### 发布/订阅模式（异步）
```
发布者 → [话题] → 订阅者1
              → 订阅者2
              → 订阅者3
```

### 服务模式（同步）
```
客户端 → [请求] → 服务器
客户端 ← [响应] ← 服务器
```

### 动作模式（异步+反馈）
```
客户端 → [Goal] → 服务器
客户端 ← [Feedback] ← 服务器
客户端 ← [Result] ← 服务器
```

---

## 完整话题列表参考

### 导航系统常见话题

| 话题名称 | 消息类型 | 方向 | 说明 |
|---------|---------|------|------|
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 输入 | 简单目标发布 |
| `/move_base/goal` | `move_base_msgs/MoveBaseActionGoal` | 输入 | 动作目标 |
| `/move_base/status` | `actionlib_msgs/GoalStatusArray` | 输出 | 动作状态 |
| `/move_base/feedback` | `move_base_msgs/MoveBaseActionFeedback` | 输出 | 动作反馈 |
| `/move_base/result` | `move_base_msgs/MoveBaseActionResult` | 输出 | 动作结果 |
| `/cmd_vel` | `geometry_msgs/Twist` | 输出 | 速度命令 |
| `/odom` | `nav_msgs/Odometry` | 输入 | 里程计数据 |
| `/map` | `nav_msgs/OccupancyGrid` | 输入 | 地图数据 |
| `/scan` | `sensor_msgs/LaserScan` | 输入 | 激光扫描 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 输入 | 初始位姿 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 输出 | AMCL定位结果 |

### 传感器常见话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/camera/image_raw` | `sensor_msgs/Image` | 原始图像 |
| `/camera/image_compressed` | `sensor_msgs/CompressedImage` | 压缩图像 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图像 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 相机标定信息 |
| `/scan` | `sensor_msgs/LaserScan` | 激光扫描 |
| `/imu/data` | `sensor_msgs/Imu` | IMU数据 |
| `/joint_states` | `sensor_msgs/JointState` | 关节状态 |

### TF常见坐标系

| 坐标系名称 | 说明 | 父坐标系 |
|-----------|------|---------|
| `map` | 地图坐标系（全局） | - |
| `odom` | 里程计坐标系 | `map` |
| `base_link` | 机器人本体坐标系 | `odom` |
| `base_footprint` | 机器人投影到地面的点 | `base_link` |
| `laser_link` | 激光雷达坐标系 | `base_link` |
| `camera_link` | 相机坐标系 | `base_link` |
| `imu_link` | IMU坐标系 | `base_link` |

---

## 消息字段详细说明

### geometry_msgs/Twist（速度命令）

**完整定义**：
```python
geometry_msgs/Vector3 linear    # 线速度 (m/s)
  float64 x                     # 前进/后退
  float64 y                     # 左/右
  float64 z                     # 上/下
geometry_msgs/Vector3 angular   # 角速度 (rad/s)
  float64 x                     # 滚转
  float64 y                     # 俯仰
  float64 z                     # 偏航
```

**差速机器人示例**：
```python
cmd = Twist()
cmd.linear.x = 0.5   # 前进 0.5 m/s
cmd.angular.z = 0.3  # 转向 0.3 rad/s
```

### nav_msgs/Odometry（里程计）

**完整定义**：
```python
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id              # 通常是 "odom"
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x, y, z
    geometry_msgs/Quaternion orientation
      float64 x, y, z, w
  float64[36] covariance       # 6x6协方差矩阵
string child_frame_id          # 通常是 "base_link"
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
    geometry_msgs/Vector3 angular
  float64[36] covariance
```

### sensor_msgs/LaserScan（激光扫描）

**完整定义**：
```python
std_msgs/Header header
float32 angle_min              # 起始角度（弧度）
float32 angle_max              # 结束角度（弧度）
float32 angle_increment        # 角度增量（弧度）
float32 time_increment         # 时间增量（秒）
float32 scan_time              # 扫描时间（秒）
float32 range_min              # 最小距离（米）
float32 range_max              # 最大距离（米）
float32[] ranges               # 距离数组（米）
float32[] intensities         # 强度数组
```

**使用示例**：
```python
def scan_callback(msg):
    # 遍历所有扫描点
    for i in range(len(msg.ranges)):
        angle = msg.angle_min + i * msg.angle_increment
        distance = msg.ranges[i]
        
        if distance < msg.range_min or distance > msg.range_max:
            continue  # 无效数据
        
        # 转换为笛卡尔坐标
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
```

### sensor_msgs/Image（图像）

**完整定义**：
```python
std_msgs/Header header
uint32 height                  # 图像高度（像素）
uint32 width                   # 图像宽度（像素）
string encoding                # 编码格式
uint8 is_bigendian            # 字节序
uint32 step                    # 行长度（字节）
uint8[] data                   # 图像数据
```

**常用编码格式**：
- `bgr8`: BGR彩色（8位每通道）
- `rgb8`: RGB彩色（8位每通道）
- `mono8`: 灰度（8位）
- `mono16`: 灰度（16位）
- `32FC1`: 32位浮点单通道（深度图）
- `16UC1`: 16位无符号单通道（深度图）

### nav_msgs/OccupancyGrid（地图）

**完整定义**：
```python
std_msgs/Header header
nav_msgs/MapMetaData info
  time map_load_time
  float32 resolution          # 分辨率（米/像素）
  uint32 width                # 地图宽度（像素）
  uint32 height               # 地图高度（像素）
  geometry_msgs/Pose origin  # 地图原点
int8[] data                   # 地图数据
```

**数据值含义**：
- `-1`: 未知区域
- `0`: 自由空间
- `100`: 障碍物
- `1-99`: 代价（值越大，代价越高）

---

## 常见服务详细说明

### nav_msgs/GetPlan（路径规划服务）

**请求**：
```python
geometry_msgs/PoseStamped start  # 起始位姿
geometry_msgs/PoseStamped goal   # 目标位姿
float32 tolerance                # 容忍度（米）
```

**响应**：
```python
nav_msgs/Path plan               # 路径（位姿序列）
```

**使用示例**：
```python
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped

rospy.wait_for_service('/move_base/make_plan')
make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

start = PoseStamped()
start.header.frame_id = "map"
start.pose.position.x = 0.0
start.pose.position.y = 0.0

goal = PoseStamped()
goal.header.frame_id = "map"
goal.pose.position.x = 5.0
goal.pose.position.y = 3.0

resp = make_plan(start=start, goal=goal, tolerance=0.2)
if resp.plan.poses:
    rospy.loginfo(f"路径规划成功，路径长度: {len(resp.plan.poses)}个点")
```

---

## 常见节点功能说明

### move_base
- **包名**：`move_base`
- **功能**：全局路径规划和局部路径跟踪
- **配置文件**：
  - `costmap_common_params.yaml`: 代价地图通用参数
  - `local_costmap_params.yaml`: 局部代价地图参数
  - `global_costmap_params.yaml`: 全局代价地图参数
  - `base_local_planner_params.yaml`: 局部规划器参数
  - `global_planner_params.yaml`: 全局规划器参数

### amcl
- **包名**：`amcl`
- **功能**：自适应蒙特卡洛定位
- **输入**：激光扫描、地图、初始位姿
- **输出**：定位结果、粒子云

### map_server
- **包名**：`map_server`
- **功能**：地图服务器
- **启动**：`rosrun map_server map_server map.yaml`

### robot_state_publisher
- **包名**：`robot_state_publisher`
- **功能**：发布机器人TF变换
- **输入**：`/joint_states`
- **输出**：TF变换

---

## 消息转换工具

### CvBridge（图像转换）

```python
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

# ROS消息 → OpenCV
cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")

# OpenCV → ROS消息
ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
```

### TF转换

```python
import tf2_ros
import tf2_geometry_msgs

# 初始化
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# 查询变换
transform = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))

# 转换位姿
pose_stamped = PoseStamped()
pose_stamped.header.frame_id = "base_link"
pose_stamped.pose.position.x = 0.0
pose_stamped.pose.position.y = 0.0

pose_map = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
```

---

## 调试技巧

### 1. 查看话题图
```bash
# 文本方式
rosrun rqt_graph rqt_graph

# 或使用rqt
rqt
# 然后选择 Plugins -> Introspection -> Node Graph
```

### 2. 监控话题数据
```bash
# 实时查看
rostopic echo /topic_name

# 查看特定字段
rostopic echo /odom/pose/pose/position

# 查看频率
rostopic hz /topic_name

# 查看带宽
rostopic bw /topic_name
```

### 3. 录制和回放
```bash
# 录制
rosbag record -O my_bag.bag /topic1 /topic2

# 回放
rosbag play my_bag.bag

# 查看信息
rosbag info my_bag.bag
```

### 4. 检查TF树
```bash
# 生成PDF
rosrun tf view_frames

# 查看变换
rosrun tf tf_echo map base_link

# 监控TF
rosrun tf tf_monitor
```

---

**祝学习顺利！**

