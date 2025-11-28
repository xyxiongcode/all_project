# environment.py 文件详细解释

本文档详细解释 `src/isaac_sim/scripts/environment.py` 文件的每个部分，这是一个用于 Isaac Sim 与 ROS 集成的单机器人仿真连接脚本。

---

## 一、文件头部和导入部分（第1-43行）

### 1.1 文件说明和基础导入

```python
# This is an Isaac Sim Connection Scripts for single Carter-v1 robot
# Multi Robots simulation can inherit Class IsaacConnection
import geometry_msgs.msg
from omni.isaac.kit import SimulationApp
import os
```

**解释**：
- 文件注释说明这是用于单机器人 Carter-v1 的 Isaac Sim 连接脚本
- 多机器人仿真可以继承 `IsaacConnection` 类
- 导入 ROS 几何消息、Isaac Sim 的仿真应用和操作系统模块

### 1.2 USD 文件路径配置

```python
linux_user = os.getlogin()
CARTER_USD_PATH = f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/isaac/carter.usd"
```

**解释**：
- 获取当前 Linux 用户名
- 构建 Carter 机器人 USD 文件的完整路径
- USD（Universal Scene Description）是 Isaac Sim 使用的场景描述格式

### 1.3 仿真应用配置

```python
config = {
    "headless": False,
    # "active_gpu": 0,
    # "physics_gpu": 0,
    # "multi_gpu": False
}
simulation_app = SimulationApp(config)
```

**解释**：
- `headless: False`：启用图形界面（如果设为 `True` 则无头模式，适合服务器运行）
- 注释掉的 GPU 配置：可以指定用于渲染和物理计算的 GPU
- 创建仿真应用实例，这是 Isaac Sim 的入口点

### 1.4 工具库导入

```python
# utils
import sys
import numpy as np
from enum import Enum
import carb
import time
import threading
```

**解释**：
- `sys`：系统相关功能
- `numpy`：数值计算
- `Enum`：枚举类型（用于定义仿真状态）
- `carb`：Isaac Sim 的底层 API
- `time`：时间相关功能
- `threading`：多线程支持（用于线程锁）

### 1.5 Isaac Sim 核心模块导入

```python
# isaac
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.prims import XFormPrim, GeometryPrim
import omni.graph.core as og
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import RotatingLidarPhysX
from omni.isaac.range_sensor import _range_sensor
```

**解释**：
- `World`：Isaac Sim 的仿真世界，管理所有对象和物理模拟
- `enable_extension`：启用 Isaac Sim 扩展（如 ROS 桥接）
- `WheeledRobot`：轮式机器人基类
- `DifferentialController`：差速驱动控制器（用于两轮差速机器人）
- `XFormPrim`, `GeometryPrim`：场景图原语（Primitive），用于表示 3D 对象
- `og`：Omni Graph，用于创建节点图（如时钟发布）
- `add_reference_to_stage`：将 USD 文件添加到场景中
- `RotatingLidarPhysX`：旋转式 LiDAR 传感器
- `_range_sensor`：距离传感器接口

### 1.6 ROS 相关导入

```python
# ros
import rospy
from actionlib import SimpleActionServer
from std_srvs.srv import Empty, EmptyResponse
import rosgraph
from isaac_sim.msg import ResetPosesAction, ResetPosesResult, StepAction, StepResult
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
```

**解释**：
- `rospy`：ROS Python 客户端库
- `SimpleActionServer`：ROS Action 服务器（用于异步任务）
- `Empty`：ROS 空服务请求/响应
- `rosgraph`：ROS 图工具（用于检查 roscore 是否运行）
- 自定义消息类型：`ResetPosesAction`（复位位姿）、`StepAction`（单步执行）
- `Quaternion`：四元数消息类型
- `euler_from_quaternion`：四元数转欧拉角

---

## 二、仿真状态枚举（第46-51行）

```python
class SimulationState(Enum):
    RESET = 0
    PAUSE = 1
    STEP = 2
    NORMAL = 4
    CLOSE = 8
```

**解释**：
- **`RESET`**：复位状态，用于重置机器人位姿
- **`PAUSE`**：暂停状态，暂停仿真
- **`STEP`**：单步执行状态，执行一步仿真
- **`NORMAL`**：正常运行状态，连续仿真
- **`CLOSE`**：关闭状态，退出仿真

**设计说明**：使用位标志（0, 1, 2, 4, 8），理论上可以组合多个状态（虽然这里没有使用）

---

## 三、IsaacSimConnection 类（第54-269行）

### 3.1 初始化方法（`__init__`，第55-102行）

#### 3.1.1 ROS 扩展和检查

```python
def __init__(self, training_scene="env"):
    enable_extension("omni.isaac.ros_bridge")
    while not rosgraph.is_master_online():
        carb.log_error("Please run roscore before executing this script")
        time.sleep(2.0)
```

**解释**：
- `training_scene`：训练场景名称（默认 "env"）
- `enable_extension("omni.isaac.ros_bridge")`：启用 ROS 桥接扩展，使 Isaac Sim 可以与 ROS 通信
- 循环检查 `roscore` 是否运行，如果没有则等待并报错

#### 3.1.2 场景和线程锁初始化

```python
    # scene
    self.training_scene = training_scene

    # multi threading lock
    self.reset_lock = threading.Lock()
    self.state_lock = threading.Lock()
```

**解释**：
- 保存场景名称
- **`reset_lock`**：复位操作的线程锁，防止多线程同时执行复位
- **`state_lock`**：状态变更的线程锁，保护仿真状态的修改

#### 3.1.3 LiDAR 数据池初始化

```python
    # laser
    self.lidar_pool = []
    self.lidar_capacity = 100
```

**解释**：
- `lidar_pool`：LiDAR 数据池，存储历史扫描数据
- `lidar_capacity`：数据池容量（100 帧）

#### 3.1.4 复位位姿容器初始化

```python
    # reset pose containers
    self.reset_prefix = []
    self.reset_poses = []
    self.init_pose = (1.5, 1.5, 0.0)
    self.step_vel = np.array((0.0, 0.0))
    self.step_flag = False
    self.reset_flag = False
```

**解释**：
- `reset_prefix`：需要复位的机器人前缀列表（用于多机器人场景）
- `reset_poses`：对应的复位位姿列表 `(x, y, yaw)`
- `init_pose`：初始位姿 `(x, y, yaw)`，默认 `(1.5, 1.5, 0.0)`
- `step_vel`：单步执行的速度命令 `[forward, angular]`
- `step_flag`：单步执行完成标志
- `reset_flag`：复位完成标志

#### 3.1.5 机器人视图和仿真时间

```python
    # robot view
    self.robots = []

    # simulation time
    self.time = None
```

**解释**：
- `robots`：机器人对象列表（支持多机器人）
- `time`：仿真时间（当前未使用）

#### 3.1.6 ROS 服务和 Action 服务器

```python
    # ros
    self.close_server = rospy.Service("/close", Empty, self._close_callback)
    self.reset_server = SimpleActionServer("/reset", ResetPosesAction, self._reset_callback, auto_start=False)
    self.step_server = SimpleActionServer("/step", StepAction, self._step_callback, auto_start=False)
    self.reset_server.start()
    self.step_server.start()
```

**解释**：
- **`/close` 服务**：关闭仿真的服务，回调函数 `_close_callback`
- **`/reset` Action 服务器**：复位机器人位姿的 Action，回调函数 `_reset_callback`
- **`/step` Action 服务器**：单步执行仿真的 Action，回调函数 `_step_callback`
- `auto_start=False`：手动启动服务器（更灵活）

#### 3.1.7 场景和机器人初始化

```python
    # scene
    self.world = World(stage_units_in_meters=1.0, physics_dt=0.05, rendering_dt=0.05)
    if self.training_scene == "env":
        self.world.scene.add_default_ground_plane()
    self._add_robot()
    self._add_env()
    self._add_action_graph()

    # simulation state
    self.state = SimulationState.NORMAL
```

**解释**：
- **创建仿真世界**：
  - `stage_units_in_meters=1.0`：场景单位（1 单位 = 1 米）
  - `physics_dt=0.05`：物理时间步长（0.05 秒 = 20 Hz）
  - `rendering_dt=0.05`：渲染时间步长（0.05 秒 = 20 FPS）
- 如果是 "env" 场景，添加默认地面
- 调用方法添加机器人、环境和动作图
- 初始状态设为 `NORMAL`（正常运行）

---

### 3.2 主循环（`cycle`，第104-135行）

```python
def cycle(self):
    self.world.reset()
    self.carter_lidar.enable_visualization()
    self.carter_lidar.add_depth_data_to_frame()
    while simulation_app.is_running():
        if self.state == SimulationState.NORMAL:
            self.world.step()
            self._store_laser()
        elif self.state == SimulationState.PAUSE:
            self.world.pause()
        elif self.state == SimulationState.STEP:
            self.robot.apply_wheel_actions(self.controller.forward(command=self.step_vel))
            self.world.play()
            self._store_laser()
            with self.state_lock:
                self.state = SimulationState.PAUSE
                self.step_flag = True
        elif self.state == SimulationState.RESET:
            self.world.pause()
            self._reset_process()
            self.robot.apply_wheel_actions(self.controller.forward(command=np.array((0., 0.))))
            self.world.play()
            self._store_laser()
            with self.state_lock:
                self.state = SimulationState.NORMAL
                self.reset_flag = True
        elif self.state == SimulationState.CLOSE:
            break
    print("simulation app is out of running")
    self.world.stop()
    simulation_app.close()
```

**执行流程**：

1. **初始化**：
   - 重置世界
   - 启用 LiDAR 可视化
   - 添加深度数据到帧

2. **主循环**（根据状态执行不同操作）：
   - **`NORMAL`**：正常仿真，每步执行 `world.step()` 并存储 LiDAR 数据
   - **`PAUSE`**：暂停仿真
   - **`STEP`**：
     - 应用速度命令到机器人
     - 执行一步仿真
     - 存储 LiDAR 数据
     - 切换回 `PAUSE` 状态，设置 `step_flag = True`
   - **`RESET`**：
     - 暂停仿真
     - 执行复位过程
     - 应用零速度命令
     - 恢复仿真
     - 存储 LiDAR 数据
     - 切换回 `NORMAL` 状态，设置 `reset_flag = True`
   - **`CLOSE`**：退出循环

3. **清理**：停止世界并关闭仿真应用

---

### 3.3 复位过程（`_reset_process`，第137-151行）

```python
def _reset_process(self):
    with self.reset_lock:
        for i, prefix in enumerate(self.reset_prefix):
            try:
                x, y, yaw = self.reset_poses[i]
                position = np.array([x, y, 0.3])
                orientation = np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])
                self.robots[prefix].set_world_pose(
                    position=position, orientation=orientation
                )
            except IndexError:
                rospy.logerr("reset msg contains error prefix, reset later")
        self.reset_prefix.clear()
        self.reset_poses.clear()
        self.lidar_pool.clear()
```

**解释**：
- 使用 `reset_lock` 保护，防止并发复位
- 遍历需要复位的机器人：
  - 从 `reset_poses` 获取位姿 `(x, y, yaw)`
  - 构建位置 `[x, y, 0.3]`（z=0.3 米，机器人高度）
  - 将 yaw 角转换为四元数：`[cos(yaw/2), 0, 0, sin(yaw/2)]`（绕 z 轴旋转）
  - 设置机器人世界位姿
- 清空复位列表和 LiDAR 数据池

**注意**：四元数格式为 `[w, x, y, z]`，这里只绕 z 轴旋转（yaw）

---

### 3.4 存储 LiDAR 数据（`_store_laser`，第153-157行）

```python
def _store_laser(self):
    cur_laser = self.lidarInterface.get_linear_depth_data(self.lidar_path)
    if len(self.lidar_pool) >= self.lidar_capacity:
        self.lidar_pool.pop(0)
    self.lidar_pool.append(cur_laser)
```

**解释**：
- 从 LiDAR 接口获取当前线性深度数据
- 如果数据池已满（>= 100），移除最旧的数据（FIFO）
- 添加当前数据到数据池

---

### 3.5 添加机器人（`_add_robot`，第159-185行）

```python
def _add_robot(self):
    wheel_dof_names = ["left_wheel", "right_wheel"]
    self.robot = self.world.scene.add(
        WheeledRobot(
            prim_path="/World/Carters/Carter_0",
            name="Carter_0",
            wheel_dof_names=wheel_dof_names,
            create_robot=True,
            usd_path=CARTER_USD_PATH,
            position=np.array([self.init_pose[0], self.init_pose[1], 0.3]),
            orientation=np.array([np.cos(self.init_pose[2] / 2), 0.0, 0.0, np.sin(self.init_pose[2] / 2)])
        )
    )
```

**解释**：
- **轮子自由度名称**：`["left_wheel", "right_wheel"]`（左轮、右轮）
- **创建轮式机器人**：
  - `prim_path`：场景图中的路径 `/World/Carters/Carter_0`
  - `name`：机器人名称 `Carter_0`
  - `wheel_dof_names`：轮子自由度名称（用于差速驱动）
  - `create_robot=True`：创建机器人对象
  - `usd_path`：USD 文件路径
  - `position`：初始位置 `[x, y, 0.3]`（z=0.3 米）
  - `orientation`：初始朝向（四元数，从 yaw 角转换）

```python
    self.lidar_path = "/World/Carters/Carter_0/chassis_link/lidar"
    self.carter_lidar = self.world.scene.add(
        RotatingLidarPhysX(
            prim_path=self.lidar_path,
            name="lidar",
            rotation_frequency=0,
            translation=np.array([-0.06, 0, 0.38]),
            fov=(270, 0.0), resolution=(0.25, 0.0),
            valid_range=(0.4, 10.0)
        )
    )
```

**解释**：
- **LiDAR 路径**：`/World/Carters/Carter_0/chassis_link/lidar`（在底盘链接上）
- **创建旋转 LiDAR**：
  - `rotation_frequency=0`：不旋转（固定 LiDAR）
  - `translation`：相对于底盘的位置 `[-0.06, 0, 0.38]`（向前 6cm，向上 38cm）
  - `fov=(270, 0.0)`：视场角 270 度水平，0 度垂直（2D LiDAR）
  - `resolution=(0.25, 0.0)`：分辨率 0.25 度水平
  - `valid_range=(0.4, 10.0)`：有效范围 0.4-10 米

```python
    self.lidarInterface = _range_sensor.acquire_lidar_sensor_interface()
    self.controller = DifferentialController(name="simple_control", wheel_radius=0.24, wheel_base=0.56)
    self.robots.append(self.robot)
```

**解释**：
- 获取 LiDAR 传感器接口
- **创建差速控制器**：
  - `wheel_radius=0.24`：轮子半径 0.24 米
  - `wheel_base=0.56`：轮距 0.56 米（两轮中心距离）
- 将机器人添加到列表

---

### 3.6 添加环境（`_add_env`，第187-198行）

```python
def _add_env(self):
    env_usd_path = f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/isaac/{self.training_scene}.usd"
    add_reference_to_stage(usd_path=env_usd_path, prim_path="/World/Envs/Env_0")
    self.world.scene.add(
        GeometryPrim(
            prim_path="/World/Envs/Env_0",
            name="Env",
            collision=True,
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0])
        )
    )
```

**解释**：
- 构建环境 USD 文件路径（根据场景名称）
- 将 USD 文件添加到场景图路径 `/World/Envs/Env_0`
- 创建几何原语：
  - `collision=True`：启用碰撞检测
  - 位置和朝向（默认值）

---

### 3.7 添加动作图（`_add_action_graph`，第200-219行）

```python
@staticmethod
def _add_action_graph():
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/clock_graph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("ClockPub", "omni.isaac.ros_bridge.ROS1PublishClock"),
            ],
            keys.SET_VALUES: [
                ("ClockPub.inputs:topicName", "clock"),
            ],
            keys.CONNECT: [
                ("SimTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
                ("OnTick.outputs:tick", "ClockPub.inputs:execIn"),
            ]
        }
    )
```

**解释**：
- **静态方法**：不依赖实例，可以直接调用
- **创建 Omni Graph**：用于发布 ROS 时钟
- **节点**：
  - `OnTick`：播放时触发（每帧）
  - `SimTime`：读取仿真时间
  - `ClockPub`：发布 ROS 时钟消息
- **连接**：
  - 仿真时间 → 时钟发布的时间戳
  - 播放事件 → 时钟发布的执行输入
- **作用**：同步 ROS 时间与仿真时间，发布到 `/clock` 话题

---

### 3.8 辅助方法：获取 Yaw 角（`_get_yaw`，第221-224行）

```python
@staticmethod
def _get_yaw(quaternion: Quaternion):
    _, _, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return yaw
```

**解释**：
- 从四元数提取 yaw 角（绕 z 轴旋转）
- 忽略 roll 和 pitch（`_` 表示未使用）

---

### 3.9 单步执行回调（`_step_callback`，第226-238行）

```python
def _step_callback(self, msg):
    with self.state_lock:
        self.step_vel = np.array((msg.forward, msg.angular))
        self.state = SimulationState.STEP
        self.step_flag = False
    # rospy.loginfo("finish lock")
    while not self.step_flag:
        time.sleep(0.05)
    # rospy.loginfo("finish stepping")
    result = StepResult()
    result.frames = 6
    result.ranges = np.array(self._get_lidar()).flatten().tolist()
    self.step_server.set_succeeded(result=result)
```

**解释**：
1. **设置状态**（加锁）：
   - 保存速度命令 `[forward, angular]`
   - 切换到 `STEP` 状态
   - 重置 `step_flag`
2. **等待执行完成**：循环等待 `step_flag` 变为 `True`（在主循环中设置）
3. **返回结果**：
   - 创建 `StepResult`
   - 设置帧数（6 帧）
   - 获取 LiDAR 数据并展平为列表
   - 设置 Action 为成功

---

### 3.10 复位回调（`_reset_callback`，第240-254行）

```python
def _reset_callback(self, msg):
    with self.reset_lock and self.state_lock:
        self.state = SimulationState.RESET
        self.reset_flag = False
        for i, prefix in enumerate(msg.prefix):
            pose = msg.poses[i]
            self.reset_prefix.append(prefix)
            yaw = self._get_yaw(pose.pose.orientation)
            self.reset_poses.append((pose.pose.position.x, pose.pose.position.y, yaw))
    while not self.reset_flag:
        time.sleep(0.05)
    result = ResetPosesResult()
    result.frames = 6
    result.ranges = np.array(self._get_lidar()).flatten().tolist()
    self.reset_server.set_succeeded(result=result)
```

**解释**：
1. **保存复位请求**（加锁）：
   - 切换到 `RESET` 状态
   - 重置 `reset_flag`
   - 遍历消息中的机器人前缀和位姿：
     - 保存前缀
     - 提取 yaw 角
     - 保存位姿 `(x, y, yaw)`
2. **等待复位完成**：循环等待 `reset_flag` 变为 `True`
3. **返回结果**：与 `_step_callback` 类似，返回 LiDAR 数据

---

### 3.11 获取 LiDAR 数据（`_get_lidar`，第256-264行）

```python
def _get_lidar(self, frames=6):
    lasers = [self.lidar_pool[-1] for _ in range(frames)]
    for i in range(frames - 1, 0, -1):
        prefix = len(self.lidar_pool) - i * 10
        if prefix < 0:
            continue
        else:
            lasers[frames - i - 1] = self.lidar_pool[prefix]
    return lasers
```

**解释**：
- 默认返回 6 帧 LiDAR 数据
- **策略**：
  - 初始化为最新的数据（`lidar_pool[-1]`）
  - 从历史数据中选择：
    - 第 1 帧：最新（索引 `-1`）
    - 第 2 帧：10 步前（索引 `len - 10`）
    - 第 3 帧：20 步前（索引 `len - 20`）
    - ...
  - 如果索引 < 0，跳过（数据不足）
- **用途**：提供历史 LiDAR 数据，可能用于时序分析或训练

---

### 3.12 关闭回调（`_close_callback`，第266-269行）

```python
def _close_callback(self, msg):
    with self.state_lock:
        self.state = SimulationState.CLOSE
    return EmptyResponse()
```

**解释**：
- 切换到 `CLOSE` 状态
- 返回空响应
- 主循环检测到 `CLOSE` 状态后会退出

---

## 四、主程序入口（第272-280行）

```python
if __name__ == "__main__":
    rospy.init_node("IsaacSimConnection")
    if len(sys.argv) > 1:
        scene = sys.argv[1]
        assert scene in ["env", "warehouse", "hospital", "office"]
        connection = IsaacSimConnection(scene)
    else:
        connection = IsaacSimConnection()
    connection.cycle()
```

**解释**：
1. **初始化 ROS 节点**：`IsaacSimConnection`
2. **处理命令行参数**：
   - 如果提供了场景参数，验证是否为有效场景（"env", "warehouse", "hospital", "office"）
   - 使用指定场景创建连接
   - 否则使用默认场景（"env"）
3. **启动主循环**：`cycle()`

**使用示例**：
```bash
# 使用默认场景
python environment.py

# 使用指定场景
python environment.py warehouse
```

---

## 五、关键设计模式

### 5.1 状态机模式
- 使用 `SimulationState` 枚举管理仿真状态
- 主循环根据状态执行不同操作

### 5.2 线程安全
- 使用 `threading.Lock` 保护共享资源
- `reset_lock`：保护复位操作
- `state_lock`：保护状态变更

### 5.3 Action 服务器模式
- 使用 ROS Action 处理异步任务（复位、单步执行）
- 通过标志位（`step_flag`, `reset_flag`）同步主循环和回调

### 5.4 数据池模式
- LiDAR 数据池存储历史数据
- FIFO 队列，容量限制为 100 帧

---

## 六、关键参数说明

### 6.1 机器人参数
- **轮子半径**：0.24 米
- **轮距**：0.56 米
- **初始位姿**：`(1.5, 1.5, 0.0)` 米/弧度

### 6.2 LiDAR 参数
- **视场角**：270 度（水平）
- **分辨率**：0.25 度
- **有效范围**：0.4-10 米
- **位置**：向前 6cm，向上 38cm（相对于底盘）

### 6.3 仿真参数
- **物理时间步长**：0.05 秒（20 Hz）
- **渲染时间步长**：0.05 秒（20 FPS）
- **场景单位**：1 单位 = 1 米

---

## 七、扩展建议

1. **多机器人支持**：当前代码已预留接口（`robots` 列表），可以扩展支持多机器人
2. **更多传感器**：可以添加摄像头、IMU 等传感器
3. **动态环境**：可以添加动态障碍物或移动目标
4. **性能优化**：可以调整物理和渲染时间步长以提高性能

