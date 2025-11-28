# 机器人修改指南：USD文件说明和四轮机器人配置

## USD文件包含的机器人数据

USD（Universal Scene Description）文件定义了机器人的以下信息：

### 1. **几何结构（Geometry）**
- 机器人的3D模型（mesh）
- 各个部件的形状和尺寸
- 视觉外观（材质、纹理）

### 2. **关节定义（Joints）**
- 关节类型（旋转关节、滑动关节等）
- 关节名称（如 "left_wheel", "right_wheel"）
- 关节的运动范围和限制

### 3. **物理属性（Physics）**
- 质量（mass）
- 惯性矩阵（inertia）
- 碰撞形状（collision shapes）
- 摩擦系数、阻尼等

### 4. **层级结构（Hierarchy）**
- 机器人部件的父子关系
- 坐标系变换（transform）
- 部件之间的连接关系

### 5. **传感器定义**
- 传感器位置和朝向
- 传感器参数（LiDAR、相机等）

### 6. **驱动系统**
- 驱动器类型
- 最大速度和力矩限制

## 如何将机器人改为四轮机器人

### 方案1：使用现有的四轮机器人USD文件（推荐）

如果您有现成的四轮机器人USD文件：

1. **修改USD文件路径**（environment.py 第8行）：
```python
CARTER_USD_PATH = f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/isaac/your_four_wheel_robot.usd"
```

2. **修改轮子关节名称**（environment.py 第160行）：
```python
# 两轮差速驱动
wheel_dof_names = ["left_wheel", "right_wheel"]

# 四轮差速驱动（前轮和后轮分别控制）
wheel_dof_names = ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"]

# 或者四轮独立驱动
wheel_dof_names = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"]
```

3. **修改控制器**（environment.py 第184行）：
```python
# 两轮差速驱动控制器（当前）
self.controller = DifferentialController(
    name="simple_control", 
    wheel_radius=0.24, 
    wheel_base=0.56
)

# 四轮差速驱动（需要根据机器人类型选择）
# 选项1：如果四轮机器人使用差速驱动（前轮和后轮分别控制左右）
# 可能需要自定义控制器或使用其他控制器类型

# 选项2：如果四轮机器人是独立驱动（每个轮子独立控制）
# 需要使用不同的控制器，可能需要直接控制关节速度
```

### 方案2：从URDF导入四轮机器人

如果您有URDF格式的四轮机器人模型：

1. **在Isaac Sim中导入URDF**：
   - 打开Isaac Sim
   - File → Import → URDF
   - 选择您的URDF文件
   - 保存为USD格式

2. **修改代码使用新的USD文件**

### 方案3：修改现有USD文件（需要USD编辑工具）

使用Omniverse或支持USD的软件（如Blender）编辑USD文件：

1. **添加两个额外的轮子**
2. **定义轮子关节**
3. **设置物理属性**
4. **保存修改后的USD文件**

## 代码修改示例：四轮差速驱动机器人

假设您有一个四轮差速驱动机器人（前轮和后轮分别控制左右）：

```python
def _add_robot(self):
    # 四轮差速驱动：前轮和后轮分别控制
    wheel_dof_names = [
        "front_left_wheel", 
        "front_right_wheel", 
        "rear_left_wheel", 
        "rear_right_wheel"
    ]
    
    self.robot = self.world.scene.add(
        WheeledRobot(
            prim_path="/World/Robots/Robot_0",
            name="Robot_0",
            wheel_dof_names=wheel_dof_names,
            create_robot=True,
            usd_path=FOUR_WHEEL_ROBOT_USD_PATH,  # 新的USD文件路径
            position=np.array([self.init_pose[0], self.init_pose[1], 0.3]),
            orientation=np.array([np.cos(self.init_pose[2] / 2), 0.0, 0.0, np.sin(self.init_pose[2] / 2)])
        )
    )
    
    # 对于四轮差速驱动，可能需要自定义控制器
    # 或者使用关节速度直接控制
    # self.controller = CustomFourWheelController(...)
```

## 注意事项

1. **控制器兼容性**：
   - `DifferentialController` 专为两轮差速驱动设计
   - 四轮机器人可能需要自定义控制器
   - 需要根据驱动方式（差速、全轮驱动、独立驱动）选择或实现控制器

2. **轮子关节名称**：
   - 必须与USD文件中定义的关节名称完全匹配
   - 可以在Isaac Sim中打开USD文件查看关节名称

3. **物理参数**：
   - 轮子半径、轮距等参数需要根据实际机器人调整
   - 四轮机器人的轮距计算方式可能不同

4. **LiDAR路径**：
   - 如果机器人结构不同，LiDAR的路径可能需要修改
   - 当前路径：`/World/Carters/Carter_0/chassis_link/lidar`

## 检查USD文件内容的方法

1. **在Isaac Sim中打开USD文件**：
   ```bash
   # 启动Isaac Sim后，File → Open → 选择USD文件
   ```

2. **查看关节名称**：
   - 在Stage面板中展开机器人层级
   - 查找关节（Joints）节点
   - 查看关节名称

3. **使用Python脚本检查**：
   ```python
   from pxr import Usd, UsdGeom
   stage = Usd.Stage.Open("path/to/robot.usd")
   # 遍历所有prim，查找关节
   ```

## 推荐步骤

1. **确认四轮机器人USD文件**：
   - 检查是否有现成的四轮机器人USD文件
   - 或准备从URDF转换

2. **查看USD文件中的关节名称**：
   - 在Isaac Sim中打开USD文件
   - 记录所有轮子关节的名称

3. **修改代码**：
   - 更新USD文件路径
   - 更新轮子关节名称列表
   - 根据需要修改或实现控制器

4. **测试**：
   - 运行修改后的代码
   - 检查机器人是否能正常移动
   - 调整物理参数

