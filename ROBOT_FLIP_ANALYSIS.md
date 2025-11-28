# 机器人侧翻问题分析与修复方案

## 问题描述

两轮差速机器人在仿真中运行一段时间后经常发生侧翻，影响仿真稳定性和数据收集。

---

## 一、可能原因分析

### 1.1 物理时间步长过大（第94行）

```python
self.world = World(stage_units_in_meters=1.0, physics_dt=0.05, rendering_dt=0.05)
```

**问题**：
- `physics_dt=0.05` 秒（20 Hz）对于快速移动的机器人来说可能不够精确
- 较大的时间步长会导致数值积分误差累积
- 在高速运动或急转弯时，物理引擎可能无法准确计算，导致不稳定

**影响**：
- 加速度计算不准确
- 角速度变化剧烈时容易失稳
- 碰撞检测可能不准确

---

### 1.2 缺少物理稳定性设置

**问题**：
- 没有设置物理稳定性参数（稳定器、阻尼等）
- Isaac Sim 默认的物理参数可能不适合两轮机器人
- 没有添加防止侧翻的约束

**影响**：
- 机器人容易在惯性作用下侧翻
- 急转弯时离心力过大
- 没有恢复力矩

---

### 1.3 地面摩擦力可能不足

```python
if self.training_scene == "env":
    self.world.scene.add_default_ground_plane()
```

**问题**：
- 使用默认地面，可能摩擦力设置不当
- 没有明确设置地面材质属性
- 可能缺少足够的静摩擦和动摩擦

**影响**：
- 轮子打滑
- 转弯时侧滑
- 加速时后轮打滑导致失稳

---

### 1.4 机器人物理属性未明确设置

```python
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

**问题**：
- 没有设置机器人的质量分布
- 没有设置重心位置
- 没有设置惯性矩阵
- 没有设置阻尼参数

**影响**：
- 重心可能过高，导致不稳定
- 缺少阻尼导致振荡
- 惯性设置不当导致运动不自然

---

### 1.5 控制器缺少速度限制

```python
self.controller = DifferentialController(name="simple_control", wheel_radius=0.24, wheel_base=0.56)
```

**问题**：
- `DifferentialController` 可能没有内置速度限制
- 没有检查速度命令是否过大
- 没有加速度限制

**影响**：
- 速度命令过大导致急加速/急减速
- 角速度过大导致急转弯
- 惯性力过大导致侧翻

---

### 1.6 LiDAR 位置可能影响重心

```python
self.carter_lidar = self.world.scene.add(
    RotatingLidarPhysX(
        prim_path=self.lidar_path,
        name="lidar",
        rotation_frequency=0,
        translation=np.array([-0.06, 0, 0.38]),  # 向上38cm
        ...
    )
)
```

**问题**：
- LiDAR 位于机器人上方 38cm，增加了重心高度
- 如果 LiDAR 质量较大，会显著影响重心位置
- 没有补偿重心的偏移

**影响**：
- 重心上移导致稳定性下降
- 转弯时更容易侧翻

---

### 1.7 缺少速度平滑处理

**问题**：
- 速度命令直接应用，没有平滑处理
- 速度突变会导致大的加速度
- 没有速度限制检查

**影响**：
- 急加速/急减速
- 急转弯
- 惯性力过大

---

## 二、修复方案

### 2.1 减小物理时间步长（推荐）

**修改位置**：第94行

```python
# 原代码
self.world = World(stage_units_in_meters=1.0, physics_dt=0.05, rendering_dt=0.05)

# 修改为（提高物理更新频率到50Hz或更高）
self.world = World(stage_units_in_meters=1.0, physics_dt=0.02, rendering_dt=0.05)
# 或者
self.world = World(stage_units_in_meters=1.0, physics_dt=0.01, rendering_dt=0.05)
```

**说明**：
- `physics_dt=0.02`：50 Hz 物理更新（推荐）
- `physics_dt=0.01`：100 Hz 物理更新（更稳定但计算量更大）
- `rendering_dt` 可以保持 0.05（20 FPS 足够）

**权衡**：
- ✅ 提高稳定性
- ❌ 增加计算量（但现代GPU通常可以承受）

---

### 2.2 添加物理稳定性设置

**修改位置**：在 `_add_robot()` 方法后添加

```python
def _add_robot(self):
    # ... 原有代码 ...
    self.robots.append(self.robot)
    
    # 新增：设置物理稳定性参数
    self._configure_robot_physics()
    
def _configure_robot_physics(self):
    """配置机器人物理属性以提高稳定性"""
    # 获取机器人的物理属性
    from omni.isaac.core.utils.prims import get_prim_at_path
    
    # 设置底盘（chassis）的物理属性
    chassis_path = "/World/Carters/Carter_0/chassis_link"
    chassis_prim = get_prim_at_path(chassis_path)
    
    if chassis_prim:
        # 设置阻尼（增加稳定性）
        from pxr import PhysxSchema
        physx_rigid_body = PhysxSchema.PhysxRigidBodyAPI.Get(chassis_prim, "physics")
        if physx_rigid_body:
            # 增加线性阻尼和角阻尼
            physx_rigid_body.GetLinearDampingAttr().Set(0.1)  # 线性阻尼
            physx_rigid_body.GetAngularDampingAttr().Set(0.2)  # 角阻尼（防止旋转过快）
            
        # 设置稳定器（Stabilization）
        physx_rigid_body.GetStabilizationEnabledAttr().Set(True)
        physx_rigid_body.GetStabilizationThresholdAttr().Set(0.01)
        
    # 设置轮子的物理属性
    wheel_paths = [
        "/World/Carters/Carter_0/left_wheel",
        "/World/Carters/Carter_0/right_wheel"
    ]
    
    for wheel_path in wheel_paths:
        wheel_prim = get_prim_at_path(wheel_path)
        if wheel_prim:
            physx_rigid_body = PhysxSchema.PhysxRigidBodyAPI.Get(wheel_prim, "physics")
            if physx_rigid_body:
                # 增加轮子阻尼
                physx_rigid_body.GetLinearDampingAttr().Set(0.05)
                physx_rigid_body.GetAngularDampingAttr().Set(0.1)
```

**说明**：
- **线性阻尼**：减少线性速度，防止过度加速
- **角阻尼**：减少角速度，防止急转弯
- **稳定器**：当机器人接近不稳定状态时自动稳定

---

### 2.3 改进地面摩擦力设置

**修改位置**：第95-96行

```python
# 原代码
if self.training_scene == "env":
    self.world.scene.add_default_ground_plane()

# 修改为：创建自定义地面，设置合适的摩擦力
if self.training_scene == "env":
    self._add_ground_plane_with_friction()

def _add_ground_plane_with_friction(self):
    """添加具有合适摩擦力的地面"""
    from omni.isaac.core.materials import PreviewSurface
    
    # 创建地面材质
    ground_material = PreviewSurface(
        prim_path="/World/GroundMaterial",
        color=np.array([0.5, 0.5, 0.5]),
        roughness=0.8,  # 增加粗糙度以提高摩擦力
    )
    
    # 添加地面
    ground_plane = self.world.scene.add(
        GeometryPrim(
            prim_path="/World/GroundPlane",
            name="GroundPlane",
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            scale=np.array([100.0, 100.0, 1.0]),  # 大平面
            collision=True,
        )
    )
    
    # 设置物理材质属性
    from pxr import PhysxSchema
    ground_prim = ground_plane.prim
    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(ground_prim, "physics")
    if physx_material:
        physx_material.GetStaticFrictionAttr().Set(1.0)  # 静摩擦系数
        physx_material.GetDynamicFrictionAttr().Set(0.8)  # 动摩擦系数
        physx_material.GetRestitutionAttr().Set(0.0)  # 恢复系数（无弹性）
```

**说明**：
- **静摩擦系数 1.0**：防止轮子打滑
- **动摩擦系数 0.8**：运动时仍有足够摩擦力
- **恢复系数 0.0**：完全非弹性碰撞，减少反弹

---

### 2.4 添加速度限制和平滑处理

**修改位置**：在类中添加速度限制方法，并在应用速度前调用

```python
def __init__(self, ...):
    # ... 原有代码 ...
    
    # 新增：速度限制参数
    self.max_linear_vel = 1.5  # m/s，最大线速度
    self.max_angular_vel = 1.0  # rad/s，最大角速度
    self.max_linear_accel = 0.5  # m/s²，最大线加速度
    self.max_angular_accel = 1.0  # rad/s²，最大角加速度
    
    # 当前速度（用于平滑）
    self.current_linear_vel = 0.0
    self.current_angular_vel = 0.0

def _limit_and_smooth_velocity(self, linear_cmd, angular_cmd):
    """限制和平滑速度命令"""
    # 限制速度命令
    linear_cmd = np.clip(linear_cmd, -self.max_linear_vel, self.max_linear_vel)
    angular_cmd = np.clip(angular_cmd, -self.max_angular_vel, self.max_angular_vel)
    
    # 计算加速度限制
    dt = self.world.get_physics_dt()
    max_linear_change = self.max_linear_accel * dt
    max_angular_change = self.max_angular_accel * dt
    
    # 平滑速度变化
    linear_diff = linear_cmd - self.current_linear_vel
    angular_diff = angular_cmd - self.current_angular_vel
    
    if abs(linear_diff) > max_linear_change:
        linear_cmd = self.current_linear_vel + np.sign(linear_diff) * max_linear_change
    
    if abs(angular_diff) > max_angular_change:
        angular_cmd = self.current_angular_vel + np.sign(angular_diff) * max_angular_change
    
    # 更新当前速度
    self.current_linear_vel = linear_cmd
    self.current_angular_vel = angular_cmd
    
    return linear_cmd, angular_cmd

# 修改 cycle() 方法中的速度应用
def cycle(self):
    # ... 原有代码 ...
    elif self.state == SimulationState.STEP:
        # 限制和平滑速度
        linear_cmd, angular_cmd = self._limit_and_smooth_velocity(
            self.step_vel[0], self.step_vel[1]
        )
        self.robot.apply_wheel_actions(
            self.controller.forward(command=np.array([linear_cmd, angular_cmd]))
        )
        # ... 其余代码 ...
```

**说明**：
- **速度限制**：防止速度过大
- **加速度限制**：防止急加速/急减速
- **平滑处理**：逐步改变速度，避免突变

---

### 2.5 调整 LiDAR 位置或补偿重心

**选项1：降低 LiDAR 位置**

```python
# 原代码
translation=np.array([-0.06, 0, 0.38])

# 修改为：降低 LiDAR 高度
translation=np.array([-0.06, 0, 0.25])  # 从38cm降到25cm
```

**选项2：在底盘添加配重（降低重心）**

```python
def _add_robot(self):
    # ... 原有代码 ...
    
    # 添加配重（可选）
    self._add_counterweight()

def _add_counterweight(self):
    """在底盘底部添加配重以降低重心"""
    from omni.isaac.core.prims import GeometryPrim
    
    counterweight = self.world.scene.add(
        GeometryPrim(
            prim_path="/World/Carters/Carter_0/chassis_link/counterweight",
            name="Counterweight",
            position=np.array([0.0, 0.0, -0.1]),  # 在底盘下方
            scale=np.array([0.3, 0.3, 0.05]),  # 小但重的配重
            collision=True,
        )
    )
    
    # 设置配重质量（增加底盘质量）
    from pxr import PhysxSchema
    prim = counterweight.prim
    physx_rigid_body = PhysxSchema.PhysxRigidBodyAPI.Get(prim, "physics")
    if physx_rigid_body:
        physx_rigid_body.GetMassAttr().Set(5.0)  # 5kg配重
```

---

### 2.6 添加侧翻检测和自动恢复（可选）

```python
def cycle(self):
    # ... 原有代码 ...
    
    if self.state == SimulationState.NORMAL:
        self.world.step()
        self._store_laser()
        
        # 新增：检查侧翻
        if self._check_rollover():
            rospy.logwarn("检测到机器人侧翻，执行恢复")
            self._recover_from_rollover()

def _check_rollover(self):
    """检查机器人是否侧翻"""
    # 获取机器人姿态
    pose = self.robot.get_world_pose()
    orientation = pose[1]  # 四元数 [w, x, y, z]
    
    # 转换为欧拉角
    from tf.transformations import euler_from_quaternion
    roll, pitch, _ = euler_from_quaternion([orientation[1], orientation[2], orientation[3], orientation[0]])
    
    # 检查 roll 或 pitch 是否过大（>30度）
    roll_deg = abs(np.degrees(roll))
    pitch_deg = abs(np.degrees(pitch))
    
    return roll_deg > 30.0 or pitch_deg > 30.0

def _recover_from_rollover(self):
    """从侧翻中恢复"""
    # 停止机器人
    self.robot.apply_wheel_actions(self.controller.forward(command=np.array([0.0, 0.0])))
    
    # 重置位姿到初始位置
    position = np.array([self.init_pose[0], self.init_pose[1], 0.3])
    orientation = np.array([np.cos(self.init_pose[2] / 2), 0.0, 0.0, np.sin(self.init_pose[2] / 2)])
    self.robot.set_world_pose(position=position, orientation=orientation)
    
    # 重置速度
    self.current_linear_vel = 0.0
    self.current_angular_vel = 0.0
```

---

## 三、推荐的修复优先级

### 高优先级（必须修复）

1. **减小物理时间步长**（2.1）
   - 影响：⭐⭐⭐⭐⭐
   - 难度：⭐
   - 效果：显著提高稳定性

2. **添加速度限制和平滑**（2.4）
   - 影响：⭐⭐⭐⭐⭐
   - 难度：⭐⭐
   - 效果：防止急加速/急转弯

### 中优先级（强烈推荐）

3. **添加物理稳定性设置**（2.2）
   - 影响：⭐⭐⭐⭐
   - 难度：⭐⭐⭐
   - 效果：提高稳定性

4. **改进地面摩擦力**（2.3）
   - 影响：⭐⭐⭐
   - 难度：⭐⭐
   - 效果：减少打滑

### 低优先级（可选）

5. **调整 LiDAR 位置**（2.5）
   - 影响：⭐⭐
   - 难度：⭐
   - 效果：轻微改善

6. **添加侧翻检测和恢复**（2.6）
   - 影响：⭐⭐
   - 难度：⭐⭐⭐
   - 效果：自动恢复，但不解决根本问题

---

## 四、完整修复示例代码

见 `environment_fixed.py` 文件（如果创建）

---

## 五、测试建议

1. **逐步测试**：每次只应用一个修复，观察效果
2. **记录参数**：记录修复前后的侧翻频率
3. **性能监控**：注意修复对仿真性能的影响
4. **场景测试**：在不同场景（env, warehouse等）中测试

---

## 六、其他注意事项

1. **USD 文件检查**：检查 `carter.usd` 文件中的物理属性设置
2. **控制器参数**：检查 `DifferentialController` 是否有内置限制
3. **导航参数**：检查 move_base 的速度限制参数
4. **环境障碍物**：检查环境中是否有导致侧翻的障碍物

