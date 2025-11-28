# 减少机器人侧滑的额外改进方法

本文档提供更多减少两轮差速机器人侧滑的方法，可以添加到 `environment.py` 文件中。

## 已实现的改进

1. ✅ 减小物理时间步长（0.05 → 0.02秒）
2. ✅ 速度限制和平滑处理
3. ✅ 物理阻尼设置（底盘和轮子）
4. ✅ 降低LiDAR位置（0.38 → 0.25米）

## 额外改进方法

### 1. 设置轮子摩擦力（高优先级）

**问题**：轮子与地面的摩擦力不足会导致打滑和侧滑

**修复**：为轮子设置物理材质，增加摩擦系数

```python
def _configure_robot_physics(self):
    # ... 现有代码 ...
    
    # 新增：设置轮子摩擦力
    self._configure_wheel_friction()
    
def _configure_wheel_friction(self):
    """配置轮子摩擦力，减少打滑"""
    try:
        from omni.isaac.core.utils.prims import get_prim_at_path
        from pxr import PhysxSchema
        
        wheel_paths = [
            "/World/Carters/Carter_0/left_wheel",
            "/World/Carters/Carter_0/right_wheel"
        ]
        
        for wheel_path in wheel_paths:
            wheel_prim = get_prim_at_path(wheel_path)
            if wheel_prim:
                # 设置轮子物理材质
                physx_material = PhysxSchema.PhysxMaterialAPI.Get(wheel_prim, "physics")
                if not physx_material:
                    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(wheel_prim, "physics")
                
                if physx_material:
                    # 增加摩擦系数（减少打滑）
                    if hasattr(physx_material, 'GetStaticFrictionAttr'):
                        physx_material.GetStaticFrictionAttr().Set(1.2)  # 静摩擦系数（增加）
                    if hasattr(physx_material, 'GetDynamicFrictionAttr'):
                        physx_material.GetDynamicFrictionAttr().Set(1.0)  # 动摩擦系数（增加）
                    if hasattr(physx_material, 'GetRestitutionAttr'):
                        physx_material.GetRestitutionAttr().Set(0.0)  # 无弹性
                    
                    rospy.loginfo(f"[物理配置] 已设置轮子摩擦力: {wheel_path}")
        
    except Exception as e:
        rospy.logwarn(f"[物理配置] 设置轮子摩擦力失败: {e}")
```

---

### 2. 改进地面摩擦力（高优先级）

**问题**：默认地面可能摩擦力不足

**修复**：创建自定义地面，设置合适的摩擦系数

```python
def __init__(self, ...):
    # ... 现有代码 ...
    
    # 修改地面创建
    if self.training_scene == "env":
        self._add_ground_plane_with_friction()  # 替换默认地面

def _add_ground_plane_with_friction(self):
    """添加具有合适摩擦力的地面"""
    try:
        from omni.isaac.core.prims import GeometryPrim
        from pxr import PhysxSchema
        
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
        ground_prim = ground_plane.prim
        physx_material = PhysxSchema.PhysxMaterialAPI.Get(ground_prim, "physics")
        if not physx_material:
            physx_material = PhysxSchema.PhysxMaterialAPI.Apply(ground_prim, "physics")
        
        if physx_material:
            # 增加地面摩擦系数
            if hasattr(physx_material, 'GetStaticFrictionAttr'):
                physx_material.GetStaticFrictionAttr().Set(1.0)  # 静摩擦系数
            if hasattr(physx_material, 'GetDynamicFrictionAttr'):
                physx_material.GetDynamicFrictionAttr().Set(0.9)  # 动摩擦系数（增加）
            if hasattr(physx_material, 'GetRestitutionAttr'):
                physx_material.GetRestitutionAttr().Set(0.0)  # 无弹性
            
            rospy.loginfo("[物理配置] 已设置地面摩擦力参数")
            
    except Exception as e:
        rospy.logwarn(f"[物理配置] 设置地面属性失败: {e}")
        # 如果失败，回退到默认地面
        self.world.scene.add_default_ground_plane()
```

---

### 3. 添加速度命令低通滤波（中优先级）

**问题**：速度命令的突然变化会导致侧滑

**修复**：使用低通滤波器平滑速度命令

```python
def __init__(self, ...):
    # ... 现有代码 ...
    
    # 新增：低通滤波器参数
    self.velocity_filter_alpha = 0.3  # 滤波系数（0-1，越小越平滑）

def _limit_and_smooth_velocity(self, linear_cmd, angular_cmd):
    """限制和平滑速度命令，防止急加速/急转弯导致侧翻"""
    # 限制速度命令
    linear_cmd = np.clip(linear_cmd, -self.max_linear_vel, self.max_linear_vel)
    angular_cmd = np.clip(angular_cmd, -self.max_angular_vel, self.max_angular_vel)
    
    # 新增：低通滤波（指数移动平均）
    self.current_linear_vel = (1 - self.velocity_filter_alpha) * self.current_linear_vel + \
                               self.velocity_filter_alpha * linear_cmd
    self.current_angular_vel = (1 - self.velocity_filter_alpha) * self.current_angular_vel + \
                                self.velocity_filter_alpha * angular_cmd
    
    # 计算加速度限制
    dt = self.world.get_physics_dt()
    max_linear_change = self.max_linear_accel * dt
    max_angular_change = self.max_angular_accel * dt
    
    # 平滑速度变化（二次限制）
    linear_diff = self.current_linear_vel - (self.current_linear_vel if hasattr(self, '_prev_linear_vel') else 0)
    angular_diff = self.current_angular_vel - (self.current_angular_vel if hasattr(self, '_prev_angular_vel') else 0)
    
    if abs(linear_diff) > max_linear_change:
        self.current_linear_vel = (self.current_linear_vel if hasattr(self, '_prev_linear_vel') else 0) + \
                                   np.sign(linear_diff) * max_linear_change
    
    if abs(angular_diff) > max_angular_change:
        self.current_angular_vel = (self.current_angular_vel if hasattr(self, '_prev_angular_vel') else 0) + \
                                   np.sign(angular_diff) * max_angular_change
    
    return self.current_linear_vel, self.current_angular_vel
```

---

### 4. 调整机器人质量分布（中优先级）

**问题**：重心过高或质量分布不均会导致不稳定

**修复**：调整底盘质量，降低重心

```python
def _configure_robot_physics(self):
    # ... 现有代码 ...
    
    # 新增：调整质量分布
    if chassis_prim:
        physx_rigid_body = PhysxSchema.PhysxRigidBodyAPI.Get(chassis_prim, "physics")
        if not physx_rigid_body:
            physx_rigid_body = PhysxSchema.PhysxRigidBodyAPI.Apply(chassis_prim, "physics")
        
        if physx_rigid_body:
            # 增加底盘质量（提高稳定性）
            if hasattr(physx_rigid_body, 'GetMassAttr'):
                current_mass = physx_rigid_body.GetMassAttr().Get()
                if current_mass < 20.0:  # 如果质量太小
                    physx_rigid_body.GetMassAttr().Set(20.0)  # 设置为20kg
                    rospy.loginfo(f"[物理配置] 已调整底盘质量: {current_mass} -> 20.0 kg")
            
            # 调整重心（降低重心）
            if hasattr(physx_rigid_body, 'GetCenterOfMassOffsetAttr'):
                # 将重心稍微下移（z轴负方向）
                com_offset = physx_rigid_body.GetCenterOfMassOffsetAttr().Get()
                if com_offset is None or len(com_offset) < 3:
                    from pxr import Gf
                    physx_rigid_body.GetCenterOfMassOffsetAttr().Set(Gf.Vec3f(0.0, 0.0, -0.05))  # 下移5cm
                    rospy.loginfo("[物理配置] 已调整重心位置（下移5cm）")
```

---

### 5. 添加侧向力限制（中优先级）

**问题**：急转弯时侧向力过大导致侧滑

**修复**：限制角速度与线速度的组合，防止侧向力过大

```python
def _limit_and_smooth_velocity(self, linear_cmd, angular_cmd):
    """限制和平滑速度命令，防止急加速/急转弯导致侧翻"""
    # ... 现有限制代码 ...
    
    # 新增：侧向力限制（防止急转弯时侧滑）
    # 侧向力与 v * ω 成正比，限制这个乘积
    max_lateral_force_factor = 0.8  # 最大侧向力因子
    lateral_force = abs(linear_cmd * angular_cmd)
    max_lateral_force = max_lateral_force_factor * self.max_linear_vel * self.max_angular_vel
    
    if lateral_force > max_lateral_force:
        # 按比例减小速度和角速度
        scale_factor = max_lateral_force / lateral_force
        linear_cmd *= scale_factor
        angular_cmd *= scale_factor
        rospy.logdebug_throttle(2.0, f"[速度限制] 侧向力过大，缩放因子: {scale_factor:.2f}")
    
    # ... 其余代码 ...
```

---

### 6. 改进控制器参数（低优先级）

**问题**：DifferentialController 的默认参数可能不适合

**修复**：检查并调整控制器参数（如果支持）

```python
def _add_robot(self):
    # ... 现有代码 ...
    
    # 修改：尝试设置控制器参数
    self.controller = DifferentialController(
        name="simple_control", 
        wheel_radius=0.24, 
        wheel_base=0.56
    )
    
    # 新增：如果控制器支持，设置最大速度限制
    if hasattr(self.controller, 'set_max_linear_velocity'):
        self.controller.set_max_linear_velocity(self.max_linear_vel)
    if hasattr(self.controller, 'set_max_angular_velocity'):
        self.controller.set_max_angular_velocity(self.max_angular_vel)
```

---

### 7. 添加速度前馈补偿（低优先级）

**问题**：速度命令与实际速度有延迟，导致控制不稳定

**修复**：添加前馈补偿，预测速度变化

```python
def _limit_and_smooth_velocity(self, linear_cmd, angular_cmd):
    """限制和平滑速度命令，防止急加速/急转弯导致侧翻"""
    # ... 现有代码 ...
    
    # 新增：速度前馈补偿（预测未来速度）
    # 基于当前加速度，预测下一帧的速度
    dt = self.world.get_physics_dt()
    predicted_linear = self.current_linear_vel + (linear_cmd - self.current_linear_vel) * 0.5
    predicted_angular = self.current_angular_vel + (angular_cmd - self.current_angular_vel) * 0.5
    
    # 使用预测速度进行限制检查
    if abs(predicted_linear) > self.max_linear_vel:
        linear_cmd = np.sign(predicted_linear) * self.max_linear_vel
    
    if abs(predicted_angular) > self.max_angular_vel:
        angular_cmd = np.sign(predicted_angular) * self.max_angular_vel
    
    # ... 其余代码 ...
```

---

### 8. 添加速度变化率限制（低优先级）

**问题**：即使有加速度限制，速度变化率仍然可能过大

**修复**：添加速度变化率（jerk）限制

```python
def __init__(self, ...):
    # ... 现有代码 ...
    
    # 新增：速度变化率限制
    self.max_linear_jerk = 1.0  # m/s³，最大线速度变化率
    self.max_angular_jerk = 2.0  # rad/s³，最大角速度变化率
    self.prev_linear_vel = 0.0
    self.prev_angular_vel = 0.0

def _limit_and_smooth_velocity(self, linear_cmd, angular_cmd):
    """限制和平滑速度命令，防止急加速/急转弯导致侧翻"""
    # ... 现有代码 ...
    
    # 新增：速度变化率限制（jerk限制）
    dt = self.world.get_physics_dt()
    
    # 计算速度变化率
    linear_jerk = (linear_cmd - self.current_linear_vel) / dt
    angular_jerk = (angular_cmd - self.current_angular_vel) / dt
    
    # 限制变化率
    if abs(linear_jerk) > self.max_linear_jerk:
        linear_cmd = self.current_linear_vel + np.sign(linear_jerk) * self.max_linear_jerk * dt
    
    if abs(angular_jerk) > self.max_angular_jerk:
        angular_cmd = self.current_angular_vel + np.sign(angular_jerk) * self.max_angular_jerk * dt
    
    # ... 其余代码 ...
```

---

## 推荐实施顺序

### 高优先级（立即实施）
1. **设置轮子摩擦力** - 直接减少打滑
2. **改进地面摩擦力** - 提高整体稳定性

### 中优先级（强烈推荐）
3. **添加速度命令低通滤波** - 进一步平滑速度
4. **调整机器人质量分布** - 提高稳定性
5. **添加侧向力限制** - 防止急转弯侧滑

### 低优先级（可选）
6. **改进控制器参数** - 如果控制器支持
7. **添加速度前馈补偿** - 提高控制精度
8. **添加速度变化率限制** - 进一步平滑

---

## 完整改进代码示例

见 `ENVIRONMENT_ANTI_SLIP_COMPLETE.py` 文件（如果创建）

