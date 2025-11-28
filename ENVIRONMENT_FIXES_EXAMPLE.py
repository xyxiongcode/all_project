# environment.py 修复示例代码
# 这些代码片段可以添加到现有的 environment.py 文件中

# ============================================
# 修复1：减小物理时间步长（第94行）
# ============================================
# 原代码：
# self.world = World(stage_units_in_meters=1.0, physics_dt=0.05, rendering_dt=0.05)

# 修复后：
self.world = World(stage_units_in_meters=1.0, physics_dt=0.02, rendering_dt=0.05)
# physics_dt=0.02 表示50Hz物理更新，提高稳定性


# ============================================
# 修复2：在 __init__ 中添加速度限制参数
# ============================================
def __init__(self, training_scene="env"):
    # ... 原有代码 ...
    
    # 新增：速度限制参数
    self.max_linear_vel = 1.5  # m/s，最大线速度
    self.max_angular_vel = 1.0  # rad/s，最大角速度
    self.max_linear_accel = 0.5  # m/s²，最大线加速度
    self.max_angular_accel = 1.0  # rad/s²，最大角加速度
    
    # 当前速度（用于平滑）
    self.current_linear_vel = 0.0
    self.current_angular_vel = 0.0


# ============================================
# 修复3：添加速度限制和平滑方法
# ============================================
def _limit_and_smooth_velocity(self, linear_cmd, angular_cmd):
    """限制和平滑速度命令，防止急加速/急转弯"""
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


# ============================================
# 修复4：修改 cycle() 方法应用速度限制
# ============================================
def cycle(self):
    self.world.reset()
    self.carter_lidar.enable_visualization()
    self.carter_lidar.add_depth_data_to_frame()
    
    # 重置速度
    self.current_linear_vel = 0.0
    self.current_angular_vel = 0.0
    
    while simulation_app.is_running():
        if self.state == SimulationState.NORMAL:
            self.world.step()
            self._store_laser()
        elif self.state == SimulationState.PAUSE:
            self.world.pause()
        elif self.state == SimulationState.STEP:
            # 修改：应用速度限制和平滑
            linear_cmd, angular_cmd = self._limit_and_smooth_velocity(
                self.step_vel[0], self.step_vel[1]
            )
            self.robot.apply_wheel_actions(
                self.controller.forward(command=np.array([linear_cmd, angular_cmd]))
            )
            self.world.play()
            self._store_laser()
            with self.state_lock:
                self.state = SimulationState.PAUSE
                self.step_flag = True
        elif self.state == SimulationState.RESET:
            self.world.pause()
            self._reset_process()
            # 修改：应用零速度（已限制）
            self.robot.apply_wheel_actions(
                self.controller.forward(command=np.array([0.0, 0.0]))
            )
            # 重置速度
            self.current_linear_vel = 0.0
            self.current_angular_vel = 0.0
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


# ============================================
# 修复5：在 _add_robot() 后添加物理稳定性配置
# ============================================
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
    self.lidar_path = "/World/Carters/Carter_0/chassis_link/lidar"
    self.carter_lidar = self.world.scene.add(
        RotatingLidarPhysX(
            prim_path=self.lidar_path,
            name="lidar",
            rotation_frequency=0,
            translation=np.array([-0.06, 0, 0.25]),  # 修改：降低LiDAR高度（从0.38到0.25）
            fov=(270, 0.0), resolution=(0.25, 0.0),
            valid_range=(0.4, 10.0)
        )
    )
    self.lidarInterface = _range_sensor.acquire_lidar_sensor_interface()
    self.controller = DifferentialController(name="simple_control", wheel_radius=0.24, wheel_base=0.56)
    self.robots.append(self.robot)
    
    # 新增：配置物理稳定性
    self._configure_robot_physics()


def _configure_robot_physics(self):
    """配置机器人物理属性以提高稳定性"""
    try:
        from omni.isaac.core.utils.prims import get_prim_at_path
        from pxr import PhysxSchema
        
        # 设置底盘（chassis）的物理属性
        chassis_path = "/World/Carters/Carter_0/chassis_link"
        chassis_prim = get_prim_at_path(chassis_path)
        
        if chassis_prim:
            # 获取或创建 PhysxRigidBodyAPI
            physx_rigid_body = PhysxSchema.PhysxRigidBodyAPI.Get(chassis_prim, "physics")
            if not physx_rigid_body:
                physx_rigid_body = PhysxSchema.PhysxRigidBodyAPI.Apply(chassis_prim, "physics")
            
            if physx_rigid_body:
                # 增加线性阻尼和角阻尼
                if hasattr(physx_rigid_body, 'GetLinearDampingAttr'):
                    physx_rigid_body.GetLinearDampingAttr().Set(0.1)  # 线性阻尼
                if hasattr(physx_rigid_body, 'GetAngularDampingAttr'):
                    physx_rigid_body.GetAngularDampingAttr().Set(0.2)  # 角阻尼（防止旋转过快）
                
                # 设置稳定器（如果支持）
                if hasattr(physx_rigid_body, 'GetStabilizationEnabledAttr'):
                    physx_rigid_body.GetStabilizationEnabledAttr().Set(True)
                if hasattr(physx_rigid_body, 'GetStabilizationThresholdAttr'):
                    physx_rigid_body.GetStabilizationThresholdAttr().Set(0.01)
                
                rospy.loginfo("[物理配置] 已设置底盘物理稳定性参数")
        
        # 设置轮子的物理属性
        wheel_paths = [
            "/World/Carters/Carter_0/left_wheel",
            "/World/Carters/Carter_0/right_wheel"
        ]
        
        for wheel_path in wheel_paths:
            wheel_prim = get_prim_at_path(wheel_path)
            if wheel_prim:
                physx_rigid_body = PhysxSchema.PhysxRigidBodyAPI.Get(wheel_prim, "physics")
                if not physx_rigid_body:
                    physx_rigid_body = PhysxSchema.PhysxRigidBodyAPI.Apply(wheel_prim, "physics")
                
                if physx_rigid_body:
                    # 增加轮子阻尼
                    if hasattr(physx_rigid_body, 'GetLinearDampingAttr'):
                        physx_rigid_body.GetLinearDampingAttr().Set(0.05)
                    if hasattr(physx_rigid_body, 'GetAngularDampingAttr'):
                        physx_rigid_body.GetAngularDampingAttr().Set(0.1)
        
        rospy.loginfo("[物理配置] 已设置轮子物理稳定性参数")
        
    except Exception as e:
        rospy.logwarn(f"[物理配置] 设置物理属性时出错: {e}")


# ============================================
# 修复6：改进地面摩擦力设置（可选）
# ============================================
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
            if hasattr(physx_material, 'GetStaticFrictionAttr'):
                physx_material.GetStaticFrictionAttr().Set(1.0)  # 静摩擦系数
            if hasattr(physx_material, 'GetDynamicFrictionAttr'):
                physx_material.GetDynamicFrictionAttr().Set(0.8)  # 动摩擦系数
            if hasattr(physx_material, 'GetRestitutionAttr'):
                physx_material.GetRestitutionAttr().Set(0.0)  # 恢复系数（无弹性）
            
            rospy.loginfo("[地面配置] 已设置地面摩擦力参数")
            
    except Exception as e:
        rospy.logwarn(f"[地面配置] 设置地面属性时出错: {e}")


# 在 __init__ 中替换默认地面：
# 原代码：
# if self.training_scene == "env":
#     self.world.scene.add_default_ground_plane()

# 修改为：
if self.training_scene == "env":
    self._add_ground_plane_with_friction()
    # 或者保留默认地面，只修改物理属性


# ============================================
# 修复7：添加侧翻检测和自动恢复（可选）
# ============================================
def _check_rollover(self):
    """检查机器人是否侧翻"""
    try:
        # 获取机器人姿态
        pose = self.robot.get_world_pose()
        orientation = pose[1]  # 四元数 [w, x, y, z]
        
        # 转换为欧拉角
        from tf.transformations import euler_from_quaternion
        roll, pitch, _ = euler_from_quaternion([
            orientation[1], orientation[2], orientation[3], orientation[0]
        ])
        
        # 检查 roll 或 pitch 是否过大（>30度）
        roll_deg = abs(np.degrees(roll))
        pitch_deg = abs(np.degrees(pitch))
        
        return roll_deg > 30.0 or pitch_deg > 30.0
    except Exception as e:
        rospy.logwarn_throttle(5.0, f"[侧翻检测] 检查侧翻时出错: {e}")
        return False


def _recover_from_rollover(self):
    """从侧翻中恢复"""
    rospy.logwarn("[侧翻恢复] 检测到机器人侧翻，执行恢复")
    
    # 停止机器人
    self.robot.apply_wheel_actions(
        self.controller.forward(command=np.array([0.0, 0.0]))
    )
    
    # 重置位姿到初始位置
    position = np.array([self.init_pose[0], self.init_pose[1], 0.3])
    orientation = np.array([
        np.cos(self.init_pose[2] / 2), 0.0, 0.0, np.sin(self.init_pose[2] / 2)
    ])
    self.robot.set_world_pose(position=position, orientation=orientation)
    
    # 重置速度
    self.current_linear_vel = 0.0
    self.current_angular_vel = 0.0
    
    rospy.loginfo("[侧翻恢复] 机器人已恢复到初始位置")


# 在 cycle() 的 NORMAL 状态中添加侧翻检测：
# if self.state == SimulationState.NORMAL:
#     self.world.step()
#     self._store_laser()
#     
#     # 新增：检查侧翻
#     if self._check_rollover():
#         self._recover_from_rollover()

