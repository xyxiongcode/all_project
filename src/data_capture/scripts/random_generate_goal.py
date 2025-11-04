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


class OptimizedRandomGoalGenerator:
    def __init__(self):
        rospy.init_node('optimized_random_goal_generator')
        self.make_plan = None
        self.map_ready = None
        self.first_goal_sent = False

        # ===== 参数 =====
        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')

        # ===== 新增：IMU 话题参数 =====
        self.imu_topic = rospy.get_param('~imu_topic', '/imu')

        # 距离参数
        self.min_distance = rospy.get_param('~min_distance', 0.5)  # m
        self.max_distance = rospy.get_param('~max_distance', 100.0)  # m

        # 障碍物距离偏好
        self.min_obstacle_distance = rospy.get_param('~min_obstacle_distance', 0.5)  # m
        self.max_obstacle_distance = rospy.get_param('~max_obstacle_distance', 1.0)  # m
        self.empty_space_discard_prob = rospy.get_param('~empty_space_discard_prob', 0.8)

        self.goal_period = rospy.get_param('~goal_period', 15.0)  # s
        self.goal_timeout = rospy.get_param('~goal_timeout', 15.0)  # s
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
        self.stuck_distance_threshold = rospy.get_param('~stuck_distance_threshold', 0.05)  # m，位移小于此视为未动
        self.stuck_time_threshold = rospy.get_param('~stuck_time_threshold', 45.0)  # s，持续未动的时间阈值
        self.rollover_angle_deg = rospy.get_param('~rollover_angle_deg', 30.0)  # 度，|roll|或|pitch|超此视为侧翻
        self.recovery_cooldown = rospy.get_param('~recovery_cooldown', 10.0)  # s，两次恢复间隔
        self.stuck_goal_grace = rospy.get_param('~stuck_goal_grace', 5.0)  # s，发目标后的宽限
        self.initial_pose_topic = rospy.get_param('~initial_pose_topic', '/initialpose')
        self.reset_action_name = rospy.get_param('~reset_action', '/reset')  # ResetPoses.action 的名字

        # ===== 订阅 / 发布 =====
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=50)

        # ===== 新增：IMU 订阅 =====
        self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=50)

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        self.initial_pose_pub = rospy.Publisher(self.initial_pose_topic, PoseWithCovarianceStamped, queue_size=1)

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

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 服务代理
        self.make_plan = None

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

        # 定时器
        self.timer = rospy.Timer(rospy.Duration(self.goal_period), self.generate_goal_timer_cb)
        # 新增：卡死周期检查（不会影响你的采样逻辑）
        self.stuck_timer = rospy.Timer(rospy.Duration(1.0), self.stuck_check_cb)

        rospy.loginfo("优化版随机目标生成器已启动")
        rospy.loginfo(f"目标距离范围: {self.min_distance}-{self.max_distance}m")
        rospy.loginfo(f"障碍物偏好: {self.min_obstacle_distance}-{self.max_obstacle_distance}m范围内")
        rospy.loginfo(f"空旷区域丢弃概率: {self.empty_space_discard_prob * 100}%")
        rospy.loginfo(
            f"卡死检测: 距离<{self.stuck_distance_threshold}m 且时间>{self.stuck_time_threshold}s；侧翻阈值≈{self.rollover_angle_deg}°")
        rospy.loginfo(f"使用IMU话题进行侧翻检测: {self.imu_topic}")

    # ===== 新增：IMU 回调函数 =====
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

    # ===== 原有的回调函数 =====
    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        self.map_data = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        self.map_ready = True
        rospy.loginfo_once(
            "收到地图：{}x{}，分辨率={:.3f} m".format(msg.info.width, msg.info.height, msg.info.resolution))

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

            # ===== 移除原有的姿态侧翻判断，改用IMU数据 =====

        self.last_pose = self.current_pose_map
        self.last_pose_time = now

        self.check_goal_reached_or_timeout()

    # ===== 定时器回调 =====
    def generate_goal_timer_cb(self, _):
        if self.current_pose_map is None or self.map_data is None:
            return

        need_new = (self.current_goal is None)
        if not need_new and (rospy.Time.now() - self.last_goal_time).to_sec() > self.goal_timeout:
            rospy.logwarn("目标超时，重新采样")
            need_new = True

        if need_new:
            self.sample_and_publish_goal()

    # ===== 修改：卡死周期检查 & 恢复入口 =====
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

    # ===== 其余函数保持不变 =====
    def check_goal_reached_or_timeout(self):
        if self.current_goal is None or self.current_pose_map is None:
            return
        cx = self.current_pose_map.position.x
        cy = self.current_pose_map.position.y
        gx, gy = self.current_goal
        dist = np.hypot(cx - gx, cy - gy)

        if dist < self.reach_tolerance:
            rospy.loginfo("到达目标点，立即采样新目标")
            self.current_goal = None
            self.sample_and_publish_goal()

    # ===== 核心：优化后的目标生成逻辑（保持不变）=====
    def generate_random_goal(self):
        # ... 保持不变 ...
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

    def is_within_map(self, x, y):
        if self.map_info is None:
            return False
        mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return (0 <= mx < self.map_info.width and 0 <= my < self.map_info.height)

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

    def sample_and_publish_goal(self):
        goal = self.generate_random_goal()
        if goal is None:
            rospy.logwarn("无法找到合适的随机目标")
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

        self.goal_pub.publish(goal_msg)
        self.current_goal = (goal_x, goal_y)
        self.last_goal_time = rospy.Time.now()
        self._last_goal_publish_ts = self.last_goal_time

        obstacle_distance = self.get_min_obstacle_distance(goal_x, goal_y)
        rospy.loginfo("发布新目标: ({:.2f}, {:.2f}), 障碍物距离: {:.2f}m".format(
            goal_x, goal_y, obstacle_distance))

    def perform_reset(self):
        # ... 保持不变 ...
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

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        OptimizedRandomGoalGenerator().run()
    except rospy.ROSInterruptException:
        pass

# import rospy
# import random
# import numpy as np
# from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
# from nav_msgs.msg import OccupancyGrid, Odometry
# from nav_msgs.srv import GetPlan
# from std_msgs.msg import Header
# import tf2_ros
# import tf2_geometry_msgs
#
#
# # === 可选：Isaac Sim 的 ResetPoses.action（如果没有这个包，会自动回退 /initialpose） ===
# try:
#     from isaac_sim.msg import ResetPosesAction, ResetPosesGoal
#     import actionlib
#     HAS_RESET_ACTION = True
# except Exception:
#     ResetPosesAction = ResetPosesGoal = None
#     actionlib = None
#     HAS_RESET_ACTION = False
#
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
#
#
# class OptimizedRandomGoalGenerator:
#     def __init__(self):
#         rospy.init_node('optimized_random_goal_generator')
#         self.make_plan = None
#         self.map_ready = None
#         self.first_goal_sent = False
#
#         # ===== 参数 =====
#         self.map_topic = rospy.get_param('~map_topic', '/map')
#         self.goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')
#         self.odom_topic = rospy.get_param('~odom_topic', '/odom')
#
#         # 距离参数
#         self.min_distance = rospy.get_param('~min_distance', 0.5)   # m
#         self.max_distance = rospy.get_param('~max_distance', 100.0) # m
#
#         # 障碍物距离偏好
#         self.min_obstacle_distance = rospy.get_param('~min_obstacle_distance', 0.5)  # m
#         self.max_obstacle_distance = rospy.get_param('~max_obstacle_distance', 1.0)  # m
#         self.empty_space_discard_prob = rospy.get_param('~empty_space_discard_prob', 0.8)
#
#         self.goal_period = rospy.get_param('~goal_period', 15.0)   # s
#         self.goal_timeout = rospy.get_param('~goal_timeout', 15.0) # s
#         self.reach_tolerance = rospy.get_param('~reach_tolerance', 0.5) # m
#
#         # 可达性检测
#         self.make_plan_srv_name = rospy.get_param('~make_plan_srv', '/move_base/make_plan')
#         self.plan_tolerance = rospy.get_param('~plan_tolerance', 0.2)   # m
#         self.min_plan_length = rospy.get_param('~min_plan_length', 0.5) # m
#
#         # TF 设置
#         self.use_tf = rospy.get_param('~use_tf', True)
#         self.map_frame = rospy.get_param('~map_frame', 'map')
#
#         # ===== 新增：卡死/侧翻检测 & 恢复相关参数 =====
#         self.enable_stuck_check = rospy.get_param('~enable_stuck_check', True)
#         self.stuck_distance_threshold = rospy.get_param('~stuck_distance_threshold', 0.05)   # m，位移小于此视为未动
#         self.stuck_time_threshold = rospy.get_param('~stuck_time_threshold', 45.0)           # s，持续未动的时间阈值
#         self.rollover_angle_deg = rospy.get_param('~rollover_angle_deg', 30.0)               # 度，|roll|或|pitch|超此视为侧翻
#         self.recovery_cooldown = rospy.get_param('~recovery_cooldown', 10.0)                 # s，两次恢复间隔
#         self.stuck_goal_grace = rospy.get_param('~stuck_goal_grace', 5.0)                    # s，发目标后的宽限
#         self.initial_pose_topic = rospy.get_param('~initial_pose_topic', '/initialpose')
#         self.reset_action_name = rospy.get_param('~reset_action', '/reset')                  # ResetPoses.action 的名字
#
#         # ===== 订阅 / 发布 =====
#         self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
#         self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=50)
#         self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
#         self.initial_pose_pub = rospy.Publisher(self.initial_pose_topic, PoseWithCovarianceStamped, queue_size=1)
#
#         # ===== 变量 =====
#         self.map_data = None
#         self.map_info = None
#         self.current_pose_map = None
#         self.current_goal = None
#         self.last_goal_time = rospy.Time(0)
#
#         # ===== 新增：卡死监测状态 =====
#         self.last_pose = None
#         self.last_pose_time = rospy.Time(0)
#         self.stuck_start_time = None
#         self.is_stuck = False
#         self._last_recovery_ts = rospy.Time(0)
#         self._last_goal_publish_ts = rospy.Time(0)
#
#         # TF
#         self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
#
#         # 服务代理
#         self.make_plan = None
#
#         # ===== 新增：ResetPoses Action 客户端（如果可用）=====
#         self.reset_client = None
#         if HAS_RESET_ACTION:
#             try:
#                 self.reset_client = actionlib.SimpleActionClient(self.reset_action_name, ResetPosesAction)
#                 if self.reset_client.wait_for_server(rospy.Duration(2.0)):
#                     rospy.loginfo(f"[reset] 已连接 ResetPoses action: {self.reset_action_name}")
#                 else:
#                     rospy.logwarn(f"[reset] 未连接到 {self.reset_action_name}，将回退 /initialpose")
#                     self.reset_client = None
#             except Exception as e:
#                 rospy.logwarn(f"[reset] 连接 {self.reset_action_name} 异常：{e}")
#                 self.reset_client = None
#         else:
#             rospy.logwarn("[reset] 未找到 isaac_sim/ResetPosesAction，复位将使用 /initialpose")
#
#         # 定时器
#         self.timer = rospy.Timer(rospy.Duration(self.goal_period), self.generate_goal_timer_cb)
#         # 新增：卡死周期检查（不会影响你的采样逻辑）
#         self.stuck_timer = rospy.Timer(rospy.Duration(1.0), self.stuck_check_cb)
#
#         rospy.loginfo("优化版随机目标生成器已启动")
#         rospy.loginfo(f"目标距离范围: {self.min_distance}-{self.max_distance}m")
#         rospy.loginfo(f"障碍物偏好: {self.min_obstacle_distance}-{self.max_obstacle_distance}m范围内")
#         rospy.loginfo(f"空旷区域丢弃概率: {self.empty_space_discard_prob * 100}%")
#         rospy.loginfo(f"卡死检测: 距离<{self.stuck_distance_threshold}m 且时间>{self.stuck_time_threshold}s；侧翻阈值≈{self.rollover_angle_deg}°")
#
#     # ===== 回调函数 =====
#     def map_callback(self, msg: OccupancyGrid):
#         self.map_info = msg.info
#         self.map_data = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
#         self.map_ready = True
#         rospy.loginfo_once("收到地图：{}x{}，分辨率={:.3f} m".format(msg.info.width, msg.info.height, msg.info.resolution))
#
#
#     def odom_callback(self, msg: Odometry):
#         pose_stamped = PoseStamped()
#         pose_stamped.header = msg.header
#         pose_stamped.pose = msg.pose.pose
#
#         if self.use_tf:
#             try:
#                 # 更稳：用最新变换（Time(0)），避免时间外推问题
#                 tfm = self.tf_buffer.lookup_transform(self.map_frame,
#                                                       pose_stamped.header.frame_id,
#                                                       rospy.Time(0),
#                                                       rospy.Duration(0.2))
#                 pose_map = tf2_geometry_msgs.do_transform_pose(pose_stamped, tfm).pose
#                 self.current_pose_map = pose_map
#             except Exception as e:
#                 rospy.logwarn_throttle(2.0, f"TF 变换失败：{e}")
#                 self.current_pose_map = msg.pose.pose
#         else:
#             self.current_pose_map = msg.pose.pose
#
#         # ===== 新增：位移与侧翻检测（不影响你的采样逻辑）=====
#         now = rospy.Time.now()
#         if self.enable_stuck_check:
#             if self.last_pose is not None:
#                 dx = self.current_pose_map.position.x - self.last_pose.position.x
#                 dy = self.current_pose_map.position.y - self.last_pose.position.y
#                 dist = np.hypot(dx, dy)
#                 # 距离阈值（未移动）
#                 if dist < self.stuck_distance_threshold:
#                     if self.stuck_start_time is None:
#                         self.stuck_start_time = now
#                     elif (now - self.stuck_start_time).to_sec() > self.stuck_time_threshold:
#                         self.is_stuck = True
#                 else:
#                     self.stuck_start_time = None
#                     self.is_stuck = False
#
#             # 姿态侧翻判断（roll/pitch 超阈值）
#             o = self.current_pose_map.orientation
#             try:
#                 roll, pitch, _ = euler_from_quaternion([o.x, o.y, o.z, o.w])
#                 deg = np.degrees
#                 if abs(deg(roll)) > self.rollover_angle_deg or abs(deg(pitch)) > self.rollover_angle_deg:
#                     self.is_stuck = True
#             except Exception:
#                 pass
#
#         self.last_pose = self.current_pose_map
#         self.last_pose_time = now
#
#         self.check_goal_reached_or_timeout()
#
#     # ===== 定时器回调 =====
#     def generate_goal_timer_cb(self, _):
#         if self.current_pose_map is None or self.map_data is None:
#             return
#
#         need_new = (self.current_goal is None)
#         if not need_new and (rospy.Time.now() - self.last_goal_time).to_sec() > self.goal_timeout:
#             rospy.logwarn("目标超时，重新采样")
#             need_new = True
#
#         # 注意：不阻塞你的采样逻辑；若开启卡死检测且确实卡死，会由 stuck_check_cb 触发恢复
#         if need_new:
#             self.sample_and_publish_goal()
#
#     # ===== 新增：卡死周期检查 & 恢复入口 =====
#     def stuck_check_cb(self, _):
#         if not self.enable_stuck_check:
#             return
#         now = rospy.Time.now()
#         # 复位冷却
#         if (now - self._last_recovery_ts).to_sec() < self.recovery_cooldown:
#             return
#         # 发目标后的宽限：给局部规划一点时间起步
#         if (now - self._last_goal_publish_ts).to_sec() < self.stuck_goal_grace:
#             return
#         # 里程计长时间不更新（通信/仿真暂停）
#         if (now - self.last_pose_time).to_sec() > max(2*self.stuck_time_threshold, 10.0):
#             self.is_stuck = True
#
#         if self.is_stuck:
#             rospy.logwarn("检测到侧翻/卡死，执行复位")
#             self.perform_reset()
#
#     # ===== 目标判定 =====
#     def check_goal_reached_or_timeout(self):
#         if self.current_goal is None or self.current_pose_map is None:
#             return
#         cx = self.current_pose_map.position.x
#         cy = self.current_pose_map.position.y
#         gx, gy = self.current_goal
#         dist = np.hypot(cx - gx, cy - gy)
#
#         if dist < self.reach_tolerance:
#             rospy.loginfo("到达目标点，立即采样新目标")
#             self.current_goal = None
#             self.sample_and_publish_goal()
#
#     # ===== 核心：优化后的目标生成逻辑（保持不变）=====
#     def generate_random_goal(self):
#         if self.current_pose_map is None or self.map_data is None or self.map_info is None:
#             return None
#
#         cx = self.current_pose_map.position.x
#         cy = self.current_pose_map.position.y
#
#         for attempt in range(300):
#             angle = random.uniform(0.0, 2.0 * np.pi)
#             dist = random.uniform(self.min_distance, self.max_distance)
#
#             gx = cx + dist * np.cos(angle)
#             gy = cy + dist * np.sin(angle)
#
#             if not self.is_within_map(gx, gy):
#                 continue
#
#             obstacle_distance = self.get_min_obstacle_distance(gx, gy)
#             if obstacle_distance is None:
#                 continue
#
#             if obstacle_distance < self.min_obstacle_distance:
#                 continue
#             elif obstacle_distance <= self.max_obstacle_distance:
#                 pass
#             else:
#                 if random.random() < self.empty_space_discard_prob:
#                     continue
#
#             if not self.is_reachable_from_current(gx, gy):
#                 continue
#
#             rospy.logdebug(f"找到目标点: ({gx:.2f}, {gy:.2f}), 障碍物距离: {obstacle_distance:.2f}m")
#             return gx, gy
#
#         rospy.logwarn("在300次尝试后未找到合适目标")
#         return None
#
#     def is_within_map(self, x, y):
#         if self.map_info is None:
#             return False
#         mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
#         my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
#         return (0 <= mx < self.map_info.width and 0 <= my < self.map_info.height)
#
#     def get_min_obstacle_distance(self, x, y):
#         if self.map_info is None or self.map_data is None:
#             return None
#         mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
#         my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
#         if mx < 0 or my < 0 or mx >= self.map_info.width or my >= self.map_info.height:
#             return None
#         search_radius_cells = int(2.0 / self.map_info.resolution)
#         min_distance = float('inf')
#         for dx in range(-search_radius_cells, search_radius_cells + 1):
#             for dy in range(-search_radius_cells, search_radius_cells + 1):
#                 check_x = mx + dx
#                 check_y = my + dy
#                 if (0 <= check_x < self.map_info.width and 0 <= check_y < self.map_info.height):
#                     if self.map_data[check_y, check_x] > 0:
#                         obstacle_x = self.map_info.origin.position.x + check_x * self.map_info.resolution
#                         obstacle_y = self.map_info.origin.position.y + check_y * self.map_info.resolution
#                         distance = np.hypot(x - obstacle_x, y - obstacle_y)
#                         if distance < min_distance:
#                             min_distance = distance
#         return min_distance if min_distance != float('inf') else 100.0
#
#     def is_reachable_from_current(self, gx, gy):
#         if self.make_plan is None:
#             try:
#                 rospy.wait_for_service(self.make_plan_srv_name, timeout=15.0)
#                 self.make_plan = rospy.ServiceProxy(self.make_plan_srv_name, GetPlan)
#             except:
#                 rospy.logwarn(f"无法连接服务: {self.make_plan_srv_name}")
#                 return False
#
#         start = PoseStamped()
#         start.header.stamp = rospy.Time.now()
#         start.header.frame_id = self.map_frame
#         start.pose = self.current_pose_map
#
#         goal = PoseStamped()
#         goal.header.stamp = rospy.Time.now()
#         goal.header.frame_id = self.map_frame
#         goal.pose.position.x = gx
#         goal.pose.position.y = gy
#         goal.pose.position.z = 0.0
#         goal.pose.orientation.w = 1.0
#
#         try:
#             resp = self.make_plan(start=start, goal=goal, tolerance=self.plan_tolerance)
#             if not resp.plan.poses:
#                 return False
#             length = 0.0
#             prev = None
#             for ps in resp.plan.poses:
#                 if prev is not None:
#                     length += np.hypot(ps.pose.position.x - prev.pose.position.x,
#                                        ps.pose.position.y - prev.pose.position.y)
#                 prev = ps
#             return length >= self.min_plan_length
#         except Exception as e:
#             rospy.logwarn_throttle(2.0, f"路径规划失败: {e}")
#             return False
#
#     # ===== 目标发布 =====
#     def sample_and_publish_goal(self):
#         goal = self.generate_random_goal()
#         if goal is None:
#             rospy.logwarn("无法找到合适的随机目标")
#             return
#
#         goal_x, goal_y = goal
#         yaw = np.arctan2(goal_y - self.current_pose_map.position.y,
#                          goal_x - self.current_pose_map.position.x)
#         qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
#
#         goal_msg = PoseStamped()
#         goal_msg.header = Header(stamp=rospy.Time.now(), frame_id=self.map_frame)
#         goal_msg.pose.position.x = goal_x
#         goal_msg.pose.position.y = goal_y
#         goal_msg.pose.position.z = 0.0
#         goal_msg.pose.orientation.x = qx
#         goal_msg.pose.orientation.y = qy
#         goal_msg.pose.orientation.z = qz
#         goal_msg.pose.orientation.w = qw
#
#         self.goal_pub.publish(goal_msg)
#         self.current_goal = (goal_x, goal_y)
#         self.last_goal_time = rospy.Time.now()
#         self._last_goal_publish_ts = self.last_goal_time
#
#         obstacle_distance = self.get_min_obstacle_distance(goal_x, goal_y)
#         rospy.loginfo("发布新目标: ({:.2f}, {:.2f}), 障碍物距离: {:.2f}m".format(
#             goal_x, goal_y, obstacle_distance))
#
#     # ===== 新增：执行复位（优先 /reset Action；否则 /initialpose）=====
#     def perform_reset(self):
#         # 选择当前位置 xy，重置姿态到水平，yaw 随机（或设 0）
#         px = self.current_pose_map.position.x if self.current_pose_map else 0.0
#         py = self.current_pose_map.position.y if self.current_pose_map else 0.0
#         yaw = random.uniform(0, 2*np.pi)
#
#         pose = PoseStamped()
#         pose.header.stamp = rospy.Time.now()
#         pose.header.frame_id = self.map_frame
#         pose.pose.position.x = px
#         pose.pose.position.y = py
#         pose.pose.position.z = 0.0
#         qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
#         pose.pose.orientation.x = qx
#         pose.pose.orientation.y = qy
#         pose.pose.orientation.z = qz
#         pose.pose.orientation.w = qw
#
#         did_reset = False
#
#         # 1) 优先：ResetPoses.action
#         if self.reset_client is not None:
#             try:
#                 goal = ResetPosesGoal()
#                 if hasattr(goal, 'target'):
#                     goal.target = pose
#                 elif hasattr(goal, 'pose'):
#                     goal.pose = pose
#                 elif hasattr(goal, 'poses'):
#                     goal.poses = [pose]
#                 else:
#                     # 兜底：尝试第一个字段
#                     slots = getattr(goal, '__slots__', [])
#                     assigned = False
#                     for s in slots:
#                         try:
#                             setattr(goal, s, pose)
#                             assigned = True
#                             break
#                         except Exception:
#                             pass
#                     if not assigned:
#                         rospy.logwarn("[reset] 无法识别 ResetPosesGoal 字段，跳过 action")
#
#                 if hasattr(goal, 'target') or hasattr(goal, 'pose') or hasattr(goal, 'poses') or 'assigned' in locals():
#                     self.reset_client.send_goal(goal)
#                     ok = self.reset_client.wait_for_result(rospy.Duration(5.0))
#                     if ok:
#                         rospy.loginfo("[reset] ResetPoses.action 已返回")
#                         did_reset = True
#                     else:
#                         self.reset_client.cancel_goal()
#                         rospy.logwarn("[reset] ResetPoses.action 超时")
#             except Exception as e:
#                 rospy.logwarn(f"[reset] ResetPoses.action 失败：{e}")
#
#         # 2) 回退：/initialpose（AMCL 重定位）
#         if not did_reset:
#             init = PoseWithCovarianceStamped()
#             init.header.stamp = rospy.Time.now()
#             init.header.frame_id = self.map_frame
#             init.pose.pose = pose.pose
#             cov = [0.0]*36
#             cov[0] = cov[7] = 0.2**2
#             cov[35] = (np.deg2rad(5.0))**2
#             init.pose.covariance = cov
#             self.initial_pose_pub.publish(init)
#             rospy.loginfo("[reset] 已发布 /initialpose 进行复位")
#
#         # 3) 清状态并继续
#         self.is_stuck = False
#         self.stuck_start_time = None
#         self._last_recovery_ts = rospy.Time.now()
#         self.current_goal = None
#         rospy.sleep(0.3)
#         self.sample_and_publish_goal()
#
#     def run(self):
#         rospy.spin()
#
#
# if __name__ == "__main__":
#     try:
#         OptimizedRandomGoalGenerator().run()
#     except rospy.ROSInterruptException:
#         pass

# import rospy
# import random
# import numpy as np
# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import OccupancyGrid, Odometry
# from nav_msgs.srv import GetPlan
# from std_msgs.msg import Header
# import tf2_ros
# import tf2_geometry_msgs
#
#
# class OptimizedRandomGoalGenerator:
#     def __init__(self):
#         rospy.init_node('optimized_random_goal_generator')
#
#         # ===== 参数 =====
#         self.map_topic = rospy.get_param('~map_topic', '/map')
#         self.goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')
#         self.odom_topic = rospy.get_param('~odom_topic', '/odom')
#
#         # 距离参数（根据新要求调整）
#         self.min_distance = rospy.get_param('~min_distance', 0.5)  # m（新要求：0.5m）
#         self.max_distance = rospy.get_param('~max_distance', 100.0)  # m（新要求：100m内）
#
#         # 障碍物距离参数（新要求）
#         self.min_obstacle_distance = rospy.get_param('~min_obstacle_distance', 0.5)  # m
#         self.max_obstacle_distance = rospy.get_param('~max_obstacle_distance', 1.0)  # m
#         self.empty_space_discard_prob = rospy.get_param('~empty_space_discard_prob', 0.8)  # 80%概率丢弃空旷区域
#
#         self.goal_period = rospy.get_param('~goal_period', 15.0)  # s
#         self.goal_timeout = rospy.get_param('~goal_timeout', 15.0)  # s
#         self.reach_tolerance = rospy.get_param('~reach_tolerance', 0.5)  # m
#
#         # 可达性检测
#         self.make_plan_srv_name = rospy.get_param('~make_plan_srv', '/move_base/make_plan')
#         self.plan_tolerance = rospy.get_param('~plan_tolerance', 0.2)  # m
#         self.min_plan_length = rospy.get_param('~min_plan_length', 0.5)  # m
#
#         # TF 设置
#         self.use_tf = rospy.get_param('~use_tf', True)
#         self.map_frame = rospy.get_param('~map_frame', 'map')
#
#         # ===== 订阅 / 发布 =====
#         self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
#         self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=50)
#         self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
#
#         # ===== 变量 =====
#         self.map_data = None
#         self.map_info = None
#         self.current_pose_map = None
#         self.current_goal = None
#         self.last_goal_time = rospy.Time(0)
#
#         # TF
#         self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
#
#         # 服务代理
#         self.make_plan = None
#
#         # 定时器
#         self.timer = rospy.Timer(rospy.Duration(self.goal_period), self.generate_goal_timer_cb)
#
#         rospy.loginfo("优化版随机目标生成器已启动")
#         rospy.loginfo(f"目标距离范围: {self.min_distance}-{self.max_distance}m")
#         rospy.loginfo(f"障碍物偏好: {self.min_obstacle_distance}-{self.max_obstacle_distance}m范围内")
#         rospy.loginfo(f"空旷区域丢弃概率: {self.empty_space_discard_prob * 100}%")
#
#     # ===== 回调函数 =====
#     def map_callback(self, msg: OccupancyGrid):
#         self.map_info = msg.info
#         self.map_data = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
#         rospy.loginfo_once(
#             "收到地图：{}x{}，分辨率={:.3f} m".format(msg.info.width, msg.info.height, msg.info.resolution))
#
#     def odom_callback(self, msg: Odometry):
#         pose_stamped = PoseStamped()
#         pose_stamped.header = msg.header
#         pose_stamped.pose = msg.pose.pose
#
#         if self.use_tf:
#             try:
#                 tfm = self.tf_buffer.lookup_transform(self.map_frame,
#                                                       pose_stamped.header.frame_id,
#                                                       pose_stamped.header.stamp,
#                                                       rospy.Duration(0.2))
#                 pose_map = tf2_geometry_msgs.do_transform_pose(pose_stamped, tfm).pose
#                 self.current_pose_map = pose_map
#             except Exception as e:
#                 rospy.logwarn_throttle(2.0, f"TF 变换失败：{e}")
#                 self.current_pose_map = msg.pose.pose
#         else:
#             self.current_pose_map = msg.pose.pose
#
#         self.check_goal_reached_or_timeout()
#
#     # ===== 定时器回调 =====
#     def generate_goal_timer_cb(self, _):
#         if self.current_pose_map is None or self.map_data is None:
#             return
#
#         need_new = (self.current_goal is None)
#         if not need_new and (rospy.Time.now() - self.last_goal_time).to_sec() > self.goal_timeout:
#             rospy.logwarn("目标超时，重新采样")
#             need_new = True
#
#         if need_new:
#             self.sample_and_publish_goal()
#
#     # ===== 目标判定 =====
#     def check_goal_reached_or_timeout(self):
#         if self.current_goal is None or self.current_pose_map is None:
#             return
#         cx = self.current_pose_map.position.x
#         cy = self.current_pose_map.position.y
#         gx, gy = self.current_goal
#         dist = np.hypot(cx - gx, cy - gy)
#
#         if dist < self.reach_tolerance:
#             rospy.loginfo("到达目标点，立即采样新目标")
#             self.current_goal = None
#             self.sample_and_publish_goal()
#
#     # ===== 核心：优化后的目标生成逻辑 =====
#     def generate_random_goal(self):
#         """优化版随机目标生成：偏好靠近障碍物的区域"""
#         if self.current_pose_map is None or self.map_data is None or self.map_info is None:
#             return None
#
#         cx = self.current_pose_map.position.x
#         cy = self.current_pose_map.position.y
#
#         for attempt in range(300):  # 增加尝试次数
#             # 随机角度和距离（新范围：0.5-100米）
#             angle = random.uniform(0.0, 2.0 * np.pi)
#             dist = random.uniform(self.min_distance, self.max_distance)
#
#             gx = cx + dist * np.cos(angle)
#             gy = cy + dist * np.sin(angle)
#
#             # 检查基本有效性（是否在地图内）
#             if not self.is_within_map(gx, gy):
#                 continue
#
#             # 检查障碍物距离（新逻辑）
#             obstacle_distance = self.get_min_obstacle_distance(gx, gy)
#
#             if obstacle_distance is None:
#                 continue  # 无效点
#
#             # 应用新的障碍物距离偏好策略
#             if obstacle_distance < self.min_obstacle_distance:
#                 continue  # 太靠近障碍物，丢弃
#
#             elif obstacle_distance <= self.max_obstacle_distance:
#                 # 理想区域：0.5-1米范围内，保留
#                 pass  # 直接进入可达性检查
#
#             else:
#                 # 空旷区域（>1米），按概率丢弃
#                 if random.random() < self.empty_space_discard_prob:
#                     continue  # 80%概率丢弃
#
#             # 检查可达性
#             if not self.is_reachable_from_current(gx, gy):
#                 continue
#
#             rospy.logdebug(f"找到目标点: ({gx:.2f}, {gy:.2f}), 障碍物距离: {obstacle_distance:.2f}m")
#             return gx, gy
#
#         rospy.logwarn("在300次尝试后未找到合适目标")
#         return None
#
#     def is_within_map(self, x, y):
#         """检查点是否在地图范围内"""
#         if self.map_info is None:
#             return False
#
#         mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
#         my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
#
#         return (0 <= mx < self.map_info.width and
#                 0 <= my < self.map_info.height)
#
#     def get_min_obstacle_distance(self, x, y):
#         """计算点到最近障碍物的距离（米）"""
#         if self.map_info is None or self.map_data is None:
#             return None
#
#         # 转换到地图坐标
#         mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
#         my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
#
#         if mx < 0 or my < 0 or mx >= self.map_info.width or my >= self.map_info.height:
#             return None
#
#         # 搜索半径（换算成栅格数）
#         search_radius_cells = int(2.0 / self.map_info.resolution)  # 搜索2米范围内的障碍物
#
#         min_distance = float('inf')
#
#         # 在搜索范围内查找最近障碍物
#         for dx in range(-search_radius_cells, search_radius_cells + 1):
#             for dy in range(-search_radius_cells, search_radius_cells + 1):
#                 check_x = mx + dx
#                 check_y = my + dy
#
#                 if (0 <= check_x < self.map_info.width and
#                         0 <= check_y < self.map_info.height):
#
#                     # 如果是障碍物（值>0）
#                     if self.map_data[check_y, check_x] > 0:
#                         # 计算实际距离（米）
#                         obstacle_x = self.map_info.origin.position.x + check_x * self.map_info.resolution
#                         obstacle_y = self.map_info.origin.position.y + check_y * self.map_info.resolution
#                         distance = np.hypot(x - obstacle_x, y - obstacle_y)
#
#                         if distance < min_distance:
#                             min_distance = distance
#
#         return min_distance if min_distance != float('inf') else 100.0  # 如果没有障碍物，返回一个大值
#
#     def is_reachable_from_current(self, gx, gy):
#         """检查目标点是否可达"""
#         if self.make_plan is None:
#             try:
#                 rospy.wait_for_service(self.make_plan_srv_name, timeout=5.0)
#                 self.make_plan = rospy.ServiceProxy(self.make_plan_srv_name, GetPlan)
#             except:
#                 rospy.logwarn(f"无法连接服务: {self.make_plan_srv_name}")
#                 return False
#
#         start = PoseStamped()
#         start.header.stamp = rospy.Time.now()
#         start.header.frame_id = self.map_frame
#         start.pose = self.current_pose_map
#
#         goal = PoseStamped()
#         goal.header.stamp = rospy.Time.now()
#         goal.header.frame_id = self.map_frame
#         goal.pose.position.x = gx
#         goal.pose.position.y = gy
#         goal.pose.position.z = 0.0
#         goal.pose.orientation.w = 1.0
#
#         try:
#             resp = self.make_plan(start=start, goal=goal, tolerance=self.plan_tolerance)
#             if not resp.plan.poses:
#                 return False
#
#             # 计算路径长度
#             length = 0.0
#             prev = None
#             for ps in resp.plan.poses:
#                 if prev is not None:
#                     length += np.hypot(ps.pose.position.x - prev.pose.position.x,
#                                        ps.pose.position.y - prev.pose.position.y)
#                 prev = ps
#
#             return length >= self.min_plan_length
#         except Exception as e:
#             rospy.logwarn_throttle(2.0, f"路径规划失败: {e}")
#             return False
#
#     # ===== 目标发布 =====
#     def sample_and_publish_goal(self):
#         """生成并发布目标点"""
#         goal = self.generate_random_goal()
#         if goal is None:
#             rospy.logwarn("无法找到合适的随机目标")
#             return
#
#         goal_x, goal_y = goal
#
#         # 计算朝向角（面向目标）
#         yaw = np.arctan2(goal_y - self.current_pose_map.position.y,
#                          goal_x - self.current_pose_map.position.x)
#
#         from tf.transformations import quaternion_from_euler
#         qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
#
#         goal_msg = PoseStamped()
#         goal_msg.header = Header(stamp=rospy.Time.now(), frame_id=self.map_frame)
#         goal_msg.pose.position.x = goal_x
#         goal_msg.pose.position.y = goal_y
#         goal_msg.pose.position.z = 0.0
#         goal_msg.pose.orientation.x = qx
#         goal_msg.pose.orientation.y = qy
#         goal_msg.pose.orientation.z = qz
#         goal_msg.pose.orientation.w = qw
#
#         self.goal_pub.publish(goal_msg)
#         self.current_goal = (goal_x, goal_y)
#         self.last_goal_time = rospy.Time.now()
#
#         # 记录障碍物距离信息
#         obstacle_distance = self.get_min_obstacle_distance(goal_x, goal_y)
#         rospy.loginfo("发布新目标: ({:.2f}, {:.2f}), 障碍物距离: {:.2f}m".format(
#             goal_x, goal_y, obstacle_distance))
#
#     def run(self):
#         rospy.spin()
#
#
# if __name__ == "__main__":
#     try:
#         OptimizedRandomGoalGenerator().run()
#     except rospy.ROSInterruptException:
#         pass
