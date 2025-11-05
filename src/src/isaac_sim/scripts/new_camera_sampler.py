import rospy
import json
import csv
import os
import cv2
import numpy as np
from collections import deque
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image, CameraInfo, LaserScan, Imu
from nav_msgs.msg import Path, Odometry
import message_filters as mf
from cv_bridge import CvBridge
import math
import traceback
from tf.transformations import euler_from_quaternion
import hashlib
import random as pyrand


class EnhancedNavigationDataCollector:
    def __init__(self):
        rospy.init_node('navigation_data_collector')

        # ===== 保存目录 =====
        self.save_dir = rospy.get_param('~save_dir', "/media/gr-agv-x9xy/backup_xxy/camera_data")

        # —— 新增：目标点图像获取策略 —— #
        self.goal_image_strategy = rospy.get_param('~goal_image_strategy',
                                                   'current_rgb')  # 'current_rgb', 'static', 'none'
        self.static_goal_image_path = rospy.get_param('~static_goal_image_path', '')  # 静态目标点图像路径
        self.goal_image_topic = rospy.get_param('~goal_image_topic', '/goal_image')  # 目标点图像话题

        # —— 数据集切分参数 —— #
        self.enable_split = rospy.get_param('~enable_split', True)
        self.train_ratio = rospy.get_param('~train_ratio', 0.8)
        self.split_strategy = rospy.get_param('~split_strategy', 'episode')
        self.split_seed = int(rospy.get_param('~split_seed', 1234))

        # 创建目录
        self.subdirs = ["rgb", "depth", "depth_vis", "trajectory", "trajectory_vis",
                        "lidar", "imu", "actions", "goal_images", "goal_data"]
        if self.enable_split:
            for split in ["train", "test"]:
                for d in self.subdirs:
                    os.makedirs(os.path.join(self.save_dir, split, d), exist_ok=True)
        else:
            for d in self.subdirs:
                os.makedirs(os.path.join(self.save_dir, d), exist_ok=True)

        # ===== 采样控制 =====
        self.counter = 0
        self.max_count = rospy.get_param('~max_count', 50000)
        self.last_save_time = 0.0
        self.save_interval = rospy.get_param('~save_interval', 0.5)
        self.slop = rospy.get_param('~sync_slop', 0.05)

        # ===== 数据缓冲区 =====
        self.trajectory_history = deque(maxlen=rospy.get_param('~traj_history_len', 1000))
        self.trajectory_length = rospy.get_param('~traj_dump_len', 50)
        self.action_history = deque(maxlen=100)
        self.latest_imu = None
        self.latest_lidar = None
        self.latest_odom = None
        self.last_cmd_vel = None
        self._rng = np.random.RandomState(self.split_seed)

        # ===== 增强的Action记录模块 =====
        self.action_window_size = rospy.get_param('~action_window_size', 10)  # 动作窗口大小
        self.action_interpolation = rospy.get_param('~action_interpolation', True)  # 是否进行动作插值
        self.action_buffer = deque(maxlen=50)  # 动作缓冲区，用于存储原始动作数据
        self.last_action_time = None

        # ===== 目标点相关变量 =====
        self.current_goal_image = None
        self.current_goal_pose = None
        self.goal_image_stamp = None
        self.static_goal_image = None  # 静态目标点图像
        self.goal_history = []  # 历史目标点记录

        # 加载静态目标点图像（如果策略为static）
        if self.goal_image_strategy == 'static' and self.static_goal_image_path:
            try:
                self.static_goal_image = cv2.imread(self.static_goal_image_path)
                if self.static_goal_image is not None:
                    rospy.loginfo(f"成功加载静态目标点图像: {self.static_goal_image_path}")
                else:
                    rospy.logwarn(f"无法加载静态目标点图像: {self.static_goal_image_path}")
            except Exception as e:
                rospy.logwarn(f"加载静态目标点图像失败: {e}")

        # ===== 订阅（同步） =====
        rgb_sub = mf.Subscriber('/rgb_left', Image)
        depth_sub = mf.Subscriber('/depth_left', Image)
        odom_sub = mf.Subscriber('/odom', Odometry)
        lidar_sub = mf.Subscriber('/scan', LaserScan)
        imu_sub = mf.Subscriber('/imu', Imu)

        self.ts = mf.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, odom_sub, lidar_sub, imu_sub],
            queue_size=20,
            slop=self.slop
        )
        self.ts.registerCallback(self.sync_callback)

        # 非同步 - 增强的Action订阅
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=50)

        # 规划多源
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self._goal_callback, queue_size=10)
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.trajectory_callback, queue_size=10)
        rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.trajectory_callback, queue_size=10)
        rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan', Path, self.trajectory_callback, queue_size=10)
        rospy.Subscriber('/camera_info_left', CameraInfo, self._camera_info_callback, queue_size=1)

        # ===== 目标点图像订阅（根据策略决定是否订阅） =====
        if self.goal_image_strategy == 'topic':
            rospy.Subscriber(self.goal_image_topic, Image, self._goal_image_callback, queue_size=10)
            rospy.loginfo(f"订阅目标点图像话题: {self.goal_image_topic}")
        else:
            rospy.loginfo(f"目标点图像策略: {self.goal_image_strategy}")

        self.plan_source = None
        self.bridge = CvBridge()
        self.current_goal_msg = None
        self.current_episode_id = None
        self.planned_trajectory = None
        self.camera_info = None

        # ===== CSV 文件 =====
        self.csv_header = [
            # 索引
            'episode_id', 'stamp', 'odom_stamp', 'goal_stamp', 'split',
            # 路径
            'rgb_path', 'depth_path', 'depth_vis_path',
            'trajectory_path', 'trajectory_vis_path',
            'lidar_path', 'imu_path', 'action_path',
            'goal_image_path', 'goal_data_path',
            # 相机内参
            'fx', 'fy', 'cx', 'cy',
            # 机器人状态
            'robot_x', 'robot_y', 'robot_theta',
            'velocity_x', 'velocity_y', 'velocity_theta',
            # IMU
            'imu_linear_accel_x', 'imu_linear_accel_y', 'imu_linear_accel_z',
            'imu_angular_vel_x', 'imu_angular_vel_y', 'imu_angular_vel_z',
            'imu_orientation_x', 'imu_orientation_y', 'imu_orientation_z', 'imu_orientation_w',
            # LiDAR 元数据
            'lidar_range_min', 'lidar_range_max', 'lidar_angle_min', 'lidar_angle_max',
            'lidar_angle_increment', 'lidar_num_ranges',
            # 动作（cmd_vel）
            'cmd_vel_linear_x', 'cmd_vel_linear_y', 'cmd_vel_angular_z',
            # 增强的Action特征
            'action_mean_linear_x', 'action_mean_linear_y', 'action_mean_angular_z',
            'action_std_linear_x', 'action_std_linear_y', 'action_std_angular_z',
            'action_max_linear_x', 'action_max_linear_y', 'action_max_angular_z',
            'action_min_linear_x', 'action_min_linear_y', 'action_min_angular_z',
            'action_window_size', 'action_frequency',
            # 目标与规划
            'goal_x', 'goal_y', 'goal_theta',
            'goal_image_stamp', 'goal_image_strategy',
            'trajectory_length', 'has_planned_trajectory', 'plan_length', 'plan_source',
            # 质量与语义
            'invalid_bad_sync', 'invalid_low_depth', 'invalid_low_lidar', 'is_duplicate_motion',
            'has_goal', 'distance_to_goal', 'heading_error', 'near_obstacle', 'just_reached', 'min_lidar'
        ]

        if self.enable_split:
            self.csv_train_path = os.path.join(self.save_dir, "train", "enhanced_navigation_data.csv")
            self.csv_test_path = os.path.join(self.save_dir, "test", "enhanced_navigation_data.csv")
            self.csv_train = open(self.csv_train_path, "w", newline="")
            self.csv_test = open(self.csv_test_path, "w", newline="")
            self.csv_writer_train = csv.writer(self.csv_train)
            self.csv_writer_test = csv.writer(self.csv_test)
            self.csv_writer_train.writerow(self.csv_header)
            self.csv_writer_test.writerow(self.csv_header)
        else:
            self.csv_path = os.path.join(self.save_dir, "enhanced_navigation_data.csv")
            self.csv_file = open(self.csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(self.csv_header)

        rospy.loginfo("导航数据收集器已启动（目标点图像策略: %s）", self.goal_image_strategy)

    # ===== 回调函数 =====
    def _camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def _goal_callback(self, msg: PoseStamped):
        """目标点坐标回调"""
        self.current_goal_msg = msg
        self.current_episode_id = msg.header.stamp.to_nsec()

        # 保存目标点坐标到历史记录
        goal_pose = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'theta': self.get_yaw_from_quaternion(msg.pose.orientation),
            'stamp': msg.header.stamp.to_nsec(),
            'episode_id': self.current_episode_id
        }

        self.current_goal_pose = goal_pose
        self.goal_history.append(goal_pose)

        # 限制历史记录长度
        if len(self.goal_history) > 100:
            self.goal_history.pop(0)

        rospy.loginfo(
            f"收到新目标点: ({goal_pose['x']:.2f}, {goal_pose['y']:.2f}), 方向: {math.degrees(goal_pose['theta']):.1f}°")

    def _goal_image_callback(self, msg: Image):
        """目标点图像回调（仅当策略为topic时使用）"""
        if self.goal_image_strategy != 'topic':
            return

        try:
            goal_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_goal_image = goal_image
            self.goal_image_stamp = msg.header.stamp.to_nsec()
            rospy.loginfo("收到目标点图像，时间戳: %d", self.goal_image_stamp)
        except Exception as e:
            rospy.logwarn(f"目标点图像处理错误: {e}")

    def trajectory_callback(self, msg: Path):
        try:
            if msg.poses:
                self.planned_trajectory = [{
                    'x': p.pose.position.x,
                    'y': p.pose.position.y,
                    'z': p.pose.position.z
                } for p in msg.poses]
                self.plan_source = getattr(msg, "_connection_header", {}).get("topic", "unknown")
        except Exception as e:
            rospy.logwarn(f"轨迹处理错误: {e}")

    def cmd_vel_callback(self, msg: Twist):
        """增强的Action记录回调"""
        now_ns = rospy.Time.now().to_nsec()

        # 记录原始动作数据
        action_data = {
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'angular_z': msg.angular.z,
            'timestamp': now_ns
        }

        # 更新缓冲区
        self.action_buffer.append(action_data)
        self.last_cmd_vel = (now_ns, msg)

        # 计算动作频率
        if self.last_action_time is not None:
            time_diff = (now_ns - self.last_action_time) / 1e9  # 转换为秒
            if time_diff > 0:
                current_freq = 1.0 / time_diff
                # 可以在这里记录频率统计信息
        self.last_action_time = now_ns

    def _get_enhanced_action_features(self, current_timestamp):
        """获取增强的Action特征"""
        if not self.action_buffer:
            return self._get_default_action_features()

        # 获取最近的动作窗口
        window_actions = []
        for action in reversed(self.action_buffer):
            if current_timestamp - action['timestamp'] <= self.action_window_size * 1e9:  # 转换为纳秒
                window_actions.append(action)
            else:
                break

        if not window_actions:
            return self._get_default_action_features()

        # 反转以保持时间顺序
        window_actions.reverse()

        # 提取动作值
        linear_x = [a['linear_x'] for a in window_actions]
        linear_y = [a['linear_y'] for a in window_actions]
        angular_z = [a['angular_z'] for a in window_actions]

        # 计算统计特征
        features = {
            'current_linear_x': linear_x[-1] if linear_x else 0.0,
            'current_linear_y': linear_y[-1] if linear_y else 0.0,
            'current_angular_z': angular_z[-1] if angular_z else 0.0,
            'mean_linear_x': np.mean(linear_x) if linear_x else 0.0,
            'mean_linear_y': np.mean(linear_y) if linear_y else 0.0,
            'mean_angular_z': np.mean(angular_z) if angular_z else 0.0,
            'std_linear_x': np.std(linear_x) if linear_x else 0.0,
            'std_linear_y': np.std(linear_y) if linear_y else 0.0,
            'std_angular_z': np.std(angular_z) if angular_z else 0.0,
            'max_linear_x': np.max(linear_x) if linear_x else 0.0,
            'max_linear_y': np.max(linear_y) if linear_y else 0.0,
            'max_angular_z': np.max(angular_z) if angular_z else 0.0,
            'min_linear_x': np.min(linear_x) if linear_x else 0.0,
            'min_linear_y': np.min(linear_y) if linear_y else 0.0,
            'min_angular_z': np.min(angular_z) if angular_z else 0.0,
            'window_size': len(window_actions),
            'frequency': self._calculate_action_frequency()
        }

        return features

    def _get_default_action_features(self):
        """获取默认的Action特征"""
        return {
            'current_linear_x': 0.0,
            'current_linear_y': 0.0,
            'current_angular_z': 0.0,
            'mean_linear_x': 0.0,
            'mean_linear_y': 0.0,
            'mean_angular_z': 0.0,
            'std_linear_x': 0.0,
            'std_linear_y': 0.0,
            'std_angular_z': 0.0,
            'max_linear_x': 0.0,
            'max_linear_y': 0.0,
            'max_angular_z': 0.0,
            'min_linear_x': 0.0,
            'min_linear_y': 0.0,
            'min_angular_z': 0.0,
            'window_size': 0,
            'frequency': 0.0
        }

    def _calculate_action_frequency(self):
        """计算动作频率"""
        if len(self.action_buffer) < 2:
            return 0.0

        timestamps = [a['timestamp'] for a in list(self.action_buffer)[-10:]]  # 最近10个动作
        if len(timestamps) < 2:
            return 0.0

        time_diffs = [(timestamps[i] - timestamps[i - 1]) / 1e9 for i in range(1, len(timestamps))]
        avg_time_diff = np.mean(time_diffs) if time_diffs else 0.0

        return 1.0 / avg_time_diff if avg_time_diff > 0 else 0.0

    def _get_goal_image(self, rgb_image):
        """根据策略获取目标点图像"""
        if self.goal_image_strategy == 'topic':
            return self.current_goal_image
        elif self.goal_image_strategy == 'current_rgb':
            # 使用当前RGB图像作为目标点图像（适用于目标点就是当前位置的情况）
            return rgb_image.copy()
        elif self.goal_image_strategy == 'static':
            return self.static_goal_image
        else:  # 'none'
            return None

    # ===== 核心同步回调 =====
    def sync_callback(self, rgb_msg: Image, depth_msg: Image, odom_msg: Odometry,
                      lidar_msg: LaserScan, imu_msg: Imu):
        try:
            # 节流
            now = rospy.Time.now().to_sec()
            if now - self.last_save_time < self.save_interval:
                return
            if self.counter >= self.max_count:
                rospy.loginfo("达到最大保存数量，停止采集")
                return

            # 位姿
            robot_pose = {
                'x': odom_msg.pose.pose.position.x,
                'y': odom_msg.pose.pose.position.y,
                'theta': self.get_yaw_from_quaternion(odom_msg.pose.pose.orientation),
                'timestamp': odom_msg.header.stamp.to_nsec()
            }

            # 质量判定
            rgb_t = rgb_msg.header.stamp.to_sec()
            depth_t = depth_msg.header.stamp.to_sec()
            sync_ok = (abs(rgb_t - depth_t) <= 0.03)

            depth_np = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            depth_finite = np.isfinite(depth_np)
            valid_depth_ratio = float(depth_finite.sum()) / max(1, depth_finite.size)

            ranges = np.asarray(lidar_msg.ranges, dtype=np.float32)
            finite_mask = np.isfinite(ranges) & (ranges > lidar_msg.range_min) & (ranges < lidar_msg.range_max)
            valid_lidar_ratio = float(finite_mask.sum()) / max(1, ranges.size)
            min_lidar = float(np.nanmin(ranges[finite_mask])) if finite_mask.any() else float('inf')
            near_obstacle = (min_lidar < 0.5)

            if self.trajectory_history and len(self.trajectory_history) >= 1:
                last_pose = self.trajectory_history[-1]
                dpos = math.hypot(robot_pose['x'] - last_pose['x'], robot_pose['y'] - last_pose['y'])
                dyaw = abs(robot_pose['theta'] - last_pose['theta'])
            else:
                dpos, dyaw = 1e9, 1e9
            is_duplicate_motion = (dpos < 0.01 and dyaw < math.radians(1.0))

            # 目标语义
            has_goal = self.current_goal_msg is not None
            distance_to_goal, heading_error, just_reached = 0.0, 0.0, False
            if has_goal:
                gx = self.current_goal_msg.pose.position.x
                gy = self.current_goal_msg.pose.position.y
                rx, ry, yaw = robot_pose['x'], robot_pose['y'], robot_pose['theta']
                vec_g = math.atan2(gy - ry, gx - rx)
                distance_to_goal = math.hypot(gx - rx, gy - ry)
                heading_error = (vec_g - yaw + math.pi) % (2 * math.pi) - math.pi
                just_reached = (distance_to_goal < 0.05 and
                                abs(odom_msg.twist.twist.linear.x) < 0.02 and
                                abs(odom_msg.twist.twist.angular.z) < 0.02)

            quality_flags = {
                "bad_sync": (not sync_ok),
                "low_depth": (valid_depth_ratio < 0.6),
                "low_lidar": (valid_lidar_ratio < 0.6),
                "is_duplicate_motion": is_duplicate_motion
            }
            nav_semantics = {
                "has_goal": has_goal,
                "distance_to_goal": float(distance_to_goal),
                "heading_error": float(heading_error),
                "near_obstacle": bool(near_obstacle),
                "just_reached": bool(just_reached),
                "min_lidar": float(min_lidar)
            }

            # 硬过滤
            if quality_flags["bad_sync"] or (quality_flags["low_depth"] and quality_flags["low_lidar"]):
                return

            # 轨迹历史
            self.trajectory_history.append(robot_pose)

            # 图像
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = depth_np

            # 最近轨迹
            recent_traj = list(self.trajectory_history)[-self.trajectory_length:]

            # 目标信息
            goal_pose = None
            goal_stamp_ns = 0
            if self.current_goal_msg is not None:
                goal_pose = {
                    'x': self.current_goal_msg.pose.position.x,
                    'y': self.current_goal_msg.pose.position.y,
                    'theta': self.get_yaw_from_quaternion(self.current_goal_msg.pose.orientation)
                }
                goal_stamp_ns = self.current_goal_msg.header.stamp.to_nsec()

            # 获取目标点图像
            goal_image = self._get_goal_image(rgb_image)
            goal_image_stamp = self.goal_image_stamp if self.goal_image_stamp else goal_stamp_ns

            # 获取增强的Action特征
            action_features = self._get_enhanced_action_features(rgb_msg.header.stamp.to_nsec())

            # 决定 train/test
            split = self.decide_split(self.current_episode_id)

            # 保存
            stamp_ns = rgb_msg.header.stamp.to_nsec()
            self._save_enhanced_data(
                stamp_ns=stamp_ns,
                odom_stamp_ns=odom_msg.header.stamp.to_nsec(),
                goal_stamp_ns=goal_stamp_ns,
                episode_id=self.current_episode_id,
                split=split,
                rgb_image=rgb_image,
                depth_image=depth_image,
                goal_image=goal_image,
                goal_image_stamp=goal_image_stamp,
                robot_pose=robot_pose,
                goal_pose=goal_pose,
                recent_trajectory=recent_traj,
                odom_msg=odom_msg,
                lidar_msg=lidar_msg,
                imu_msg=imu_msg,
                action_features=action_features,  # 新增参数
                quality_flags=quality_flags,
                nav_semantics=nav_semantics
            )

            self.last_save_time = now
            self.counter += 1
            if self.counter % 100 == 0:
                rospy.loginfo(f"已保存 {self.counter} 个数据样本")

        except Exception as e:
            rospy.logerr(f"同步回调错误: {e}")
            traceback.print_exc()

    # ===== 切分策略 =====
    def decide_split(self, episode_id):
        if not self.enable_split:
            return ""
        if self.split_strategy == 'episode':
            key = str(episode_id if episode_id is not None else 0).encode('utf-8')
            h = hashlib.md5(key).hexdigest()
            frac = (int(h[-12:], 16) % 1000000) / 1000000.0
            return "train" if frac < self.train_ratio else "test"
        else:
            return "train" if self._rng.rand() < self.train_ratio else "test"

    # ===== 增强的数据保存 =====
    def _save_enhanced_data(self, stamp_ns, odom_stamp_ns, goal_stamp_ns, episode_id, split,
                            rgb_image, depth_image, goal_image, goal_image_stamp,
                            robot_pose, goal_pose, recent_trajectory,
                            odom_msg, lidar_msg, imu_msg, action_features=None,
                            quality_flags=None, nav_semantics=None):
        try:
            # 根目录
            root = os.path.join(self.save_dir, split) if (self.enable_split and split) else self.save_dir

            # 文件名
            rgb_filename = f"rgb_{stamp_ns}.png"
            depth_filename = f"depth_{stamp_ns}.npy"
            depth_vis_filename = f"depth_vis_{stamp_ns}.png"
            traj_filename = f"trajectory_{stamp_ns}.json"
            traj_vis_filename = f"trajectory_vis_{stamp_ns}.png"
            lidar_filename = f"lidar_{stamp_ns}.npy"
            imu_filename = f"imu_{stamp_ns}.json"
            action_filename = f"action_{stamp_ns}.json"
            goal_image_filename = f"goal_image_{stamp_ns}.png"
            goal_data_filename = f"goal_data_{stamp_ns}.json"

            # 路径
            base_paths = {
                'rgb': os.path.join(root, "rgb", rgb_filename),
                'depth': os.path.join(root, "depth", depth_filename),
                'depth_vis': os.path.join(root, "depth_vis", depth_vis_filename),
                'trajectory': os.path.join(root, "trajectory", traj_filename),
                'trajectory_vis': os.path.join(root, "trajectory_vis", traj_vis_filename),
                'lidar': os.path.join(root, "lidar", lidar_filename),
                'imu': os.path.join(root, "imu", imu_filename),
                'action': os.path.join(root, "actions", action_filename),
                'goal_image': os.path.join(root, "goal_images", goal_image_filename),
                'goal_data': os.path.join(root, "goal_data", goal_data_filename)
            }

            # 保存基础数据
            cv2.imwrite(base_paths['rgb'], rgb_image)
            np.save(base_paths['depth'], depth_image)
            self._save_depth_visualization(depth_image, base_paths['depth_vis'])

            # LiDAR
            lidar_pack = {
                'ranges': np.asarray(lidar_msg.ranges, dtype=np.float32),
                'angle_min': lidar_msg.angle_min,
                'angle_max': lidar_msg.angle_max,
                'angle_increment': lidar_msg.angle_increment,
                'range_min': lidar_msg.range_min,
                'range_max': lidar_msg.range_max,
                'stamp': lidar_msg.header.stamp.to_nsec()
            }
            np.save(base_paths['lidar'], lidar_pack, allow_pickle=True)

            # IMU
            imu_data = {
                'linear_acceleration': {
                    'x': imu_msg.linear_acceleration.x,
                    'y': imu_msg.linear_acceleration.y,
                    'z': imu_msg.linear_acceleration.z
                },
                'angular_velocity': {
                    'x': imu_msg.angular_velocity.x,
                    'y': imu_msg.angular_velocity.y,
                    'z': imu_msg.angular_velocity.z
                },
                'orientation': {
                    'x': imu_msg.orientation.x,
                    'y': imu_msg.orientation.y,
                    'z': imu_msg.orientation.z,
                    'w': imu_msg.orientation.w
                },
                'timestamp': stamp_ns
            }
            with open(base_paths['imu'], 'w') as f:
                json.dump(imu_data, f, indent=2)

            # 增强的Action数据保存
            action_data = {
                'current_cmd_vel': {
                    'linear_x': action_features['current_linear_x'],
                    'linear_y': action_features['current_linear_y'],
                    'angular_z': action_features['current_angular_z']
                },
                'action_statistics': {
                    'mean_linear_x': action_features['mean_linear_x'],
                    'mean_linear_y': action_features['mean_linear_y'],
                    'mean_angular_z': action_features['mean_angular_z'],
                    'std_linear_x': action_features['std_linear_x'],
                    'std_linear_y': action_features['std_linear_y'],
                    'std_angular_z': action_features['std_angular_z'],
                    'max_linear_x': action_features['max_linear_x'],
                    'max_linear_y': action_features['max_linear_y'],
                    'max_angular_z': action_features['max_angular_z'],
                    'min_linear_x': action_features['min_linear_x'],
                    'min_linear_y': action_features['min_linear_y'],
                    'min_angular_z': action_features['min_angular_z']
                },
                'action_metadata': {
                    'window_size': action_features['window_size'],
                    'frequency': action_features['frequency'],
                    'timestamp': stamp_ns
                },
                'recent_actions': list(self.action_buffer)[-10:]  # 保存最近10个原始动作
            }
            with open(base_paths['action'], 'w') as f:
                json.dump(action_data, f, indent=2)

            # ===== 保存目标点数据 =====
            # 保存目标点图像
            if goal_image is not None:
                cv2.imwrite(base_paths['goal_image'], goal_image)
                rospy.loginfo(f"保存目标点图像: {base_paths['goal_image']} (策略: {self.goal_image_strategy})")
            else:
                # 创建占位图像
                placeholder = np.zeros((100, 100, 3), dtype=np.uint8)
                cv2.putText(placeholder, "NO GOAL", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.imwrite(base_paths['goal_image'], placeholder)
                rospy.logwarn(f"无目标点图像，创建占位文件: {base_paths['goal_image']}")

            # 保存目标点数据（包含历史目标点）
            goal_data = {
                'episode_id': episode_id,
                'stamp_ns': stamp_ns,
                'goal_image_stamp': goal_image_stamp,
                'goal_image_strategy': self.goal_image_strategy,
                'current_goal_pose': self.current_goal_pose,
                'goal_history': self.goal_history[-10:],  # 最近10个目标点
                'has_goal_image': goal_image is not None,
                'goal_image_shape': goal_image.shape if goal_image is not None else None
            }
            with open(base_paths['goal_data'], 'w') as f:
                json.dump(goal_data, f, indent=2)

            # 轨迹 JSON
            traj_data = {
                'episode_id': episode_id,
                'stamp_ns': stamp_ns,
                'recent_trajectory': recent_trajectory,
                'planned_trajectory': self.planned_trajectory,
                'plan_source': self.plan_source,
                'current_pose': robot_pose,
                'goal_pose': goal_pose,
                'goal_history': self.goal_history[-10:],  # 包含历史目标点
                'velocity': {
                    'linear_x': odom_msg.twist.twist.linear.x,
                    'linear_y': odom_msg.twist.twist.linear.y,
                    'angular_z': odom_msg.twist.twist.angular.z
                },
                'action_data': action_data,  # 包含增强的Action数据
                'imu_data': imu_data,
                'lidar_metadata': {
                    'range_min': lidar_msg.range_min,
                    'range_max': lidar_msg.range_max,
                    'angle_min': lidar_msg.angle_min,
                    'angle_max': lidar_msg.angle_max,
                    'angle_increment': lidar_msg.angle_increment,
                    'num_ranges': len(lidar_msg.ranges)
                },
                'goal_data': goal_data,
                'quality_flags': quality_flags,
                'nav_semantics': nav_semantics,
                'split': split if self.enable_split else ""
            }
            with open(base_paths['trajectory'], 'w') as f:
                json.dump(traj_data, f, indent=2)

            # 轨迹可视化
            self._visualize_trajectory(recent_trajectory, robot_pose, goal_pose, base_paths['trajectory_vis'])

            # CSV
            self._update_csv_index(
                stamp_ns, odom_stamp_ns, goal_stamp_ns, episode_id, split,
                base_paths, robot_pose, odom_msg, imu_msg, lidar_msg,
                goal_pose, recent_trajectory, action_features,  # 新增参数
                quality_flags, nav_semantics, goal_image_stamp
            )

        except Exception as e:
            rospy.logerr(f"保存数据失败: {e}")
            traceback.print_exc()

    def _update_csv_index(self, stamp_ns, odom_stamp_ns, goal_stamp_ns, episode_id, split,
                          base_paths, robot_pose, odom_msg, imu_msg, lidar_msg,
                          goal_pose, recent_trajectory, action_features=None,
                          quality_flags=None, nav_semantics=None, goal_image_stamp=0):
        # 相机内参
        if self.camera_info:
            fx, fy, cx, cy = self.camera_info.K[0], self.camera_info.K[4], self.camera_info.K[2], self.camera_info.K[5]
        else:
            fx = fy = cx = cy = 0.0

        plan_len = len(self.planned_trajectory) if self.planned_trajectory else 0
        has_plan = 1 if plan_len > 0 else 0
        plan_source = self.plan_source if self.plan_source else ""

        # 使用增强的Action特征
        af = action_features or self._get_default_action_features()

        qf = quality_flags or {}
        ns = nav_semantics or {}

        row = [
            # 索引
            episode_id if episode_id is not None else 0,
            stamp_ns,
            odom_stamp_ns,
            goal_stamp_ns,
            (split if self.enable_split else ""),
            # 文件路径
            f"rgb/{os.path.basename(base_paths['rgb'])}",
            f"depth/{os.path.basename(base_paths['depth'])}",
            f"depth_vis/{os.path.basename(base_paths['depth_vis'])}",
            f"trajectory/{os.path.basename(base_paths['trajectory'])}",
            f"trajectory_vis/{os.path.basename(base_paths['trajectory_vis'])}",
            f"lidar/{os.path.basename(base_paths['lidar'])}",
            f"imu/{os.path.basename(base_paths['imu'])}",
            f"actions/{os.path.basename(base_paths['action'])}",
            f"goal_images/{os.path.basename(base_paths['goal_image'])}",
            f"goal_data/{os.path.basename(base_paths['goal_data'])}",
            # 相机内参
            fx, fy, cx, cy,
            # 状态
            robot_pose['x'], robot_pose['y'], robot_pose['theta'],
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y,
            odom_msg.twist.twist.angular.z,
            # IMU
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z,
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z,
            imu_msg.orientation.x,
            imu_msg.orientation.y,
            imu_msg.orientation.z,
            imu_msg.orientation.w,
            # LiDAR 元数据
            lidar_msg.range_min,
            lidar_msg.range_max,
            lidar_msg.angle_min,
            lidar_msg.angle_max,
            lidar_msg.angle_increment,
            len(lidar_msg.ranges),
            # 动作（当前cmd_vel）
            af['current_linear_x'], af['current_linear_y'], af['current_angular_z'],
            # 增强的Action特征
            af['mean_linear_x'], af['mean_linear_y'], af['mean_angular_z'],
            af['std_linear_x'], af['std_linear_y'], af['std_angular_z'],
            af['max_linear_x'], af['max_linear_y'], af['max_angular_z'],
            af['min_linear_x'], af['min_linear_y'], af['min_angular_z'],
            af['window_size'], af['frequency'],
            # 目标与规划
            (goal_pose['x'] if goal_pose else 0.0),
            (goal_pose['y'] if goal_pose else 0.0),
            (goal_pose['theta'] if goal_pose else 0.0),
            goal_image_stamp,
            self.goal_image_strategy,
            len(recent_trajectory),
            has_plan,
            plan_len,
            plan_source,
            # 质量与语义
            int(qf.get('bad_sync', False)),
            int(qf.get('low_depth', False)),
            int(qf.get('low_lidar', False)),
            int(qf.get('is_duplicate_motion', False)),
            int(ns.get('has_goal', False)),
            ns.get('distance_to_goal', 0.0),
            ns.get('heading_error', 0.0),
            int(ns.get('near_obstacle', False)),
            int(ns.get('just_reached', False)),
            ns.get('min_lidar', float('inf')),
        ]

        if self.enable_split:
            if split == "train":
                self.csv_writer_train.writerow(row)
                self.csv_train.flush()
            else:
                self.csv_writer_test.writerow(row)
                self.csv_test.flush()
        else:
            self.csv_writer.writerow(row)
            self.csv_file.flush()

    # ===== 工具函数 =====
    @staticmethod
    def get_yaw_from_quaternion(quat):
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def _save_depth_visualization(self, depth_image, save_path):
        try:
            if depth_image.dtype in (np.float32, np.float64):
                depth_vis = np.nan_to_num(depth_image, copy=True, nan=0.0, posinf=10.0, neginf=0.0)
                depth_vis = np.clip(depth_vis, 0, 10)
                depth_vis = (depth_vis / 10.0 * 255).astype(np.uint8)
            elif depth_image.dtype == np.uint16:
                depth_vis = (depth_image / 1000.0 * 255).astype(np.uint8)
            else:
                depth_vis = depth_image.astype(np.uint8)
            depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            cv2.imwrite(save_path, depth_colored)
        except Exception as e:
            rospy.logerr(f"深度可视化保存失败: {e}")

    def _visualize_trajectory(self, trajectory, current_pose, goal_pose, save_path):
        try:
            width, height, scale = 800, 800, 50
            origin_x, origin_y = width // 2, height // 2
            img = np.full((height, width, 3), 255, dtype=np.uint8)

            if trajectory and len(trajectory) > 1:
                pts = []
                for p in trajectory:
                    px = int(origin_x + p['x'] * scale)
                    py = int(origin_y - p['y'] * scale)
                    pts.append((px, py))

                # 绘制轨迹线
                for i in range(1, len(pts)):
                    cv2.line(img, pts[i - 1], pts[i], (0, 0, 255), 2)

            # 绘制当前位姿
            if current_pose:
                cx = int(origin_x + current_pose['x'] * scale)
                cy = int(origin_y - current_pose['y'] * scale)
                cv2.circle(img, (cx, cy), 8, (0, 255, 0), -1)
                # 绘制方向
                arrow_length = 20
                end_x = int(cx + arrow_length * math.cos(current_pose['theta']))
                end_y = int(cy - arrow_length * math.sin(current_pose['theta']))
                cv2.arrowedLine(img, (cx, cy), (end_x, end_y), (0, 255, 0), 2)

            # 绘制目标位姿
            if goal_pose:
                gx = int(origin_x + goal_pose['x'] * scale)
                gy = int(origin_y - goal_pose['y'] * scale)
                cv2.circle(img, (gx, gy), 8, (255, 0, 0), -1)
                # 绘制方向
                arrow_length = 20
                end_x = int(gx + arrow_length * math.cos(goal_pose['theta']))
                end_y = int(gy - arrow_length * math.sin(goal_pose['theta']))
                cv2.arrowedLine(img, (gx, gy), (end_x, end_y), (255, 0, 0), 2)

            cv2.imwrite(save_path, img)
        except Exception as e:
            rospy.logerr(f"轨迹可视化失败: {e}")

    def __del__(self):
        """析构函数，确保文件正确关闭"""
        if hasattr(self, 'csv_train') and self.csv_train:
            self.csv_train.close()
        if hasattr(self, 'csv_test') and self.csv_test:
            self.csv_test.close()
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()


if __name__ == '__main__':
    try:
        collector = EnhancedNavigationDataCollector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"导航数据收集器异常: {e}")
        traceback.print_exc()