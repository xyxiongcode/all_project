#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

        # —— 新增：数据集切分参数 —— #
        self.enable_split = rospy.get_param('~enable_split', True)  # 是否开启 train/test 切分
        self.train_ratio = rospy.get_param('~train_ratio', 0.8)  # 训练集比例
        self.split_strategy = rospy.get_param('~split_strategy', 'episode')  # 'episode' 或 'frame'
        self.split_seed = int(rospy.get_param('~split_seed', 1234))  # frame 模式用

        # 创建目录（带切分）- 新增目标点目录
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
        self.action_history = deque(maxlen=100)  # 最近动作
        self.latest_imu = None
        self.latest_lidar = None
        self.latest_odom = None
        self.last_cmd_vel = None  # 最近一条 /cmd_vel
        self._rng = np.random.RandomState(self.split_seed)  # 供 frame 模式使用

        # ===== 目标点相关变量 =====
        self.current_goal_image = None  # 当前目标点图像
        self.current_goal_pose = None  # 当前目标点坐标
        self.goal_image_stamp = None  # 目标点图像时间戳

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

        # 非同步
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=50)

        # 规划多源
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self._goal_callback, queue_size=10)
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.trajectory_callback, queue_size=10)
        rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.trajectory_callback, queue_size=10)
        rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan', Path, self.trajectory_callback, queue_size=10)
        rospy.Subscriber('/camera_info_left', CameraInfo, self._camera_info_callback, queue_size=1)

        # ===== 新增：目标点图像订阅 =====
        rospy.Subscriber('/goal_image', Image, self._goal_image_callback, queue_size=10)

        self.plan_source = None
        self.bridge = CvBridge()
        self.current_goal_msg = None
        self.current_episode_id = None
        self.planned_trajectory = None
        self.camera_info = None

        # ===== CSV 文件（若切分则各写各的） =====
        self.csv_header = [
            # 索引
            'episode_id', 'stamp', 'odom_stamp', 'goal_stamp', 'split',
            # 路径
            'rgb_path', 'depth_path', 'depth_vis_path',
            'trajectory_path', 'trajectory_vis_path',
            'lidar_path', 'imu_path', 'action_path',
            'goal_image_path', 'goal_data_path',  # 新增目标点路径
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
            # 目标与规划
            'goal_x', 'goal_y', 'goal_theta',
            'goal_image_stamp',  # 新增目标点图像时间戳
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

        rospy.loginfo("导航数据收集器已启动（含切分：%s, 比例=%.2f, 策略=%s）",
                      str(self.enable_split), self.train_ratio, self.split_strategy)

    # ===== 回调函数 =====
    def _camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def _goal_callback(self, msg: PoseStamped):
        """目标点坐标回调"""
        self.current_goal_msg = msg
        self.current_episode_id = msg.header.stamp.to_nsec()

        # 保存目标点坐标
        self.current_goal_pose = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'theta': self.get_yaw_from_quaternion(msg.pose.orientation),
            'stamp': msg.header.stamp.to_nsec()
        }
        rospy.loginfo(f"收到新目标点: ({self.current_goal_pose['x']:.2f}, {self.current_goal_pose['y']:.2f})")

    def _goal_image_callback(self, msg: Image):
        """目标点图像回调"""
        try:
            # 转换目标点图像
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
        now_ns = rospy.Time.now().to_nsec()
        self.last_cmd_vel = (now_ns, msg)
        self.action_history.append({
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'angular_z': msg.angular.z,
            'timestamp': now_ns
        })

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

            # —— 质量判定 —— #
            rgb_t = rgb_msg.header.stamp.to_sec()
            depth_t = depth_msg.header.stamp.to_sec()
            sync_ok = (abs(rgb_t - depth_t) <= 0.03)  # 30ms

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

            # —— 目标语义 —— #
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

            # 硬过滤：整条样本同时丢弃，确保模态对齐
            if quality_flags["bad_sync"] or (quality_flags["low_depth"] and quality_flags["low_lidar"]):
                return

            # 轨迹历史
            self.trajectory_history.append(robot_pose)

            # 图像
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = depth_np  # 已解码

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

            # —— 决定 train/test —— #
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
                robot_pose=robot_pose,
                goal_pose=goal_pose,
                recent_trajectory=recent_traj,
                odom_msg=odom_msg,
                lidar_msg=lidar_msg,
                imu_msg=imu_msg,
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
            return ""  # 不切分
        if self.split_strategy == 'episode':
            # 根据 episode_id 稳定划分，保证一个 episode 不被拆分
            key = str(episode_id if episode_id is not None else 0).encode('utf-8')
            h = hashlib.md5(key).hexdigest()
            # 取 MD5 的低 12 位转 [0,1)
            frac = (int(h[-12:], 16) % 1000000) / 1000000.0
            return "train" if frac < self.train_ratio else "test"
        else:
            # frame：按帧随机（可复现）
            return "train" if self._rng.rand() < self.train_ratio else "test"

    # ===== 增强的数据保存 =====
    def _save_enhanced_data(self, stamp_ns, odom_stamp_ns, goal_stamp_ns, episode_id, split,
                            rgb_image, depth_image, robot_pose, goal_pose, recent_trajectory,
                            odom_msg, lidar_msg, imu_msg, quality_flags=None, nav_semantics=None):
        try:
            # 根目录（考虑切分）
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

            # 新增：目标点相关文件名
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
                'goal_image': os.path.join(root, "goal_images", goal_image_filename),  # 新增
                'goal_data': os.path.join(root, "goal_data", goal_data_filename)  # 新增
            }

            # 图像
            cv2.imwrite(base_paths['rgb'], rgb_image)
            np.save(base_paths['depth'], depth_image)
            self._save_depth_visualization(depth_image, base_paths['depth_vis'])

            # LiDAR（含元数据）
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

            # 动作（cmd_vel）
            recent_actions = list(self.action_history)[-10:]
            cmd_vel_now = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
            if self.last_cmd_vel is not None:
                _, msg = self.last_cmd_vel
                cmd_vel_now = {
                    'linear_x': msg.linear.x,
                    'linear_y': msg.linear.y,
                    'angular_z': msg.angular.z
                }
            action_data = {
                'cmd_vel_now': cmd_vel_now,
                'recent_actions': recent_actions,
                'timestamp': stamp_ns
            }
            with open(base_paths['action'], 'w') as f:
                json.dump(action_data, f, indent=2)

            # ===== 新增：保存目标点数据 =====
            goal_image_stamp = self.goal_image_stamp if self.goal_image_stamp else 0

            # 保存目标点图像（如果有）
            if self.current_goal_image is not None:
                cv2.imwrite(base_paths['goal_image'], self.current_goal_image)
                rospy.loginfo(f"保存目标点图像: {base_paths['goal_image']}")
            else:
                # 如果没有目标点图像，创建一个空的占位文件或复制当前RGB图像
                cv2.imwrite(base_paths['goal_image'], np.zeros((100, 100, 3), dtype=np.uint8))
                rospy.logwarn(f"无目标点图像，创建占位文件: {base_paths['goal_image']}")

            # 保存目标点数据
            goal_data = {
                'episode_id': episode_id,
                'stamp_ns': stamp_ns,
                'goal_image_stamp': goal_image_stamp,
                'goal_pose': self.current_goal_pose if self.current_goal_pose else None,
                'has_goal_image': self.current_goal_image is not None,
                'goal_image_shape': self.current_goal_image.shape if self.current_goal_image is not None else None
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
                'velocity': {
                    'linear_x': odom_msg.twist.twist.linear.x,
                    'linear_y': odom_msg.twist.twist.linear.y,
                    'angular_z': odom_msg.twist.twist.angular.z
                },
                'imu_data': imu_data,
                'lidar_metadata': {
                    'range_min': lidar_msg.range_min,
                    'range_max': lidar_msg.range_max,
                    'angle_min': lidar_msg.angle_min,
                    'angle_max': lidar_msg.angle_max,
                    'angle_increment': lidar_msg.angle_increment,
                    'num_ranges': len(lidar_msg.ranges)
                },
                'goal_data': goal_data,  # 包含目标点信息
                'quality_flags': quality_flags,
                'nav_semantics': nav_semantics,
                'split': split if self.enable_split else ""
            }
            with open(base_paths['trajectory'], 'w') as f:
                json.dump(traj_data, f, indent=2)

            # 轨迹可视化
            self._visualize_trajectory(recent_trajectory, robot_pose, goal_pose, base_paths['trajectory_vis'])

            # CSV（写到对应 split 的 CSV）
            self._update_csv_index(
                stamp_ns, odom_stamp_ns, goal_stamp_ns, episode_id, split,
                base_paths, robot_pose, odom_msg, imu_msg, lidar_msg,
                goal_pose, recent_trajectory, quality_flags, nav_semantics,
                goal_image_stamp  # 新增参数
            )

        except Exception as e:
            rospy.logerr(f"保存数据失败: {e}")
            traceback.print_exc()

    def _update_csv_index(self, stamp_ns, odom_stamp_ns, goal_stamp_ns, episode_id, split,
                          base_paths, robot_pose, odom_msg, imu_msg, lidar_msg,
                          goal_pose, recent_trajectory, quality_flags=None, nav_semantics=None,
                          goal_image_stamp=0):  # 新增参数
        # 相机内参
        if self.camera_info:
            fx, fy, cx, cy = self.camera_info.K[0], self.camera_info.K[4], self.camera_info.K[2], self.camera_info.K[5]
        else:
            fx = fy = cx = cy = 0.0

        plan_len = len(self.planned_trajectory) if self.planned_trajectory else 0
        has_plan = 1 if plan_len > 0 else 0
        plan_source = self.plan_source if self.plan_source else ""

        # 最近 cmd_vel
        if self.last_cmd_vel is not None:
            cmd = self.last_cmd_vel[1]
            cmd_lx, cmd_ly, cmd_az = cmd.linear.x, cmd.linear.y, cmd.angular.z
        else:
            cmd_lx = cmd_ly = cmd_az = 0.0

        qf = quality_flags or {}
        ns = nav_semantics or {}

        row = [
            # 索引
            episode_id if episode_id is not None else 0,
            stamp_ns,
            odom_stamp_ns,
            goal_stamp_ns,
            (split if self.enable_split else ""),
            # 文件路径（相对 split 根）
            f"rgb/{os.path.basename(base_paths['rgb'])}",
            f"depth/{os.path.basename(base_paths['depth'])}",
            f"depth_vis/{os.path.basename(base_paths['depth_vis'])}",
            f"trajectory/{os.path.basename(base_paths['trajectory'])}",
            f"trajectory_vis/{os.path.basename(base_paths['trajectory_vis'])}",
            f"lidar/{os.path.basename(base_paths['lidar'])}",
            f"imu/{os.path.basename(base_paths['imu'])}",
            f"actions/{os.path.basename(base_paths['action'])}",
            f"goal_images/{os.path.basename(base_paths['goal_image'])}",  # 新增
            f"goal_data/{os.path.basename(base_paths['goal_data'])}",  # 新增
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
            # 动作（cmd_vel）
            cmd_lx, cmd_ly, cmd_az,
            # 目标与规划
            (goal_pose['x'] if goal_pose else 0.0),
            (goal_pose['y'] if goal_pose else 0.0),
            (goal_pose['theta'] if goal_pose else 0.0),
            goal_image_stamp,  # 新增
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
                for i in range(len(pts) - 1):
                    cv2.line(img, pts[i], pts[i + 1], (0, 0, 255), 2)

            if current_pose:
                cx = int(origin_x + current_pose['x'] * scale)
                cy = int(origin_y - current_pose['y'] * scale)
                cv2.circle(img, (cx, cy), 8, (0, 200, 0), -1)

            if goal_pose:
                gx = int(origin_x + goal_pose['x'] * scale)
                gy = int(origin_y - goal_pose['y'] * scale)
                cv2.circle(img, (gx, gy), 8, (255, 0, 0), -1)

            cv2.imwrite(save_path, img)
        except Exception as e:
            rospy.logwarn(f"轨迹可视化失败: {e}")

    def shutdown(self):
        try:
            # 关闭 CSV
            if self.enable_split:
                if hasattr(self, 'csv_train'):
                    self.csv_train.close()
                if hasattr(self, 'csv_test'):
                    self.csv_test.close()
            else:
                if hasattr(self, 'csv_file'):
                    self.csv_file.close()
        finally:
            # 统计文件数量（分别统计）
            def count_dir(root):
                counts = {}
                for d in self.subdirs:
                    path = os.path.join(root, d)
                    if not os.path.exists(path):
                        counts[d] = 0
                        continue
                    if d in ('depth', 'lidar'):
                        ext = '.npy'
                    elif d in ['trajectory', 'imu', 'actions', 'goal_data']:  # 新增 goal_data
                        ext = '.json'
                    else:
                        ext = '.png'
                    counts[d] = len([f for f in os.listdir(path) if f.endswith(ext)])
                return counts

            if self.enable_split:
                train_counts = count_dir(os.path.join(self.save_dir, "train"))
                test_counts = count_dir(os.path.join(self.save_dir, "test"))

                rospy.loginfo("=== 数据集统计 ===")
                rospy.loginfo("训练集:")
                for d, count in train_counts.items():
                    rospy.loginfo(f"  {d}: {count} 文件")

                rospy.loginfo("测试集:")
                for d, count in test_counts.items():
                    rospy.loginfo(f"  {d}: {count} 文件")

                total_train = sum(train_counts.values())
                total_test = sum(test_counts.values())
                rospy.loginfo(f"训练集总计: {total_train} 文件")
                rospy.loginfo(f"测试集总计: {total_test} 文件")
                rospy.loginfo(f"整体总计: {total_train + total_test} 文件")
                rospy.loginfo(
                    f"训练/测试比例: {total_train / (total_train + total_test):.3f} / {total_test / (total_train + total_test):.3f}")
            else:
                counts = count_dir(self.save_dir)
                rospy.loginfo("=== 数据集统计 ===")
                for d, count in counts.items():
                    rospy.loginfo(f"{d}: {count} 文件")
                rospy.loginfo(f"总计: {sum(counts.values())} 文件")

            rospy.loginfo("导航数据收集器已关闭")

    def run(self):
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

if __name__ == '__main__':
    try:
        collector = EnhancedNavigationDataCollector()
        collector.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"导航数据收集器启动失败: {e}")
        traceback.print_exc()


# import rospy
# import json
# import csv
# import os
# import cv2
# import numpy as np
# from collections import deque
# from geometry_msgs.msg import PoseStamped, Twist
# from sensor_msgs.msg import Image, CameraInfo, LaserScan, Imu
# from nav_msgs.msg import Path, Odometry
# import message_filters as mf
# from cv_bridge import CvBridge
# import math
# import traceback
# from tf.transformations import euler_from_quaternion
# import hashlib
# import random as pyrand
#
#
# class EnhancedNavigationDataCollector:
#     def __init__(self):
#         rospy.init_node('navigation_data_collector')
#
#         # ===== 保存目录 =====
#         self.save_dir = rospy.get_param('~save_dir', "/media/gr-agv-x9xy/backup_xxy/camera_data")
#
#         # —— 新增：数据集切分参数 —— #
#         self.enable_split = rospy.get_param('~enable_split', True)            # 是否开启 train/test 切分
#         self.train_ratio = rospy.get_param('~train_ratio', 0.8)               # 训练集比例
#         self.split_strategy = rospy.get_param('~split_strategy', 'episode')   # 'episode' 或 'frame'
#         self.split_seed = int(rospy.get_param('~split_seed', 1234))           # frame 模式用
#
#         # 创建目录（带切分）
#         self.subdirs = ["rgb", "depth", "depth_vis", "trajectory", "trajectory_vis", "lidar", "imu", "actions"]
#         if self.enable_split:
#             for split in ["train", "test"]:
#                 for d in self.subdirs:
#                     os.makedirs(os.path.join(self.save_dir, split, d), exist_ok=True)
#         else:
#             for d in self.subdirs:
#                 os.makedirs(os.path.join(self.save_dir, d), exist_ok=True)
#
#         # ===== 采样控制 =====
#         self.counter = 0
#         self.max_count = rospy.get_param('~max_count', 50000)
#         self.last_save_time = 0.0
#         self.save_interval = rospy.get_param('~save_interval', 0.5)
#         self.slop = rospy.get_param('~sync_slop', 0.05)
#
#         # ===== 数据缓冲区 =====
#         self.trajectory_history = deque(maxlen=rospy.get_param('~traj_history_len', 1000))
#         self.trajectory_length = rospy.get_param('~traj_dump_len', 50)
#         self.action_history = deque(maxlen=100)  # 最近动作
#         self.latest_imu = None
#         self.latest_lidar = None
#         self.latest_odom = None
#         self.last_cmd_vel = None  # 最近一条 /cmd_vel
#         self._rng = np.random.RandomState(self.split_seed)  # 供 frame 模式使用
#
#         # ===== 订阅（同步） =====
#         rgb_sub = mf.Subscriber('/rgb_left', Image)
#         depth_sub = mf.Subscriber('/depth_left', Image)
#         odom_sub = mf.Subscriber('/odom', Odometry)
#         lidar_sub = mf.Subscriber('/scan', LaserScan)
#         imu_sub = mf.Subscriber('/imu', Imu)
#
#         self.ts = mf.ApproximateTimeSynchronizer(
#             [rgb_sub, depth_sub, odom_sub, lidar_sub, imu_sub],
#             queue_size=20,
#             slop=self.slop
#         )
#         self.ts.registerCallback(self.sync_callback)
#
#         # 非同步
#         rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=50)
#
#         # 规划多源
#         rospy.Subscriber('/move_base_simple/goal', PoseStamped, self._goal_callback, queue_size=10)
#         rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.trajectory_callback, queue_size=10)
#         rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.trajectory_callback, queue_size=10)
#         rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan', Path, self.trajectory_callback, queue_size=10)
#         rospy.Subscriber('/camera_info_left', CameraInfo, self._camera_info_callback, queue_size=1)
#
#         self.plan_source = None
#         self.bridge = CvBridge()
#         self.current_goal_msg = None
#         self.current_episode_id = None
#         self.planned_trajectory = None
#         self.camera_info = None
#
#         # ===== CSV 文件（若切分则各写各的） =====
#         self.csv_header = [
#             # 索引
#             'episode_id', 'stamp', 'odom_stamp', 'goal_stamp', 'split',
#             # 路径
#             'rgb_path', 'depth_path', 'depth_vis_path',
#             'trajectory_path', 'trajectory_vis_path',
#             'lidar_path', 'imu_path', 'action_path',
#             # 相机内参
#             'fx', 'fy', 'cx', 'cy',
#             # 机器人状态
#             'robot_x', 'robot_y', 'robot_theta',
#             'velocity_x', 'velocity_y', 'velocity_theta',
#             # IMU
#             'imu_linear_accel_x', 'imu_linear_accel_y', 'imu_linear_accel_z',
#             'imu_angular_vel_x', 'imu_angular_vel_y', 'imu_angular_vel_z',
#             'imu_orientation_x', 'imu_orientation_y', 'imu_orientation_z', 'imu_orientation_w',
#             # LiDAR 元数据
#             'lidar_range_min', 'lidar_range_max', 'lidar_angle_min', 'lidar_angle_max',
#             'lidar_angle_increment', 'lidar_num_ranges',
#             # 动作（cmd_vel）
#             'cmd_vel_linear_x', 'cmd_vel_linear_y', 'cmd_vel_angular_z',
#             # 目标与规划
#             'goal_x', 'goal_y', 'goal_theta',
#             'trajectory_length', 'has_planned_trajectory', 'plan_length', 'plan_source',
#             # 质量与语义
#             'invalid_bad_sync', 'invalid_low_depth', 'invalid_low_lidar', 'is_duplicate_motion',
#             'has_goal', 'distance_to_goal', 'heading_error', 'near_obstacle', 'just_reached', 'min_lidar'
#         ]
#
#         if self.enable_split:
#             self.csv_train_path = os.path.join(self.save_dir, "train", "enhanced_navigation_data.csv")
#             self.csv_test_path  = os.path.join(self.save_dir, "test",  "enhanced_navigation_data.csv")
#             self.csv_train = open(self.csv_train_path, "w", newline="")
#             self.csv_test  = open(self.csv_test_path,  "w", newline="")
#             self.csv_writer_train = csv.writer(self.csv_train)
#             self.csv_writer_test  = csv.writer(self.csv_test)
#             self.csv_writer_train.writerow(self.csv_header)
#             self.csv_writer_test.writerow(self.csv_header)
#         else:
#             self.csv_path = os.path.join(self.save_dir, "enhanced_navigation_data.csv")
#             self.csv_file = open(self.csv_path, "w", newline="")
#             self.csv_writer = csv.writer(self.csv_file)
#             self.csv_writer.writerow(self.csv_header)
#
#         rospy.loginfo("导航数据收集器已启动（含切分：%s, 比例=%.2f, 策略=%s）",
#                       str(self.enable_split), self.train_ratio, self.split_strategy)
#
#     # ===== 回调函数 =====
#     def _camera_info_callback(self, msg: CameraInfo):
#         self.camera_info = msg
#
#     def _goal_callback(self, msg: PoseStamped):
#         self.current_goal_msg = msg
#         self.current_episode_id = msg.header.stamp.to_nsec()
#
#     def trajectory_callback(self, msg: Path):
#         try:
#             if msg.poses:
#                 self.planned_trajectory = [{
#                     'x': p.pose.position.x,
#                     'y': p.pose.position.y,
#                     'z': p.pose.position.z
#                 } for p in msg.poses]
#                 self.plan_source = getattr(msg, "_connection_header", {}).get("topic", "unknown")
#         except Exception as e:
#             rospy.logwarn(f"轨迹处理错误: {e}")
#
#     def cmd_vel_callback(self, msg: Twist):
#         now_ns = rospy.Time.now().to_nsec()
#         self.last_cmd_vel = (now_ns, msg)
#         self.action_history.append({
#             'linear_x': msg.linear.x,
#             'linear_y': msg.linear.y,
#             'angular_z': msg.angular.z,
#             'timestamp': now_ns
#         })
#
#     # ===== 核心同步回调 =====
#     def sync_callback(self, rgb_msg: Image, depth_msg: Image, odom_msg: Odometry,
#                       lidar_msg: LaserScan, imu_msg: Imu):
#         try:
#             # 节流
#             now = rospy.Time.now().to_sec()
#             if now - self.last_save_time < self.save_interval:
#                 return
#             if self.counter >= self.max_count:
#                 rospy.loginfo("达到最大保存数量，停止采集")
#                 return
#
#             # 位姿
#             robot_pose = {
#                 'x': odom_msg.pose.pose.position.x,
#                 'y': odom_msg.pose.pose.position.y,
#                 'theta': self.get_yaw_from_quaternion(odom_msg.pose.pose.orientation),
#                 'timestamp': odom_msg.header.stamp.to_nsec()
#             }
#
#             # —— 质量判定 —— #
#             rgb_t = rgb_msg.header.stamp.to_sec()
#             depth_t = depth_msg.header.stamp.to_sec()
#             sync_ok = (abs(rgb_t - depth_t) <= 0.03)  # 30ms
#
#             depth_np = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
#             depth_finite = np.isfinite(depth_np)
#             valid_depth_ratio = float(depth_finite.sum()) / max(1, depth_finite.size)
#
#             ranges = np.asarray(lidar_msg.ranges, dtype=np.float32)
#             finite_mask = np.isfinite(ranges) & (ranges > lidar_msg.range_min) & (ranges < lidar_msg.range_max)
#             valid_lidar_ratio = float(finite_mask.sum()) / max(1, ranges.size)
#             min_lidar = float(np.nanmin(ranges[finite_mask])) if finite_mask.any() else float('inf')
#             near_obstacle = (min_lidar < 0.5)
#
#             if self.trajectory_history and len(self.trajectory_history) >= 1:
#                 last_pose = self.trajectory_history[-1]
#                 dpos = math.hypot(robot_pose['x'] - last_pose['x'], robot_pose['y'] - last_pose['y'])
#                 dyaw = abs(robot_pose['theta'] - last_pose['theta'])
#             else:
#                 dpos, dyaw = 1e9, 1e9
#             is_duplicate_motion = (dpos < 0.01 and dyaw < math.radians(1.0))
#
#             # —— 目标语义 —— #
#             has_goal = self.current_goal_msg is not None
#             distance_to_goal, heading_error, just_reached = 0.0, 0.0, False
#             if has_goal:
#                 gx = self.current_goal_msg.pose.position.x
#                 gy = self.current_goal_msg.pose.position.y
#                 rx, ry, yaw = robot_pose['x'], robot_pose['y'], robot_pose['theta']
#                 vec_g = math.atan2(gy - ry, gx - rx)
#                 distance_to_goal = math.hypot(gx - rx, gy - ry)
#                 heading_error = (vec_g - yaw + math.pi) % (2*math.pi) - math.pi
#                 just_reached = (distance_to_goal < 0.05 and
#                                 abs(odom_msg.twist.twist.linear.x) < 0.02 and
#                                 abs(odom_msg.twist.twist.angular.z) < 0.02)
#
#             quality_flags = {
#                 "bad_sync": (not sync_ok),
#                 "low_depth": (valid_depth_ratio < 0.6),
#                 "low_lidar": (valid_lidar_ratio < 0.6),
#                 "is_duplicate_motion": is_duplicate_motion
#             }
#             nav_semantics = {
#                 "has_goal": has_goal,
#                 "distance_to_goal": float(distance_to_goal),
#                 "heading_error": float(heading_error),
#                 "near_obstacle": bool(near_obstacle),
#                 "just_reached": bool(just_reached),
#                 "min_lidar": float(min_lidar)
#             }
#
#             # 硬过滤：整条样本同时丢弃，确保模态对齐
#             if quality_flags["bad_sync"] or (quality_flags["low_depth"] and quality_flags["low_lidar"]):
#                 return
#
#             # 轨迹历史
#             self.trajectory_history.append(robot_pose)
#
#             # 图像
#             rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
#             depth_image = depth_np  # 已解码
#
#             # 最近轨迹
#             recent_traj = list(self.trajectory_history)[-self.trajectory_length:]
#
#             # 目标信息
#             goal_pose = None
#             goal_stamp_ns = 0
#             if self.current_goal_msg is not None:
#                 goal_pose = {
#                     'x': self.current_goal_msg.pose.position.x,
#                     'y': self.current_goal_msg.pose.position.y,
#                     'theta': self.get_yaw_from_quaternion(self.current_goal_msg.pose.orientation)
#                 }
#                 goal_stamp_ns = self.current_goal_msg.header.stamp.to_nsec()
#
#             # —— 决定 train/test —— #
#             split = self.decide_split(self.current_episode_id)
#
#             # 保存
#             stamp_ns = rgb_msg.header.stamp.to_nsec()
#             self._save_enhanced_data(
#                 stamp_ns=stamp_ns,
#                 odom_stamp_ns=odom_msg.header.stamp.to_nsec(),
#                 goal_stamp_ns=goal_stamp_ns,
#                 episode_id=self.current_episode_id,
#                 split=split,
#                 rgb_image=rgb_image,
#                 depth_image=depth_image,
#                 robot_pose=robot_pose,
#                 goal_pose=goal_pose,
#                 recent_trajectory=recent_traj,
#                 odom_msg=odom_msg,
#                 lidar_msg=lidar_msg,
#                 imu_msg=imu_msg,
#                 quality_flags=quality_flags,
#                 nav_semantics=nav_semantics
#             )
#
#             self.last_save_time = now
#             self.counter += 1
#             if self.counter % 100 == 0:
#                 rospy.loginfo(f"已保存 {self.counter} 个数据样本")
#
#         except Exception as e:
#             rospy.logerr(f"同步回调错误: {e}")
#             traceback.print_exc()
#
#     # ===== 切分策略 =====
#     def decide_split(self, episode_id):
#         if not self.enable_split:
#             return ""  # 不切分
#         if self.split_strategy == 'episode':
#             # 根据 episode_id 稳定划分，保证一个 episode 不被拆分
#             key = str(episode_id if episode_id is not None else 0).encode('utf-8')
#             h = hashlib.md5(key).hexdigest()
#             # 取 MD5 的低 12 位转 [0,1)
#             frac = (int(h[-12:], 16) % 1000000) / 1000000.0
#             return "train" if frac < self.train_ratio else "test"
#         else:
#             # frame：按帧随机（可复现）
#             return "train" if self._rng.rand() < self.train_ratio else "test"
#
#     # ===== 增强的数据保存 =====
#     def _save_enhanced_data(self, stamp_ns, odom_stamp_ns, goal_stamp_ns, episode_id, split,
#                             rgb_image, depth_image, robot_pose, goal_pose, recent_trajectory,
#                             odom_msg, lidar_msg, imu_msg, quality_flags=None, nav_semantics=None):
#         try:
#             # 根目录（考虑切分）
#             root = os.path.join(self.save_dir, split) if (self.enable_split and split) else self.save_dir
#
#             # 文件名
#             rgb_filename = f"rgb_{stamp_ns}.png"
#             depth_filename = f"depth_{stamp_ns}.npy"
#             depth_vis_filename = f"depth_vis_{stamp_ns}.png"
#             traj_filename = f"trajectory_{stamp_ns}.json"
#             traj_vis_filename = f"trajectory_vis_{stamp_ns}.png"
#             lidar_filename = f"lidar_{stamp_ns}.npy"
#             imu_filename = f"imu_{stamp_ns}.json"
#             action_filename = f"action_{stamp_ns}.json"
#
#             # 路径
#             base_paths = {
#                 'rgb': os.path.join(root, "rgb", rgb_filename),
#                 'depth': os.path.join(root, "depth", depth_filename),
#                 'depth_vis': os.path.join(root, "depth_vis", depth_vis_filename),
#                 'trajectory': os.path.join(root, "trajectory", traj_filename),
#                 'trajectory_vis': os.path.join(root, "trajectory_vis", traj_vis_filename),
#                 'lidar': os.path.join(root, "lidar", lidar_filename),
#                 'imu': os.path.join(root, "imu", imu_filename),
#                 'action': os.path.join(root, "actions", action_filename)
#             }
#
#             # 图像
#             cv2.imwrite(base_paths['rgb'], rgb_image)
#             np.save(base_paths['depth'], depth_image)
#             self._save_depth_visualization(depth_image, base_paths['depth_vis'])
#
#             # LiDAR（含元数据）
#             lidar_pack = {
#                 'ranges': np.asarray(lidar_msg.ranges, dtype=np.float32),
#                 'angle_min': lidar_msg.angle_min,
#                 'angle_max': lidar_msg.angle_max,
#                 'angle_increment': lidar_msg.angle_increment,
#                 'range_min': lidar_msg.range_min,
#                 'range_max': lidar_msg.range_max,
#                 'stamp': lidar_msg.header.stamp.to_nsec()
#             }
#             np.save(base_paths['lidar'], lidar_pack, allow_pickle=True)
#
#             # IMU
#             imu_data = {
#                 'linear_acceleration': {
#                     'x': imu_msg.linear_acceleration.x,
#                     'y': imu_msg.linear_acceleration.y,
#                     'z': imu_msg.linear_acceleration.z
#                 },
#                 'angular_velocity': {
#                     'x': imu_msg.angular_velocity.x,
#                     'y': imu_msg.angular_velocity.y,
#                     'z': imu_msg.angular_velocity.z
#                 },
#                 'orientation': {
#                     'x': imu_msg.orientation.x,
#                     'y': imu_msg.orientation.y,
#                     'z': imu_msg.orientation.z,
#                     'w': imu_msg.orientation.w
#                 },
#                 'timestamp': stamp_ns
#             }
#             with open(base_paths['imu'], 'w') as f:
#                 json.dump(imu_data, f, indent=2)
#
#             # 动作（cmd_vel）
#             recent_actions = list(self.action_history)[-10:]
#             cmd_vel_now = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
#             if self.last_cmd_vel is not None:
#                 _, msg = self.last_cmd_vel
#                 cmd_vel_now = {
#                     'linear_x': msg.linear.x,
#                     'linear_y': msg.linear.y,
#                     'angular_z': msg.angular.z
#                 }
#             action_data = {
#                 'cmd_vel_now': cmd_vel_now,
#                 'recent_actions': recent_actions,
#                 'timestamp': stamp_ns
#             }
#             with open(base_paths['action'], 'w') as f:
#                 json.dump(action_data, f, indent=2)
#
#             # 轨迹 JSON
#             traj_data = {
#                 'episode_id': episode_id,
#                 'stamp_ns': stamp_ns,
#                 'recent_trajectory': recent_trajectory,
#                 'planned_trajectory': self.planned_trajectory,
#                 'plan_source': self.plan_source,
#                 'current_pose': robot_pose,
#                 'goal_pose': goal_pose,
#                 'velocity': {
#                     'linear_x': odom_msg.twist.twist.linear.x,
#                     'linear_y': odom_msg.twist.twist.linear.y,
#                     'angular_z': odom_msg.twist.twist.angular.z
#                 },
#                 'imu_data': imu_data,
#                 'lidar_metadata': {
#                     'range_min': lidar_msg.range_min,
#                     'range_max': lidar_msg.range_max,
#                     'angle_min': lidar_msg.angle_min,
#                     'angle_max': lidar_msg.angle_max,
#                     'angle_increment': lidar_msg.angle_increment,
#                     'num_ranges': len(lidar_msg.ranges)
#                 },
#                 'quality_flags': quality_flags,
#                 'nav_semantics': nav_semantics,
#                 'split': split if self.enable_split else ""
#             }
#             with open(base_paths['trajectory'], 'w') as f:
#                 json.dump(traj_data, f, indent=2)
#
#             # 轨迹可视化
#             self._visualize_trajectory(recent_trajectory, robot_pose, goal_pose, base_paths['trajectory_vis'])
#
#             # CSV（写到对应 split 的 CSV）
#             self._update_csv_index(
#                 stamp_ns, odom_stamp_ns, goal_stamp_ns, episode_id, split,
#                 base_paths, robot_pose, odom_msg, imu_msg, lidar_msg,
#                 goal_pose, recent_trajectory, quality_flags, nav_semantics
#             )
#
#         except Exception as e:
#             rospy.logerr(f"保存数据失败: {e}")
#             traceback.print_exc()
#
#     def _update_csv_index(self, stamp_ns, odom_stamp_ns, goal_stamp_ns, episode_id, split,
#                           base_paths, robot_pose, odom_msg, imu_msg, lidar_msg,
#                           goal_pose, recent_trajectory, quality_flags=None, nav_semantics=None):
#         # 相机内参
#         if self.camera_info:
#             fx, fy, cx, cy = self.camera_info.K[0], self.camera_info.K[4], self.camera_info.K[2], self.camera_info.K[5]
#         else:
#             fx = fy = cx = cy = 0.0
#
#         plan_len = len(self.planned_trajectory) if self.planned_trajectory else 0
#         has_plan = 1 if plan_len > 0 else 0
#         plan_source = self.plan_source if self.plan_source else ""
#
#         # 最近 cmd_vel
#         if self.last_cmd_vel is not None:
#             cmd = self.last_cmd_vel[1]
#             cmd_lx, cmd_ly, cmd_az = cmd.linear.x, cmd.linear.y, cmd.angular.z
#         else:
#             cmd_lx = cmd_ly = cmd_az = 0.0
#
#         qf = quality_flags or {}
#         ns = nav_semantics or {}
#
#         row = [
#             # 索引
#             episode_id if episode_id is not None else 0,
#             stamp_ns,
#             odom_stamp_ns,
#             goal_stamp_ns,
#             (split if self.enable_split else ""),
#             # 文件路径（相对 split 根）
#             f"rgb/{os.path.basename(base_paths['rgb'])}",
#             f"depth/{os.path.basename(base_paths['depth'])}",
#             f"depth_vis/{os.path.basename(base_paths['depth_vis'])}",
#             f"trajectory/{os.path.basename(base_paths['trajectory'])}",
#             f"trajectory_vis/{os.path.basename(base_paths['trajectory_vis'])}",
#             f"lidar/{os.path.basename(base_paths['lidar'])}",
#             f"imu/{os.path.basename(base_paths['imu'])}",
#             f"actions/{os.path.basename(base_paths['action'])}",
#             # 相机内参
#             fx, fy, cx, cy,
#             # 状态
#             robot_pose['x'], robot_pose['y'], robot_pose['theta'],
#             odom_msg.twist.twist.linear.x,
#             odom_msg.twist.twist.linear.y,
#             odom_msg.twist.twist.angular.z,
#             # IMU
#             imu_msg.linear_acceleration.x,
#             imu_msg.linear_acceleration.y,
#             imu_msg.linear_acceleration.z,
#             imu_msg.angular_velocity.x,
#             imu_msg.angular_velocity.y,
#             imu_msg.angular_velocity.z,
#             imu_msg.orientation.x,
#             imu_msg.orientation.y,
#             imu_msg.orientation.z,
#             imu_msg.orientation.w,
#             # LiDAR 元数据
#             lidar_msg.range_min,
#             lidar_msg.range_max,
#             lidar_msg.angle_min,
#             lidar_msg.angle_max,
#             lidar_msg.angle_increment,
#             len(lidar_msg.ranges),
#             # 动作（cmd_vel）
#             cmd_lx, cmd_ly, cmd_az,
#             # 目标与规划
#             (goal_pose['x'] if goal_pose else 0.0),
#             (goal_pose['y'] if goal_pose else 0.0),
#             (goal_pose['theta'] if goal_pose else 0.0),
#             len(recent_trajectory),
#             has_plan,
#             plan_len,
#             plan_source,
#             # 质量与语义
#             int(qf.get('bad_sync', False)),
#             int(qf.get('low_depth', False)),
#             int(qf.get('low_lidar', False)),
#             int(qf.get('is_duplicate_motion', False)),
#             int(ns.get('has_goal', False)),
#             ns.get('distance_to_goal', 0.0),
#             ns.get('heading_error', 0.0),
#             int(ns.get('near_obstacle', False)),
#             int(ns.get('just_reached', False)),
#             ns.get('min_lidar', float('inf')),
#         ]
#
#         if self.enable_split:
#             if split == "train":
#                 self.csv_writer_train.writerow(row)
#                 self.csv_train.flush()
#             else:
#                 self.csv_writer_test.writerow(row)
#                 self.csv_test.flush()
#         else:
#             self.csv_writer.writerow(row)
#             self.csv_file.flush()
#
#     # ===== 工具函数 =====
#     @staticmethod
#     def get_yaw_from_quaternion(quat):
#         x, y, z, w = quat.x, quat.y, quat.z, quat.w
#         t3 = +2.0 * (w * z + x * y)
#         t4 = +1.0 - 2.0 * (y * y + z * z)
#         return math.atan2(t3, t4)
#
#     def _save_depth_visualization(self, depth_image, save_path):
#         try:
#             if depth_image.dtype in (np.float32, np.float64):
#                 depth_vis = np.nan_to_num(depth_image, copy=True, nan=0.0, posinf=10.0, neginf=0.0)
#                 depth_vis = np.clip(depth_vis, 0, 10)
#                 depth_vis = (depth_vis / 10.0 * 255).astype(np.uint8)
#             elif depth_image.dtype == np.uint16:
#                 depth_vis = (depth_image / 1000.0 * 255).astype(np.uint8)
#             else:
#                 depth_vis = depth_image.astype(np.uint8)
#             depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
#             cv2.imwrite(save_path, depth_colored)
#         except Exception as e:
#             rospy.logerr(f"深度可视化保存失败: {e}")
#
#     def _visualize_trajectory(self, trajectory, current_pose, goal_pose, save_path):
#         try:
#             width, height, scale = 800, 800, 50
#             origin_x, origin_y = width // 2, height // 2
#             img = np.full((height, width, 3), 255, dtype=np.uint8)
#
#             if trajectory and len(trajectory) > 1:
#                 pts = []
#                 for p in trajectory:
#                     px = int(origin_x + p['x'] * scale)
#                     py = int(origin_y - p['y'] * scale)
#                     pts.append((px, py))
#                 for i in range(len(pts) - 1):
#                     cv2.line(img, pts[i], pts[i + 1], (0, 0, 255), 2)
#
#             if current_pose:
#                 cx = int(origin_x + current_pose['x'] * scale)
#                 cy = int(origin_y - current_pose['y'] * scale)
#                 cv2.circle(img, (cx, cy), 8, (0, 200, 0), -1)
#
#             if goal_pose:
#                 gx = int(origin_x + goal_pose['x'] * scale)
#                 gy = int(origin_y - goal_pose['y'] * scale)
#                 cv2.circle(img, (gx, gy), 8, (255, 0, 0), -1)
#
#             cv2.imwrite(save_path, img)
#         except Exception as e:
#             rospy.logwarn(f"轨迹可视化失败: {e}")
#
#     def shutdown(self):
#         try:
#             # 关闭 CSV
#             if self.enable_split:
#                 if hasattr(self, 'csv_train'):
#                     self.csv_train.close()
#                 if hasattr(self, 'csv_test'):
#                     self.csv_test.close()
#             else:
#                 if hasattr(self, 'csv_file'):
#                     self.csv_file.close()
#         finally:
#             # 统计文件数量（分别统计）
#             def count_dir(root):
#                 counts = {}
#                 for d in self.subdirs:
#                     path = os.path.join(root, d)
#                     if not os.path.exists(path):
#                         counts[d] = 0
#                         continue
#                     if d in ('depth', 'lidar'):
#                         ext = '.npy'
#                     elif d in ['trajectory', 'imu', 'actions']:
#                         ext = '.json'
#                     else:
#                         ext = '.png'
#                     counts[d] = len([f for f in os.listdir(path) if f.endswith(ext)])
#                 return counts
#
#             if self.enable_split:
#                 rospy.loginfo("最终保存统计（train）：%s", count_dir(os.path.join(self.save_dir, "train")))
#                 rospy.loginfo("最终保存统计（test ）：%s", count_dir(os.path.join(self.save_dir, "test")))
#             else:
#                 rospy.loginfo("最终保存统计（all） ：%s", count_dir(self.save_dir))
#
#     def run(self):
#         rospy.on_shutdown(self.shutdown)
#         rospy.spin()
#
#
# if __name__ == "__main__":
#     try:
#         EnhancedNavigationDataCollector().run()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("节点被中断")



# import rospy
# import json
# import csv
# import os
# import cv2
# import numpy as np
# from collections import deque
# from geometry_msgs.msg import PoseStamped, Twist
# from sensor_msgs.msg import Image, CameraInfo, LaserScan, Imu
# from nav_msgs.msg import Path, Odometry
# import message_filters as mf
# from cv_bridge import CvBridge
# import math
# import traceback
# from tf.transformations import euler_from_quaternion
#
#
# class EnhancedNavigationDataCollector:
#     def __init__(self):
#         rospy.init_node('enhanced_navigation_data_collector')
#
#         # ===== 保存目录 =====
#         self.save_dir = rospy.get_param('~save_dir', "/media/gr-agv-x9xy/backup_xxy/camera_data")
#         os.makedirs(os.path.join(self.save_dir, "rgb"), exist_ok=True)
#         os.makedirs(os.path.join(self.save_dir, "depth"), exist_ok=True)
#         os.makedirs(os.path.join(self.save_dir, "depth_vis"), exist_ok=True)
#         os.makedirs(os.path.join(self.save_dir, "trajectory"), exist_ok=True)
#         os.makedirs(os.path.join(self.save_dir, "trajectory_vis"), exist_ok=True)
#         os.makedirs(os.path.join(self.save_dir, "lidar"), exist_ok=True)
#         os.makedirs(os.path.join(self.save_dir, "imu"), exist_ok=True)
#         os.makedirs(os.path.join(self.save_dir, "actions"), exist_ok=True)
#
#         # ===== 采样控制 =====
#         self.counter = 0
#         self.max_count = rospy.get_param('~max_count', 100000)
#         self.last_save_time = 0.0
#         self.save_interval = rospy.get_param('~save_interval', 0.5)
#         self.slop = rospy.get_param('~sync_slop', 0.05)
#
#         # ===== 数据缓冲区 =====
#         self.trajectory_history = deque(maxlen=rospy.get_param('~traj_history_len', 1000))
#         self.trajectory_length = rospy.get_param('~traj_dump_len', 50)
#         self.action_history = deque(maxlen=100)  # 保存最近动作
#         self.latest_imu = None
#         self.latest_lidar = None
#         self.latest_odom = None
#
#         # ===== 订阅所有传感器数据 =====
#         # 核心同步数据
#         rgb_sub = mf.Subscriber('/rgb_left', Image)
#         depth_sub = mf.Subscriber('/depth_left', Image)
#         odom_sub = mf.Subscriber('/odom', Odometry)
#
#         # 新增的同步数据
#         lidar_sub = mf.Subscriber('/scan', LaserScan)  # LiDAR数据
#         imu_sub = mf.Subscriber('/imu', Imu)  # IMU数据
#
#         # 时间同步器（现在包含5个数据源）
#         self.ts = mf.ApproximateTimeSynchronizer(
#             [rgb_sub, depth_sub, odom_sub, lidar_sub, imu_sub],
#             queue_size=20,
#             slop=self.slop
#         )
#         self.ts.registerCallback(self.sync_callback)
#
#         # 非同步数据（动作命令）
#         rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=10)
#
#         # 导航相关数据
#         rospy.Subscriber('/move_base_simple/goal', PoseStamped, self._goal_callback, queue_size=10)
#         rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.trajectory_callback, queue_size=10)
#         rospy.Subscriber('/camera_info_left', CameraInfo, self._camera_info_callback, queue_size=1)
#
#         self.bridge = CvBridge()
#         self.current_goal_msg = None
#         self.current_episode_id = None
#         self.planned_trajectory = None
#         self.camera_info = None
#
#         # ===== 增强的CSV文件 =====
#         self.csv_path = os.path.join(self.save_dir, "enhanced_navigation_data.csv")
#         self.csv_file = open(self.csv_path, "w", newline="")
#         self.csv_writer = csv.writer(self.csv_file)
#         self.csv_writer.writerow([
#             # 基本索引
#             'episode_id', 'stamp', 'odom_stamp', 'goal_stamp',
#             # 文件路径
#             'rgb_path', 'depth_path', 'depth_vis_path',
#             'trajectory_path', 'trajectory_vis_path',
#             'lidar_path', 'imu_path', 'action_path',
#             # 相机内参
#             'fx', 'fy', 'cx', 'cy',
#             # 机器人状态
#             'robot_x', 'robot_y', 'robot_theta',
#             'velocity_x', 'velocity_y', 'velocity_theta',
#             # IMU数据
#             'imu_linear_accel_x', 'imu_linear_accel_y', 'imu_linear_accel_z',
#             'imu_angular_vel_x', 'imu_angular_vel_y', 'imu_angular_vel_z',
#             'imu_orientation_x', 'imu_orientation_y', 'imu_orientation_z', 'imu_orientation_w',
#             # LiDAR元数据
#             'lidar_range_min', 'lidar_range_max', 'lidar_angle_min', 'lidar_angle_max',
#             'lidar_angle_increment', 'lidar_num_ranges',
#             # 动作数据
#             'cmd_vel_linear_x', 'cmd_vel_linear_y', 'cmd_vel_angular_z',
#             # 目标与规划
#             'goal_x', 'goal_y', 'goal_theta',
#             'trajectory_length', 'has_planned_trajectory', 'plan_length'
#         ])
#
#         rospy.loginfo("导航数据收集器已启动，包含ODOM、LiDAR、IMU、Action数据")
#
#     # ===== 回调函数 =====
#     def _camera_info_callback(self, msg: CameraInfo):
#         self.camera_info = msg
#
#     def _goal_callback(self, msg: PoseStamped):
#         self.current_goal_msg = msg
#         self.current_episode_id = msg.header.stamp.to_nsec()
#
#     def trajectory_callback(self, msg: Path):
#         try:
#             if msg.poses:
#                 self.planned_trajectory = [{
#                     'x': p.pose.position.x,
#                     'y': p.pose.position.y,
#                     'z': p.pose.position.z
#                 } for p in msg.poses]
#         except Exception as e:
#             rospy.logwarn(f"轨迹处理错误: {e}")
#
#     def cmd_vel_callback(self, msg: Twist):
#         """保存最新的动作命令"""
#         action_data = {
#             'linear_x': msg.linear.x,
#             'linear_y': msg.linear.y,
#             'angular_z': msg.angular.z,
#             'timestamp': rospy.Time.now().to_nsec()
#         }
#         self.action_history.append(action_data)
#
#     # ===== 核心同步回调 =====
#     def sync_callback(self, rgb_msg: Image, depth_msg: Image, odom_msg: Odometry,
#                       lidar_msg: LaserScan, imu_msg: Imu):
#         try:
#             # 节流控制
#             now = rospy.Time.now().to_sec()
#             if now - self.last_save_time < self.save_interval:
#                 return
#             if self.counter >= self.max_count:
#                 rospy.loginfo("达到最大保存数量，停止采集")
#                 return
#
#             # 保存最新数据到缓冲区
#             self.latest_odom = odom_msg
#             self.latest_lidar = lidar_msg
#             self.latest_imu = imu_msg
#
#             # 处理机器人位姿
#             robot_pose = {
#                 'x': odom_msg.pose.pose.position.x,
#                 'y': odom_msg.pose.pose.position.y,
#                 'theta': self.get_yaw_from_quaternion(odom_msg.pose.pose.orientation),
#                 'timestamp': odom_msg.header.stamp.to_nsec()
#             }
#             self.trajectory_history.append(robot_pose)
#
#             # 图像转换
#             rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
#             depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
#
#             # 获取最近轨迹
#             recent_traj = list(self.trajectory_history)[-self.trajectory_length:]
#
#             # 处理目标信息
#             goal_pose = None
#             goal_stamp_ns = 0
#             if self.current_goal_msg is not None:
#                 goal_pose = {
#                     'x': self.current_goal_msg.pose.position.x,
#                     'y': self.current_goal_msg.pose.position.y,
#                     'theta': self.get_yaw_from_quaternion(self.current_goal_msg.pose.orientation)
#                 }
#                 goal_stamp_ns = self.current_goal_msg.header.stamp.to_nsec()
#
#             # 保存所有数据
#             stamp_ns = rgb_msg.header.stamp.to_nsec()
#             self._save_enhanced_data(
#                 stamp_ns=stamp_ns,
#                 odom_stamp_ns=odom_msg.header.stamp.to_nsec(),
#                 goal_stamp_ns=goal_stamp_ns,
#                 episode_id=self.current_episode_id,
#                 rgb_image=rgb_image,
#                 depth_image=depth_image,
#                 robot_pose=robot_pose,
#                 goal_pose=goal_pose,
#                 recent_trajectory=recent_traj,
#                 odom_msg=odom_msg,
#                 lidar_msg=lidar_msg,
#                 imu_msg=imu_msg
#             )
#
#             self.last_save_time = now
#             self.counter += 1
#
#             if self.counter % 100 == 0:
#                 rospy.loginfo(f"已保存 {self.counter} 个数据样本")
#
#         except Exception as e:
#             rospy.logerr(f"同步回调错误: {e}")
#             traceback.print_exc()
#
#     # ===== 增强的数据保存 =====
#     def _save_enhanced_data(self, stamp_ns, odom_stamp_ns, goal_stamp_ns, episode_id,
#                             rgb_image, depth_image, robot_pose, goal_pose, recent_trajectory,
#                             odom_msg, lidar_msg, imu_msg):
#         try:
#             # 文件名生成
#             rgb_filename = f"rgb_{stamp_ns}.png"
#             depth_filename = f"depth_{stamp_ns}.npy"
#             depth_vis_filename = f"depth_vis_{stamp_ns}.png"
#             traj_filename = f"trajectory_{stamp_ns}.json"
#             traj_vis_filename = f"trajectory_vis_{stamp_ns}.png"
#             lidar_filename = f"lidar_{stamp_ns}.npy"
#             imu_filename = f"imu_{stamp_ns}.json"
#             action_filename = f"action_{stamp_ns}.json"
#
#             # 文件路径
#             base_paths = {
#                 'rgb': os.path.join(self.save_dir, "rgb", rgb_filename),
#                 'depth': os.path.join(self.save_dir, "depth", depth_filename),
#                 'depth_vis': os.path.join(self.save_dir, "depth_vis", depth_vis_filename),
#                 'trajectory': os.path.join(self.save_dir, "trajectory", traj_filename),
#                 'trajectory_vis': os.path.join(self.save_dir, "trajectory_vis", traj_vis_filename),
#                 'lidar': os.path.join(self.save_dir, "lidar", lidar_filename),
#                 'imu': os.path.join(self.save_dir, "imu", imu_filename),
#                 'action': os.path.join(self.save_dir, "actions", action_filename)
#             }
#
#             # 保存图像数据
#             cv2.imwrite(base_paths['rgb'], rgb_image)
#             np.save(base_paths['depth'], depth_image)
#             self._save_depth_visualization(depth_image, base_paths['depth_vis'])
#
#             # 保存LiDAR数据
#             lidar_ranges = np.array(lidar_msg.ranges, dtype=np.float32)
#             np.save(base_paths['lidar'], lidar_ranges)
#
#             # 保存IMU数据
#             imu_data = {
#                 'linear_acceleration': {
#                     'x': imu_msg.linear_acceleration.x,
#                     'y': imu_msg.linear_acceleration.y,
#                     'z': imu_msg.linear_acceleration.z
#                 },
#                 'angular_velocity': {
#                     'x': imu_msg.angular_velocity.x,
#                     'y': imu_msg.angular_velocity.y,
#                     'z': imu_msg.angular_velocity.z
#                 },
#                 'orientation': {
#                     'x': imu_msg.orientation.x,
#                     'y': imu_msg.orientation.y,
#                     'z': imu_msg.orientation.z,
#                     'w': imu_msg.orientation.w
#                 },
#                 'timestamp': stamp_ns
#             }
#             with open(base_paths['imu'], 'w') as f:
#                 json.dump(imu_data, f, indent=2)
#
#             # 保存动作数据（最近的动作命令）
#             recent_actions = list(self.action_history)[-10:]  # 最近10个动作
#             action_data = {
#                 'current_action': {
#                     'linear_x': odom_msg.twist.twist.linear.x,
#                     'linear_y': odom_msg.twist.twist.linear.y,
#                     'angular_z': odom_msg.twist.twist.angular.z
#                 },
#                 'recent_actions': recent_actions,
#                 'timestamp': stamp_ns
#             }
#             with open(base_paths['action'], 'w') as f:
#                 json.dump(action_data, f, indent=2)
#
#             # 保存轨迹数据
#             traj_data = {
#                 'episode_id': episode_id,
#                 'stamp_ns': stamp_ns,
#                 'recent_trajectory': recent_trajectory,
#                 'planned_trajectory': self.planned_trajectory,
#                 'current_pose': robot_pose,
#                 'goal_pose': goal_pose,
#                 'velocity': {
#                     'linear_x': odom_msg.twist.twist.linear.x,
#                     'linear_y': odom_msg.twist.twist.linear.y,
#                     'angular_z': odom_msg.twist.twist.angular.z
#                 },
#                 'imu_data': imu_data,
#                 'lidar_metadata': {
#                     'range_min': lidar_msg.range_min,
#                     'range_max': lidar_msg.range_max,
#                     'angle_min': lidar_msg.angle_min,
#                     'angle_max': lidar_msg.angle_max,
#                     'angle_increment': lidar_msg.angle_increment,
#                     'num_ranges': len(lidar_msg.ranges)
#                 }
#             }
#             with open(base_paths['trajectory'], 'w') as f:
#                 json.dump(traj_data, f, indent=2)
#
#             # 轨迹可视化
#             self._visualize_trajectory(recent_trajectory, robot_pose, goal_pose, base_paths['trajectory_vis'])
#
#             # 更新CSV索引
#             self._update_csv_index(
#                 stamp_ns, odom_stamp_ns, goal_stamp_ns, episode_id,
#                 base_paths, robot_pose, odom_msg, imu_msg, lidar_msg,
#                 goal_pose, recent_trajectory
#             )
#
#         except Exception as e:
#             rospy.logerr(f"保存数据失败: {e}")
#             traceback.print_exc()
#
#     def _update_csv_index(self, stamp_ns, odom_stamp_ns, goal_stamp_ns, episode_id,
#                           base_paths, robot_pose, odom_msg, imu_msg, lidar_msg,
#                           goal_pose, recent_trajectory):
#         """更新CSV索引文件"""
#         # 相机内参
#         if self.camera_info:
#             fx, fy, cx, cy = self.camera_info.K[0], self.camera_info.K[4], self.camera_info.K[2], self.camera_info.K[5]
#         else:
#             fx = fy = cx = cy = 0.0
#
#         plan_len = len(self.planned_trajectory) if self.planned_trajectory else 0
#         has_plan = 1 if plan_len > 0 else 0
#
#         # 获取最近的动作命令
#         current_action = list(self.action_history)[-1] if self.action_history else {'linear_x': 0, 'linear_y': 0,
#                                                                                     'angular_z': 0}
#
#         # 写入CSV行
#         self.csv_writer.writerow([
#             # 基本索引
#             episode_id if episode_id is not None else 0,
#             stamp_ns,
#             odom_stamp_ns,
#             goal_stamp_ns,
#             # 文件路径（相对路径）
#             f"rgb/{os.path.basename(base_paths['rgb'])}",
#             f"depth/{os.path.basename(base_paths['depth'])}",
#             f"depth_vis/{os.path.basename(base_paths['depth_vis'])}",
#             f"trajectory/{os.path.basename(base_paths['trajectory'])}",
#             f"trajectory_vis/{os.path.basename(base_paths['trajectory_vis'])}",
#             f"lidar/{os.path.basename(base_paths['lidar'])}",
#             f"imu/{os.path.basename(base_paths['imu'])}",
#             f"actions/{os.path.basename(base_paths['action'])}",
#             # 相机内参
#             fx, fy, cx, cy,
#             # 机器人状态
#             robot_pose['x'], robot_pose['y'], robot_pose['theta'],
#             odom_msg.twist.twist.linear.x,
#             odom_msg.twist.twist.linear.y,
#             odom_msg.twist.twist.angular.z,
#             # IMU数据
#             imu_msg.linear_acceleration.x,
#             imu_msg.linear_acceleration.y,
#             imu_msg.linear_acceleration.z,
#             imu_msg.angular_velocity.x,
#             imu_msg.angular_velocity.y,
#             imu_msg.angular_velocity.z,
#             imu_msg.orientation.x,
#             imu_msg.orientation.y,
#             imu_msg.orientation.z,
#             imu_msg.orientation.w,
#             # LiDAR元数据
#             lidar_msg.range_min,
#             lidar_msg.range_max,
#             lidar_msg.angle_min,
#             lidar_msg.angle_max,
#             lidar_msg.angle_increment,
#             len(lidar_msg.ranges),
#             # 动作数据
#             current_action['linear_x'],
#             current_action['linear_y'],
#             current_action['angular_z'],
#             # 目标与规划
#             (goal_pose['x'] if goal_pose else 0.0),
#             (goal_pose['y'] if goal_pose else 0.0),
#             (goal_pose['theta'] if goal_pose else 0.0),
#             len(recent_trajectory),
#             has_plan,
#             plan_len
#         ])
#         self.csv_file.flush()
#
#     # ===== 工具函数 =====
#     @staticmethod
#     def get_yaw_from_quaternion(quat):
#         x, y, z, w = quat.x, quat.y, quat.z, quat.w
#         t3 = +2.0 * (w * z + x * y)
#         t4 = +1.0 - 2.0 * (y * y + z * z)
#         return math.atan2(t3, t4)
#
#     def _save_depth_visualization(self, depth_image, save_path):
#         try:
#             if depth_image.dtype == np.float32:
#                 depth_vis = np.clip(depth_image, 0, 10)
#                 depth_vis = (depth_vis / 10.0 * 255).astype(np.uint8)
#             elif depth_image.dtype == np.uint16:
#                 depth_vis = (depth_image / 1000.0 * 255).astype(np.uint8)
#             else:
#                 depth_vis = depth_image.astype(np.uint8)
#             depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
#             cv2.imwrite(save_path, depth_colored)
#         except Exception as e:
#             rospy.logerr(f"深度可视化保存失败: {e}")
#
#     def _visualize_trajectory(self, trajectory, current_pose, goal_pose, save_path):
#         try:
#             width, height, scale = 800, 800, 50
#             origin_x, origin_y = width // 2, height // 2
#             img = np.full((height, width, 3), 255, dtype=np.uint8)
#
#             if trajectory and len(trajectory) > 1:
#                 pts = []
#                 for p in trajectory:
#                     px = int(origin_x + p['x'] * scale)
#                     py = int(origin_y - p['y'] * scale)
#                     pts.append((px, py))
#                 for i in range(len(pts) - 1):
#                     cv2.line(img, pts[i], pts[i + 1], (0, 0, 255), 2)
#
#             if current_pose:
#                 cx = int(origin_x + current_pose['x'] * scale)
#                 cy = int(origin_y - current_pose['y'] * scale)
#                 cv2.circle(img, (cx, cy), 8, (0, 200, 0), -1)
#
#             if goal_pose:
#                 gx = int(origin_x + goal_pose['x'] * scale)
#                 gy = int(origin_y - goal_pose['y'] * scale)
#                 cv2.circle(img, (gx, gy), 8, (255, 0, 0), -1)
#
#             cv2.imwrite(save_path, img)
#         except Exception as e:
#             rospy.logwarn(f"轨迹可视化失败: {e}")
#
#     def shutdown(self):
#         try:
#             if hasattr(self, 'csv_file'):
#                 self.csv_file.close()
#         finally:
#             # 统计文件数量
#             directories = ['rgb', 'depth', 'depth_vis', 'trajectory', 'trajectory_vis', 'lidar', 'imu', 'actions']
#             counts = {}
#             for d in directories:
#                 path = os.path.join(self.save_dir, d)
#                 if not os.path.exists(path):
#                     counts[d] = 0
#                     continue
#                 if d == 'depth':
#                     ext = '.npy'
#                 elif d == 'lidar':
#                     ext = '.npy'
#                 elif d in ['trajectory', 'imu', 'actions']:
#                     ext = '.json'
#                 else:
#                     ext = '.png'
#                 counts[d] = len([f for f in os.listdir(path) if f.endswith(ext)])
#
#             rospy.loginfo("最终保存统计:")
#             for d, count in counts.items():
#                 rospy.loginfo(f"  {d}: {count}")
#
#     def run(self):
#         rospy.on_shutdown(self.shutdown)
#         rospy.spin()
#
#
# if __name__ == "__main__":
#     try:
#         EnhancedNavigationDataCollector().run()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("节点被中断")


