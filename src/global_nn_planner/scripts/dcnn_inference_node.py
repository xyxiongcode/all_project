#!/usr/bin/env python3
import rospy
import torch
import torch.nn as nn
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path
from model import DCNN

from torchvision import transforms
import torch.nn.functional as F

class DCNNPredictor:
    def __init__(self, model_path):
        self.model = DCNN(input_channels=2, num_actions=8)
        self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
        self.model.eval()

        self.imsize = 64
        self.goal = None
        self.scan_data = None

        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.pub = rospy.Publisher("/predicted_path", Path, queue_size=1)

    def scan_callback(self, msg):
        if self.goal is None:
            rospy.logwarn("[DCNN] Goal is None, skipping scan.")
            return
        rospy.loginfo("[DCNN] Received scan, processing input...")
        scan = np.array(msg.ranges)
        scan = np.clip(scan, 0, msg.range_max)
        scan = scan / msg.range_max
        scan_line = scan[:self.imsize]
        obstacle_map = np.tile(scan_line.reshape((self.imsize, 1)), (1, self.imsize)).T

        goal_map = np.zeros((self.imsize, self.imsize), dtype=np.float32)
        gx, gy = int(self.goal[0] * self.imsize), int(self.goal[1] * self.imsize)
        gx = np.clip(gx, 0, self.imsize - 1)
        gy = np.clip(gy, 0, self.imsize - 1)
        goal_map[gx, gy] = 1.0

        input_tensor = np.stack([obstacle_map, goal_map], axis=0)
        input_tensor = torch.tensor(input_tensor, dtype=torch.float32).unsqueeze(0)
        print(f"[DCNN] Input tensor shape: {input_tensor.shape}")

        with torch.no_grad():
            output = self.model(input_tensor)
            print(f"[DCNN] Output tensor shape: {output.shape}")
            if output.ndim == 3:
                path_points = output[0].numpy()  # shape: (T, 2)
            else:
                path_points = np.zeros((0, 2))

        self.publish_path(path_points, msg.header)

    def goal_callback(self, msg):
        # Normalize goal to [0, 1] range (you can improve this with actual world-to-map logic)
        self.goal = [msg.pose.position.x / 10.0, msg.pose.position.y / 10.0]  # Assume world max 10x10m
        rospy.loginfo(f"[DCNN] Received goal: {self.goal}")

    def publish_path(self, points, header):
        print(f"[DCNN] Publishingn] path with {len(points)} points.")
        path = Path()
        path.header = Header()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = header.frame_id

        for pt in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            path.poses.append(pose)

        self.pub.publish(path)

if __name__ == '__main__':
    rospy.init_node("dcnn_predictor")
    model_path =  "/home/gr-agv-x9xy/isaac_sim_ws/src/global_nn_planner/scripts/runs/DCNN_20250616-162117/DCNN_final.pth"  # 你的模型路径
    predictor = DCNNPredictor(model_path)
    rospy.spin()


