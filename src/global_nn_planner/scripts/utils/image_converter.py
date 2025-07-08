
import numpy as np
import cv2
import rospy

def scan_to_image(scan_data, goal_xy, imsize=64):
    # 将 1D scan 转为 2D 障碍图（模拟）
    scan = np.array(scan_data.ranges)
    scan = np.clip(scan, 0, 10.0)
    scan = scan[:imsize]  # 取前 imsize 项
    obstacle_map = np.tile(scan.reshape((imsize, 1)), (1, imsize)).T

    goal_map = np.zeros((imsize, imsize), dtype=np.float32)
    gx, gy = int(goal_xy[0]), int(goal_xy[1])
    if 0 <= gx < imsize and 0 <= gy < imsize:
        goal_map[gx, gy] = 1.0

    image = np.stack([obstacle_map, goal_map], axis=0)  # [2, 64, 64]
    return image
