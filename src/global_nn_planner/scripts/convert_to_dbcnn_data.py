import os
import json
import numpy as np
import joblib
from tqdm import tqdm

# === CONFIG ===
dataset_root = "/home/gr-agv-x9xy/Downloads/pretraining_dataset"
scenes = ["hospital", "office"]
imsize = 64  # DB-CNN 图像尺寸
output_path = "/home/gr-agv-x9xy/Downloads/pretraining_dataset/db_cnn_data/data_64.pkl"

# 地图参数（从 rostopic echo /map 获取）
origin_x = -24.025  # 地图左下角x坐标
origin_y = -33.825  # 地图左下角y坐标
resolution = 0.05   # 每像素米数（地图分辨率）

# === HELPER FUNCTIONS ===
def world_to_map(x, y, origin_x, origin_y, resolution, imsize):
    mx = int((x - origin_x) / resolution)
    my = int((y - origin_y) / resolution)
    # 缩放到 imsize x imsize 范围内
    mx = mx * imsize // 603  # 603 是你地图的 width
    my = my * imsize // 1879 # 1879 是你地图的 height
    mx = np.clip(mx, 0, imsize - 1)
    my = np.clip(my, 0, imsize - 1)
    return mx, my

def create_goal_map(gx, gy, imsize):
    goal_map = np.zeros((imsize, imsize), dtype=np.float32)
    if 0 <= gx < imsize and 0 <= gy < imsize:
        goal_map[gx, gy] = 1.0
    return goal_map

def normalize_scan(scan, max_range=10.0):
    return np.clip(np.array(scan) / max_range, 0.0, 1.0)

def compute_label(start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    direction = (np.sign(dx), np.sign(dy))
    dir_map = {
        (-1, 0): 0,  # up
        (1, 0): 1,   # down
        (0, 1): 2,   # right
        (0, -1): 3,  # left
        (-1, 1): 4,  # up-right
        (-1, -1): 5, # up-left
        (1, 1): 6,   # down-right
        (1, -1): 7,  # down-left
    }
    return dir_map.get(direction, 0)

# === MAIN SCRIPT ===
images, coords, labels = [], [], []

for scene in scenes:
    scene_path = os.path.join(dataset_root, scene)
    dataset_info_path = os.path.join(scene_path, "dataset_info.json")
    if not os.path.exists(dataset_info_path):
        continue

    with open(dataset_info_path, "r") as f:
        scene_data = json.load(f)

    for traj_key, traj_data in tqdm(scene_data.items(), desc=f"Processing {scene}"):
        data_points = traj_data.get("data", [])
        for i in range(len(data_points) - 1):
            step = data_points[i]
            next_step = data_points[i + 1]

            # Read laser scan
            laser_path = step["laser_path"]
            if not os.path.exists(laser_path):
                continue
            scan = normalize_scan(np.load(laser_path))

            if len(scan) < imsize:
                continue
            scan = scan[:imsize]
            obstacle_map = np.tile(scan.reshape((imsize, 1)), (1, imsize)).T

            # Goal map
            gx, gy = world_to_map(step["target_x"], step["target_y"], origin_x, origin_y, resolution, imsize)
            goal_map = create_goal_map(gx, gy, imsize)

            # Image = (obstacle, goal) channel
            image = np.stack([obstacle_map, goal_map], axis=-1)
            images.append(image)
            coords.append([gx, gy])

            # Motion label
            label = compute_label(
                world_to_map(step["target_x"], step["target_y"], origin_x, origin_y, resolution, imsize),
                world_to_map(next_step["target_x"], next_step["target_y"], origin_x, origin_y, resolution, imsize)
            )
            labels.append(label)

# === SAVE ===
print(f"\n✅ Saving {len(images)} samples to {output_path}")
os.makedirs(os.path.dirname(output_path), exist_ok=True)
joblib.dump((images, None, coords, labels), output_path)
print("✅ Done.")
