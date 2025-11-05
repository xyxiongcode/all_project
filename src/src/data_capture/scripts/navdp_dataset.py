import numpy as np
import os
import json
import cv2
import open3d as o3d
import io
import time
from datetime import datetime
import torch
from tqdm import tqdm
from torch.utils.data import Dataset
from scipy.spatial.transform import Rotation
from scipy.interpolate import CubicSpline
import torchvision.transforms as T
from PIL import Image
from io import BytesIO
from glob import glob

# ---------- timestamped print ----------
import builtins
_original_print = builtins.print
def print(*args, **kwargs):
    try:
        rank = int(os.environ.get('RANK', 0))
        if rank == 0:
            ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            _original_print(f"[{ts}]", *args, **kwargs)
    except Exception:
        pass
builtins.print = print

def safe_load_json(path):
    with open(path, 'r') as f:
        return json.load(f)

def ensure_dir(p): os.makedirs(p, exist_ok=True)

def default_camera_intrinsic(image_size):
    fx = fy = float(image_size)
    cx = cy = float(image_size) / 2.0
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1 ]], dtype=np.float32)
    return K

def identity_extrinsic():
    T = np.eye(4, dtype=np.float32)
    return T

def load_npy_or_json(path):
    if path.endswith(".npy"):
        return np.load(path)
    with open(path, "r") as f:
        return np.array(json.load(f), dtype=np.float32)

def nearest_index(src_ts, ref_ts, max_diff):
    """
    对于每个 ref_ts，找到 src_ts 中最近的索引；若最小差值 > max_diff（秒），返回 -1
    """
    src_ts = np.asarray(src_ts)
    ref_ts = np.asarray(ref_ts)
    idxs = np.searchsorted(src_ts, ref_ts)
    idxs = np.clip(idxs, 1, len(src_ts)-1)
    prev = idxs - 1
    next_ = idxs
    prev_diff = np.abs(ref_ts - src_ts[prev])
    next_diff = np.abs(ref_ts - src_ts[next_])
    choose_prev = prev_diff <= next_diff
    out = np.where(choose_prev, prev, next_)
    bad = np.abs(ref_ts - src_ts[out]) > max_diff
    out[bad] = -1
    return out

def make_bev_from_points(points_xyz, image_size, meters_per_pixel=0.05, z_clip=( -1.0, 2.0 )):
    """
    把点云投影到鸟瞰图：输出 HxWx3 （count / maxZ / meanZ）
    """
    H = W = int(image_size)
    if points_xyz.size == 0:
        return np.zeros((H, W, 3), np.float32)

    # 只取车体附近范围（以 image_size * mpp 为尺度）
    half_w = (W * meters_per_pixel) / 2.0
    half_h = (H * meters_per_pixel) / 2.0
    x = points_xyz[:,0]; y = points_xyz[:,1]; z = points_xyz[:,2]
    # 将 x->右(+), y->前(+)，这里假设数据就是机器人坐标系的平面；若你是 x前 y左, 可交换
    mask = (x >= -half_w) & (x < half_w) & (y >= -half_h) & (y < half_h)
    x = x[mask]; y = y[mask]; z = z[mask]

    # 像素坐标：以图中心为(0,0)
    # y 轴向前：映射到图像的 -y -> 像素向上
    px = (x / meters_per_pixel + W/2.0).astype(np.int32)
    py = (H/2.0 - y / meters_per_pixel).astype(np.int32)
    good = (px>=0)&(px<W)&(py>=0)&(py<H)
    px = px[good]; py = py[good]; z = z[good]
    if px.size == 0:
        return np.zeros((H, W, 3), np.float32)

    # 三个通道：计数/最大高度/平均高度（都归一化到0~1）
    count = np.zeros((H,W), np.float32)
    maxz  = np.full((H,W), -1e6, np.float32)
    sumz  = np.zeros((H,W), np.float32)

    for i in range(px.size):
        count[py[i], px[i]] += 1.0
        if z[i] > maxz[py[i], px[i]]:
            maxz[py[i], px[i]] = z[i]
        sumz[py[i], px[i]] += z[i]

    meanz = np.zeros_like(sumz)
    nz = count > 0
    meanz[nz] = sumz[nz] / np.clip(count[nz], 1, None)

    # 归一化
    c = count
    if c.max() > 0: c = c / (c.max() + 1e-8)
    mz = maxz
    mz[mz<-1e5] = z_clip[0]
    mz = (np.clip(mz, z_clip[0], z_clip[1]) - z_clip[0]) / (z_clip[1]-z_clip[0] + 1e-8)
    az = meanz
    az = (np.clip(az, z_clip[0], z_clip[1]) - z_clip[0]) / (z_clip[1]-z_clip[0] + 1e-8)

    bev = np.stack([c, mz, az], axis=-1)
    return bev.astype(np.float32)

def depth_like_from_points(points_xyz, image_size, meters_per_pixel=0.05, z_clip=(0.0, 5.0)):
    """
    生成单通道“深度风格”图：取最大高度或距离，这里用最大高度
    """
    H = W = int(image_size)
    if points_xyz.size == 0:
        return np.zeros((H, W, 1), np.float32)
    bev = make_bev_from_points(points_xyz, image_size, meters_per_pixel, z_clip=( -1.0, z_clip[1] ))
    # 用第二通道（最大高度）近似深度，裁剪 [0,5] 米
    depth = bev[...,1:2]
    return depth

class NavDP_Base_Datset(Dataset):
    def __init__(self,
                 root_dirs,
                 preload_path,
                 memory_size=16,
                 predict_size=24,
                 batch_size=64,
                 image_size=224,
                 scene_data_scale=1.0,
                 trajectory_data_scale=1.0,
                 debug=False,
                 preload=False,
                 random_digit=False,
                 prior_sample=False,
                 dataset_mode='auto',          # 'auto' | 'navdp_v1' | 'isaac_recorder_v1'
                 max_time_diff=0.05,          # seconds，对齐容差
                 lidar_bev_scale=0.05,        # m/px
                 lidar_accumulate_frames=8    # 构造障碍点时，累计多少帧 LiDAR
                 ):

        self.dataset_dirs = root_dirs
        self.memory_size = memory_size
        self.image_size = image_size
        self.scene_scale_size = scene_data_scale
        self.trajectory_data_scale = trajectory_data_scale
        self.predict_size = predict_size
        self.debug = debug
        self.random_digit = random_digit
        self.prior_sample = prior_sample
        self.item_cnt = 0
        self.batch_size = batch_size
        self.batch_time_sum = 0.0
        self._last_time = None

        self.dataset_mode = dataset_mode
        self.max_time_diff = float(max_time_diff)
        self.lidar_bev_scale = float(lidar_bev_scale)
        self.lidar_accumulate_frames = int(lidar_accumulate_frames)

        # 统一的“索引缓存”
        self.trajectory_dirs = []
        self.trajectory_meta = []          # 每条轨迹的元数据 dict
        self.trajectory_rgb_path = []      # list[list[str]] or empty
        self.trajectory_depth_path = []    # list[list[str]] or empty
        self.trajectory_data_dir = []      # data.json (navdp) 或 poses/odom 路径(isaac)
        self.trajectory_afford_path = []   # path.ply（navdp）或 global_path.(npy|json)

        if not preload:
            self._build_index(preload_path)
        else:
            self._load_index(preload_path)

    # ---------- index building ----------
    def _detect_mode(self, traj_dir):
        if self.dataset_mode != 'auto':
            return self.dataset_mode
        # 简单探测：存在 lidar/ 或 poses/odom 则 isaac_recorder_v1
        cand = [os.path.join(traj_dir, "lidar"),
                os.path.join(traj_dir, "poses.npy"),
                os.path.join(traj_dir, "odom.npy")]
        if any(os.path.exists(p) for p in cand):
            return 'isaac_recorder_v1'
        return 'navdp_v1'

    def _build_index(self, preload_path):
        for group_dir in self.dataset_dirs:
            all_scene_dirs = np.array(sorted([p for p in os.listdir(group_dir)]))
            select_scene_dirs = all_scene_dirs[np.arange(0, all_scene_dirs.shape[0], max(1, int(1/self.scene_scale_size))).astype(np.int32)]
            for scene_dir in select_scene_dirs:
                scene_path = os.path.join(group_dir, scene_dir)
                all_traj_dirs = np.array(sorted([p for p in os.listdir(scene_path)]))
                select_traj_dirs = all_traj_dirs[np.arange(0, all_traj_dirs.shape[0], max(1, int(1/self.trajectory_data_scale))).astype(np.int32)]
                for traj_dir in tqdm(select_traj_dirs):
                    entire_task_dir = os.path.join(scene_path, traj_dir)
                    mode = self._detect_mode(entire_task_dir)
                    if mode == 'navdp_v1':
                        ok = self._index_navdp(entire_task_dir)
                    else:
                        ok = self._index_isaac(entire_task_dir)
                    if not ok and self.debug:
                        print(f"Skip (no valid data): {entire_task_dir}")

        save_dict = {
            'trajectory_dirs': self.trajectory_dirs,
            'trajectory_meta': self.trajectory_meta,
            'trajectory_rgb_path': self.trajectory_rgb_path,
            'trajectory_depth_path': self.trajectory_depth_path,
            'trajectory_data_dir': self.trajectory_data_dir,
            'trajectory_afford_path': self.trajectory_afford_path
        }
        ensure_dir(os.path.dirname(preload_path))
        with open(preload_path, 'w') as f:
            json.dump(save_dict, f, indent=2)

    def _load_index(self, preload_path):
        load_dict = json.load(open(preload_path, 'r'))
        # 为了增大 epoch 数据，多倍展开（保持你原有 *50 的思路）
        self.trajectory_dirs = load_dict['trajectory_dirs'] * 50
        self.trajectory_meta = load_dict['trajectory_meta'] * 50
        self.trajectory_rgb_path = load_dict['trajectory_rgb_path'] * 50
        self.trajectory_depth_path = load_dict['trajectory_depth_path'] * 50
        self.trajectory_data_dir = load_dict['trajectory_data_dir'] * 50
        self.trajectory_afford_path = load_dict['trajectory_afford_path'] * 50

    # ---------- index helpers ----------
    def _index_navdp(self, task_dir):
        rgb_dir   = os.path.join(task_dir, "rgb")
        depth_dir = os.path.join(task_dir, "depth")
        data_path = os.path.join(task_dir, "data.json")
        afford    = os.path.join(task_dir, "path.ply")

        if not (os.path.isdir(rgb_dir) and os.path.isdir(depth_dir) and os.path.exists(data_path) and os.path.exists(afford)):
            return False
        rgbs = sorted([os.path.join(rgb_dir, f) for f in os.listdir(rgb_dir) if f.endswith(".jpg")])
        deps = sorted([os.path.join(depth_dir, f) for f in os.listdir(depth_dir) if f.endswith(".png")])
        if len(rgbs) == 0 or len(rgbs) != len(deps):
            return False

        self.trajectory_dirs.append(task_dir)
        self.trajectory_meta.append({
            'mode': 'navdp_v1',
            'has_rgb': True,
            'has_depth': True,
            'has_lidar': False,
            'has_intrinsic': True,
            'has_extrinsic': True
        })
        self.trajectory_rgb_path.append(rgbs)
        self.trajectory_depth_path.append(deps)
        self.trajectory_data_dir.append(data_path)      # data.json
        self.trajectory_afford_path.append(afford)      # path.ply
        return True

    def _index_isaac(self, task_dir):
        """
        兼容：lidar/*.npy, poses.npy|odom.npy, (optional) rgb/*.jpg, depth/*.png, global_path.(npy|json), obstacles.(npy|json)
        """
        lidar_dir = os.path.join(task_dir, "lidar")
        poses_npy = os.path.join(task_dir, "poses.npy")
        odom_npy  = os.path.join(task_dir, "odom.npy")
        rgb_dir   = os.path.join(task_dir, "rgb")
        depth_dir = os.path.join(task_dir, "depth")

        if not os.path.isdir(lidar_dir):
            return False
        pose_path = poses_npy if os.path.exists(poses_npy) else (odom_npy if os.path.exists(odom_npy) else None)
        if pose_path is None:
            return False

        lidar_files = sorted(glob(os.path.join(lidar_dir, "*.npy")))
        if len(lidar_files) == 0:
            return False

        rgbs = sorted(glob(os.path.join(rgb_dir, "*.jpg"))) if os.path.isdir(rgb_dir) else []
        deps = sorted(glob(os.path.join(depth_dir, "*.png"))) if os.path.isdir(depth_dir) else []

        # 路径/障碍
        global_path = None
        for cand in ["global_path.npy", "global_path.json"]:
            p = os.path.join(task_dir, cand)
            if os.path.exists(p): global_path = p; break
        obstacles_path = None
        for cand in ["obstacles.npy", "obstacles.json"]:
            p = os.path.join(task_dir, cand)
            if os.path.exists(p): obstacles_path = p; break

        self.trajectory_dirs.append(task_dir)
        self.trajectory_meta.append({
            'mode': 'isaac_recorder_v1',
            'has_rgb': len(rgbs) > 0,
            'has_depth': len(deps) > 0,
            'has_lidar': True,
            'pose_path': pose_path,
            'lidar_files': lidar_files,
            'rgb_files': rgbs,
            'depth_files': deps,
            'global_path': global_path,
            'obstacles_path': obstacles_path,
            # 可选相机/基座内外参
            'camera_intrinsic': os.path.join(task_dir, "camera_intrinsic.json") if os.path.exists(os.path.join(task_dir, "camera_intrinsic.json")) else None,
            'camera_extrinsic': os.path.join(task_dir, "camera_extrinsic.json") if os.path.exists(os.path.join(task_dir, "camera_extrinsic.json")) else None,
            'base_extrinsic':   os.path.join(task_dir, "base_extrinsic.json")   if os.path.exists(os.path.join(task_dir, "base_extrinsic.json"))   else None
        })
        self.trajectory_rgb_path.append(rgbs)    # 允许为空
        self.trajectory_depth_path.append(deps)  # 允许为空
        self.trajectory_data_dir.append(pose_path)
        self.trajectory_afford_path.append(global_path if global_path else "")
        return True

    def __len__(self):
        return len(self.trajectory_dirs)

    # ---------- modality loaders ----------
    def load_image(self, image_url):
        image = Image.open(image_url).convert("RGB")
        return np.array(image, np.uint8)

    def load_depth(self, depth_url):
        depth = Image.open(depth_url)
        return np.array(depth, np.uint16)

    def load_pointcloud(self, pcd_url):
        return o3d.io.read_point_cloud(pcd_url)

    def process_image(self, image_path):
        image = self.load_image(image_path)
        H, W, C = image.shape
        prop = self.image_size / max(H, W)
        image = cv2.resize(image, (int(W*prop), int(H*prop)))
        pad_w = max((self.image_size - image.shape[1]) // 2, 0)
        pad_h = max((self.image_size - image.shape[0]) // 2, 0)
        pad_image = np.pad(image, ((pad_h, pad_h), (pad_w, pad_w), (0, 0)), mode='constant', constant_values=0)
        image = cv2.resize(pad_image, (self.image_size, self.image_size))
        image = np.asarray(image, np.float32) / 255.0
        return torch.tensor(image, dtype=torch.float32)

    def process_depth(self, depth_path):
        depth = (self.load_depth(depth_path) / 10000.0)
        H, W = depth.shape
        prop = self.image_size / max(H, W)
        depth = cv2.resize(depth, (int(W*prop), int(H*prop)))
        pad_w = max((self.image_size - depth.shape[1]) // 2, 0)
        pad_h = max((self.image_size - depth.shape[0]) // 2, 0)
        pad_depth = np.pad(depth, ((pad_h, pad_h), (pad_w, pad_w)), mode='constant', constant_values=0)
        pad_depth[pad_depth > 5.0] = 0
        pad_depth[pad_depth < 0.1] = 0
        depth = cv2.resize(pad_depth, (self.image_size, self.image_size))
        depth = np.asarray(depth, np.float32)
        return depth[:, :, np.newaxis]

    # ---------- ISAAC helpers ----------
    def _load_poses_isaac(self, meta):
        poses = np.load(meta['pose_path']).astype(np.float32)  # [N,4,4] or [N,7] xyz+quat
        if poses.ndim == 3 and poses.shape[1:] == (4,4):
            extr = poses
        elif poses.ndim == 2 and poses.shape[1] == 7:
            # xyz+quat -> 4x4
            t = poses[:,0:3]
            q = poses[:,3:7]  # x y z w
            Rm = Rotation.from_quat(q).as_matrix().astype(np.float32)
            extr = np.tile(np.eye(4, dtype=np.float32), (poses.shape[0],1,1))
            extr[:,0:3,0:3] = Rm
            extr[:,0:3,3]   = t
        else:
            raise ValueError(f"Unsupported pose shape: {poses.shape}")
        return extr

    def _timestamp_from_filename(self, path):
        """
        支持 lidar/1234567890.123.npy 或 000001.npy 的简单解析
        """
        name = os.path.splitext(os.path.basename(path))[0]
        try:
            return float(name)
        except Exception:
            # 序号转时间戳（假定 30Hz）：index / 30.0
            try:
                return int(name)/30.0
            except Exception:
                return None

    def _load_lidar_points(self, npy_path):
        pts = np.load(npy_path).astype(np.float32)  # 期望 Nx3 或 Nx4(xyz[i])
        if pts.ndim == 2 and pts.shape[1] >= 3:
            return pts[:,0:3]
        raise ValueError(f"Unsupported lidar shape in {npy_path}: {pts.shape}")

    def _bev_image_from_lidar_file(self, npy_path):
        pts = self._load_lidar_points(npy_path)
        bev = make_bev_from_points(pts, self.image_size, meters_per_pixel=self.lidar_bev_scale)
        return torch.tensor(bev, dtype=torch.float32)

    def _depth_like_from_lidar_file(self, npy_path):
        pts = self._load_lidar_points(npy_path)
        depth = depth_like_from_points(pts, self.image_size, meters_per_pixel=self.lidar_bev_scale)
        return depth

    # ---------- path/obstacle ----------
    def process_path_points(self, index):
        meta = self.trajectory_meta[index]
        mode = meta['mode']
        if mode == 'navdp_v1':
            trajectory_pcd = self.load_pointcloud(self.trajectory_afford_path[index])
            trajectory_color = np.asarray(trajectory_pcd.colors)
            color_distance = np.abs(trajectory_color - np.array([0, 0, 0])).sum(axis=-1)
            select_index = np.where(color_distance < 0.05)[0]
            trajectory_path = o3d.geometry.PointCloud()
            trajectory_path.points = o3d.utility.Vector3dVector(np.asarray(trajectory_pcd.points)[select_index])
            trajectory_path.colors = o3d.utility.Vector3dVector(np.asarray(trajectory_pcd.colors)[select_index])
            return np.asarray(trajectory_path.points), trajectory_path
        else:
            # isaac_recorder_v1
            gp = meta.get('global_path', None)
            if gp:
                pts = load_npy_or_json(gp).astype(np.float32)
                if pts.ndim == 2 and pts.shape[1] >= 2:
                    if pts.shape[1] == 2:
                        pts = np.concatenate([pts, np.zeros((pts.shape[0],1), np.float32)], axis=1)
                    # 用 open3d 容器只为保持接口一致
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(pts[:,0:3])
                    colors = np.zeros_like(pts[:,0:3]); pcd.colors = o3d.utility.Vector3dVector(colors)
                    return pts[:,0:3], pcd

            # 回退：用机器人轨迹作为路径参考
            extr = self._load_poses_isaac(meta)
            pts = extr[:,0:3,3]
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)
            colors = np.zeros_like(pts); pcd.colors = o3d.utility.Vector3dVector(colors)
            return pts, pcd

    def process_obstacle_points(self, index, path_points):
        meta = self.trajectory_meta[index]
        mode = meta['mode']
        if mode == 'navdp_v1':
            trajectory_pcd = self.load_pointcloud(self.trajectory_afford_path[index])
            trajectory_color = np.asarray(trajectory_pcd.colors)
            trajectory_points = np.asarray(trajectory_pcd.points)
            color_distance = np.abs(trajectory_color - np.array([0, 0, 0.5])).sum(axis=-1)
            path_lower = path_points.min(axis=0)
            path_upper = path_points.max(axis=0)
            cond_x = (trajectory_points[:,0] >= path_lower[0]-2.0) & (trajectory_points[:,0] <= path_upper[0]+2.0)
            cond_y = (trajectory_points[:,1] >= path_lower[1]-2.0) & (trajectory_points[:,1] <= path_upper[1]+2.0)
            select = np.where((color_distance < 0.05) & cond_x & cond_y)[0]
            obstacle = o3d.geometry.PointCloud()
            obstacle.points = o3d.utility.Vector3dVector(trajectory_points[select])
            obstacle.colors = o3d.utility.Vector3dVector(trajectory_color[select])
            return np.asarray(obstacle.points), obstacle
        else:
            # isaac_recorder_v1
            if meta.get('obstacles_path'):
                pts = load_npy_or_json(meta['obstacles_path']).astype(np.float32)
                if pts.ndim == 2 and pts.shape[1] >= 3:
                    return pts[:,0:3], None

            # 回退：在路径包围盒附近累计多帧 LiDAR 构造障碍点
            path_lower = path_points.min(axis=0) - np.array([2.0,2.0,3.0], np.float32)
            path_upper = path_points.max(axis=0) + np.array([2.0,2.0,3.0], np.float32)
            pts_all = []
            take = min(self.lidar_accumulate_frames, len(meta['lidar_files']))
            # 均匀取样几帧
            idxs = np.linspace(0, len(meta['lidar_files'])-1, take).astype(int)
            for i in idxs:
                p = self._load_lidar_points(meta['lidar_files'][i])
                mask = (p[:,0]>=path_lower[0])&(p[:,0]<=path_upper[0])&(p[:,1]>=path_lower[1])&(p[:,1]<=path_upper[1])
                pts_all.append(p[mask])
            if len(pts_all)==0:
                return np.zeros((0,3), np.float32), None
            pts = np.concatenate(pts_all, axis=0)
            # 简单下采样
            if pts.shape[0] > 200000:
                choice = np.random.choice(pts.shape[0], 200000, replace=False)
                pts = pts[choice]
            return pts.astype(np.float32), None

    # ---------- memory & pixel ----------
    def process_memory(self, rgb_paths, depth_paths, start_step, memory_digit=1,
                       bev_fallback=None, depth_fallback=None):
        """
        当 rgb_paths/depth_paths 为空时，使用 bev_fallback / depth_fallback(index)->tensor
        """
        mem_idx = np.arange(start_step - (self.memory_size - 1) * memory_digit, start_step + 1, memory_digit)
        outrange = (mem_idx < 0).sum()
        mem_idx = mem_idx[outrange:]

        context_image = np.zeros((self.memory_size, self.image_size, self.image_size, 3), np.float32)
        # RGB 路径存在 -> 正常读取；否则用 bev_fallback
        if rgb_paths and len(rgb_paths) > 0:
            imgs = [self.process_image(rgb_paths[i]).numpy() for i in mem_idx]
        else:
            assert bev_fallback is not None, "BEV fallback is required when RGB missing."
            imgs = [bev_fallback(i).numpy() for i in mem_idx]
        context_image[outrange:] = np.array(imgs, dtype=np.float32)

        # Depth：有就读，没有用 depth_fallback(start_step)
        if depth_paths and len(depth_paths) > 0:
            depth = self.process_depth(depth_paths[start_step])
        else:
            assert depth_fallback is not None, "Depth fallback is required when depth missing."
            depth = depth_fallback(start_step)

        return context_image, depth, mem_idx

    def process_pixel_from_bytes(self, image_url, target_point, camera_intrinsic, camera_extrinsic):
        image = Image.open(image_url).convert("RGB")
        image = np.array(image, np.uint8)
        resize_image = self.process_image(image_url)  # (H,W,3) torch

        coordinate = np.array([-target_point[1], target_point[0], camera_extrinsic[2, 3] * 0.8], dtype=np.float32)
        cam_coord = np.matmul(camera_extrinsic[0:3, 0:3], coordinate[:, None])
        px = camera_intrinsic[0, 2] + (cam_coord[0] / (cam_coord[2]+1e-6)) * camera_intrinsic[0, 0]
        py = camera_intrinsic[1, 2] + (-cam_coord[1] / (cam_coord[2]+1e-6)) * camera_intrinsic[1, 1]

        pixel_mask = np.zeros_like(image)
        H, W, C = image.shape
        # 边缘提示
        if px < 0: pixel_mask = cv2.rectangle(pixel_mask, (0, 0), (10, H), (255, 255, 255), -1)
        if py < 0: pixel_mask = cv2.rectangle(pixel_mask, (0, 0), (W, 10), (255, 255, 255), -1)
        if px >= W: pixel_mask = cv2.rectangle(pixel_mask, (W-10, 0), (W, H), (255, 255, 255), -1)
        if py >= H: pixel_mask = cv2.rectangle(pixel_mask, (0, H-10), (W, H), (255, 255, 255), -1)

        if (0 <= px < W) and (0 <= py < H):
            pixel_mask = cv2.rectangle(
                pixel_mask,
                (int(px - np.random.randint(6, 12)), int(py - np.random.randint(6, 12))),
                (int(px + np.random.randint(6, 12)), int(py + np.random.randint(6, 12))),
                (255, 255, 255), -1
            )

        prop = self.image_size / max(H, W)
        pixel_mask = cv2.resize(pixel_mask, (int(W*prop), int(H*prop)))
        pad_w = max((self.image_size - pixel_mask.shape[1]) // 2, 0)
        pad_h = max((self.image_size - pixel_mask.shape[0]) // 2, 0)
        pad_mask = np.pad(pixel_mask, ((pad_h, pad_h), (pad_w, pad_w), (0, 0)), mode='constant', constant_values=0)
        mask = cv2.resize(pad_mask, (self.image_size, self.image_size), interpolation=cv2.INTER_NEAREST)
        mask = (np.asarray(mask, np.float32) / 255.0).mean(axis=-1)[:, :, None]
        return np.concatenate((resize_image.numpy(), mask), axis=-1)

    # ---------- pose transforms ----------
    def relative_pose(self, R_base, T_base, R_world, T_world, base_extrinsic):
        R_base = np.matmul(R_base, np.linalg.inv(base_extrinsic[0:3, 0:3]))
        if len(T_world.shape) == 1:
            homo_RT = np.eye(4)
            homo_RT[0:3, 0:3] = R_base
            homo_RT[0:3, 3] = T_base
            R_frame = np.dot(R_world, R_base.T)
            T_frame = np.dot(np.linalg.inv(homo_RT), np.array([*T_world, 1]).T)[0:3]
            T_frame = np.array([T_frame[1], -T_frame[0], T_frame[2]])
            return R_frame, T_frame
        else:
            homo_RT = np.eye(4)
            homo_RT[0:3, 0:3] = R_base
            homo_RT[0:3, 3] = T_base
            R_frame = np.dot(R_world, R_base.T)
            T_frame = np.dot(np.linalg.inv(homo_RT),
                             np.concatenate((T_world, np.ones((T_world.shape[0], 1))), axis=-1).T).T[:, 0:3]
            T_frame = T_frame[:, [1, 0, 2]]
            T_frame[:, 1] = -T_frame[:, 1]
            return R_frame, T_frame

    def absolute_pose(self, R_base, T_base, R_frame, T_frame, base_extrinsic):
        R_base = np.matmul(R_base, np.linalg.inv(base_extrinsic[0:3, 0:3]))
        if len(T_frame.shape) == 1:
            homo_RT = np.eye(4)
            homo_RT[0:3, 0:3] = R_base
            homo_RT[0:3, 3] = T_base
            R_world = np.dot(R_frame, R_base)
            T_world = np.dot(homo_RT, np.array([-T_frame[1], T_frame[0], T_frame[2], 1]).T)[0:3]
        else:
            homo_RT = np.eye(4)
            homo_RT[0:3, 0:3] = R_base
            homo_RT[0:3, 3] = T_base
            R_world = np.dot(R_frame, R_base)
            T_world = np.dot(homo_RT, np.concatenate(
                (np.stack((-T_frame[:, 1], T_frame[:, 0], T_frame[:, 2]), axis=-1), np.ones((T_frame.shape[0], 1))),
                axis=-1).T).T[:, 0:3]
        return R_world, T_world

    def xyz_to_xyt(self, xyz_actions, init_vector):
        xyt_actions = []
        for i in range(0, xyz_actions.shape[0] - 1):
            current = xyz_actions[i + 1] - xyz_actions[i]
            dotp = np.dot(init_vector[0:2], current[0:2])
            crossp = np.cross(init_vector[0:2], current[0:2])
            theta = np.arctan2(crossp, dotp)
            xyt_actions.append([xyz_actions[i][0], xyz_actions[i][1], theta])
        return np.array(xyt_actions, dtype=np.float32)

    def process_actions(self, extrinsics, base_extrinsic, start_step, end_step, pred_digit=1):
        label_linear_pos = []
        for f_ext in extrinsics[start_step:end_step + 1]:
            R, T = self.relative_pose(extrinsics[start_step][0:3, 0:3], extrinsics[start_step][0:3, 3],
                                      f_ext[0:3, 0:3], f_ext[0:3, 3], base_extrinsic)
            label_linear_pos.append(T)
        label_actions = np.array(label_linear_pos, dtype=np.float32)

        # augmentation
        rotate_yaw = np.random.uniform(-np.pi/3, np.pi/3)
        rot2 = np.array([[np.cos(rotate_yaw), -np.sin(rotate_yaw)],
                         [np.sin(rotate_yaw),  np.cos(rotate_yaw)]], np.float32)
        rot_local = np.matmul(rot2, label_actions[:,0:2].T).T
        rot_local = np.stack((rot_local[:,0], rot_local[:,1], np.zeros_like(rot_local[:,0])), axis=-1)

        rotate_world_points = []
        for act in rot_local:
            w_rot, w_act = self.absolute_pose(extrinsics[start_step,0:3,0:3], extrinsics[start_step,0:3,3],
                                              np.eye(3), act, base_extrinsic)
            rotate_world_points.append(w_act)
        rotate_world_points = np.array(rotate_world_points, dtype=np.float32)
        origin_world_points = extrinsics[start_step:end_step + 1, 0:3, 3].astype(np.float32)
        mix_anchor_points = rotate_world_points

        t = np.linspace(0, 1, mix_anchor_points.shape[0])
        cs_x = CubicSpline(t, mix_anchor_points[:, 0])
        cs_y = CubicSpline(t, mix_anchor_points[:, 1])
        cs_z = CubicSpline(t, mix_anchor_points[:, 2])
        interp_n = origin_world_points.shape[0]
        t_fine = np.linspace(0, 1, int(interp_n))
        x_fine = cs_x(t_fine); y_fine = cs_y(t_fine); z_fine = cs_z(t_fine)
        augment_world_points = np.stack((x_fine, y_fine, z_fine), axis=-1).astype(np.float32)

        local_label_points = []
        local_augment_points = []
        for f_ext, g_ext in zip(origin_world_points, augment_world_points):
            _, Tf = self.relative_pose(extrinsics[start_step][0:3,0:3], extrinsics[start_step][0:3,3],
                                       np.eye(3), f_ext, base_extrinsic)
            _, Tg = self.relative_pose(extrinsics[start_step][0:3,0:3], extrinsics[start_step][0:3,3],
                                       np.eye(3), g_ext, base_extrinsic)
            local_label_points.append(Tf); local_augment_points.append(Tg)
        local_label_points  = np.array(local_label_points , dtype=np.float32)
        local_augment_points= np.array(local_augment_points, dtype=np.float32)
        action_indexes = np.clip(np.arange(self.predict_size + 1) * pred_digit, 0, local_label_points.shape[0] - 1)
        return local_label_points, local_augment_points, origin_world_points, augment_world_points, action_indexes

    def rank_steps(self, extrinsics, obstacle_points, pred_digit=4):
        points_score = []
        trajectory = extrinsics[:, 0:2, 3]
        for i in range(0, trajectory.shape[0] - 1):
            future = trajectory[i:min(i + self.predict_size * pred_digit, trajectory.shape[0] - 1)]
            fb = [np.min(future[:,0])-1, np.min(future[:,1])-1, np.max(future[:,0])+1, np.max(future[:,1])+1]
            if obstacle_points.shape[0] == 0:
                points_score.append(0.0)
                continue
            within = (obstacle_points[:,0] > fb[0]) & (obstacle_points[:,1] > fb[1]) & \
                     (obstacle_points[:,0] < fb[2]) & (obstacle_points[:,1] < fb[3])
            points_score.append(np.sum(within))
        ps = np.array(points_score, dtype=np.float32)
        ps = ps / (ps.max() + 1e-8)
        probs = np.exp(ps / 0.2) / np.sum(np.exp(ps / 0.2))
        start_choice = np.random.choice(np.arange(probs.shape[0]), p=probs)
        cand = np.arange(start_choice + 1, trajectory.shape[0])
        if cand.size == 0:
            start_choice = 0
            cand = np.arange(1, trajectory.shape[0])
        p = (cand - start_choice) / (float((cand - start_choice).max()) + 1e-8)
        p = np.exp(p / 0.2); p = p / p.sum()
        target_choice = np.random.choice(cand, p=p)
        return start_choice, target_choice

    # ---------- GETITEM ----------
    def __getitem__(self, index):
        if self._last_time is None:
            self._last_time = time.time()
        start_time = time.time()

        meta = self.trajectory_meta[index]
        mode = meta['mode']

        if mode == 'navdp_v1':
            # === 原逻辑 ===
            traj_data = safe_load_json(self.trajectory_data_dir[index])
            trajectory_extrinsics = np.array(traj_data['camera_trajectory'], dtype=np.float32)
            trajectory_base_extrinsic = np.array(traj_data['camera_extrinsic'], dtype=np.float32)
            camera_intrinsic = np.array(traj_data['camera_intrinsic'], dtype=np.float32)

            traj_len = len(trajectory_extrinsics)
            path_pts, _ = self.process_path_points(index)
            obstacle_pts, _ = self.process_obstacle_points(index, path_pts)

            if self.prior_sample:
                start_choice, target_choice = self.rank_steps(trajectory_extrinsics, obstacle_pts, pred_digit=4)
            else:
                start_choice = np.random.randint(0, max(1, traj_len // 2))
                target_choice = np.random.randint(start_choice + 1, traj_len - 1)

            if self.random_digit:
                memory_digit = np.random.randint(2, 8)
                pred_digit = memory_digit
            else:
                memory_digit = 4
                pred_digit = 4

            memory_images, depth_image, memory_index = self.process_memory(
                self.trajectory_rgb_path[index],
                self.trajectory_depth_path[index],
                start_choice, memory_digit=memory_digit
            )

            (target_local_points,
             augment_local_points,
             target_world_points,
             augment_world_points,
             action_indexes) = self.process_actions(trajectory_extrinsics,
                                                    trajectory_base_extrinsic,
                                                    start_choice, target_choice,
                                                    pred_digit=pred_digit)

            init_vec = target_local_points[1] - target_local_points[0]
            target_xyt = self.xyz_to_xyt(target_local_points, init_vec)
            augment_xyt= self.xyz_to_xyt(augment_local_points,  init_vec)

            pred_actions   = target_xyt[action_indexes]
            augment_actions= augment_xyt[action_indexes]

            if obstacle_pts.shape[0] != 0:
                pdist = np.abs(target_world_points[:,None,0:2] - obstacle_pts[None,:,0:2]).sum(axis=-1).min(axis=-1)
                adist = np.abs(augment_world_points[:,None,0:2] - obstacle_pts[None,:,0:2]).sum(axis=-1).min(axis=-1)
                pred_critic = -5.0 * (pdist[action_indexes[:-1]] < 0.1).mean() + 0.5 * (pdist[action_indexes][1:] - pdist[action_indexes][:-1]).sum()
                augment_critic = -5.0 * (adist[action_indexes[:-1]] < 0.1).mean() + 0.5 * (adist[action_indexes][1:] - adist[action_indexes][:-1]).sum()
            else:
                pred_critic = 2.0
                augment_critic = 2.0

            point_goal = target_xyt[-1]
            # 拼接目标/起始图像
            image_goal = np.concatenate((self.process_image(self.trajectory_rgb_path[index][target_choice]).numpy(),
                                         self.process_image(self.trajectory_rgb_path[index][start_choice]).numpy()), axis=-1)
            pixel_goal = self.process_pixel_from_bytes(self.trajectory_rgb_path[index][start_choice],
                                                       pred_actions[-1], camera_intrinsic, trajectory_base_extrinsic)

        else:
            # === ISAAC RECORDER V1 ===
            # poses/extrinsics
            extr = self._load_poses_isaac(meta)                         # [N,4,4]
            traj_len = extr.shape[0]
            base_extrinsic = identity_extrinsic()
            K = default_camera_intrinsic(self.image_size)
            # 路径/障碍
            path_pts, _ = self.process_path_points(index)
            obstacle_pts, _ = self.process_obstacle_points(index, path_pts)

            # 采样起止
            if self.prior_sample:
                start_choice, target_choice = self.rank_steps(extr, obstacle_pts, pred_digit=4)
            else:
                if traj_len < 4:  # 兜底
                    start_choice = 0; target_choice = max(1, traj_len-1)
                else:
                    start_choice = np.random.randint(0, traj_len // 2)
                    target_choice = np.random.randint(start_choice + 1, traj_len - 1)

            # 步长
            if self.random_digit:
                memory_digit = np.random.randint(2, 8)
                pred_digit = memory_digit
            else:
                memory_digit = 4
                pred_digit = 4

            # 构造“伪 RGB/Depth”时的 fallback（来自 LiDAR）
            def bev_fallback(i):
                # i 索引映射到 lidar 文件：按帧号近似
                lid_idx = np.clip(i, 0, len(meta['lidar_files'])-1)
                return self._bev_image_from_lidar_file(meta['lidar_files'][lid_idx])

            def depth_fallback(i):
                lid_idx = np.clip(i, 0, len(meta['lidar_files'])-1)
                return self._depth_like_from_lidar_file(meta['lidar_files'][lid_idx])

            memory_images, depth_image, memory_index = self.process_memory(
                self.trajectory_rgb_path[index],       # 可能为空
                self.trajectory_depth_path[index],     # 可能为空
                start_choice, memory_digit=memory_digit,
                bev_fallback=bev_fallback,
                depth_fallback=depth_fallback
            )

            (target_local_points,
             augment_local_points,
             target_world_points,
             augment_world_points,
             action_indexes) = self.process_actions(extr,
                                                    base_extrinsic,
                                                    start_choice, target_choice,
                                                    pred_digit=pred_digit)

            init_vec = target_local_points[1] - target_local_points[0]
            target_xyt = self.xyz_to_xyt(target_local_points, init_vec)
            augment_xyt= self.xyz_to_xyt(augment_local_points,  init_vec)

            pred_actions   = target_xyt[action_indexes]
            augment_actions= augment_xyt[action_indexes]

            if obstacle_pts.shape[0] != 0:
                pdist = np.abs(target_world_points[:,None,0:2] - obstacle_pts[None,:,0:2]).sum(axis=-1).min(axis=-1)
                adist = np.abs(augment_world_points[:,None,0:2] - obstacle_pts[None,:,0:2]).sum(axis=-1).min(axis=-1)
                pred_critic = -5.0 * (pdist[action_indexes[:-1]] < 0.1).mean() + 0.5 * (pdist[action_indexes][1:] - pdist[action_indexes][:-1]).sum()
                augment_critic = -5.0 * (adist[action_indexes[:-1]] < 0.1).mean() + 0.5 * (adist[action_indexes][1:] - adist[action_indexes][:-1]).sum()
            else:
                pred_critic = 2.0
                augment_critic = 2.0

            point_goal = target_xyt[-1]

            # image_goal & pixel_goal：
            # 优先用真实 RGB，否则用 BEV 代替；pixel_goal 里的“图像”也走同样降级
            def get_rgb_like(idx):
                if meta['rgb_files']:
                    return self.process_image(meta['rgb_files'][np.clip(idx,0,len(meta['rgb_files'])-1)]).numpy()
                return bev_fallback(idx).numpy()

            image_goal = np.concatenate((get_rgb_like(target_choice),
                                         get_rgb_like(start_choice)), axis=-1)

            # pixel_goal：若无真实 RGB，就用 BEV 作为底图
            if meta['rgb_files']:
                start_img_path = meta['rgb_files'][np.clip(start_choice,0,len(meta['rgb_files'])-1)]
                pixel_goal = self.process_pixel_from_bytes(start_img_path,
                                                           pred_actions[-1], K, base_extrinsic)
            else:
                # 用 BEV 代替，mask 叠加为空间提示：直接把 mask 全 0（或基于像素投影到 BEV，可加复杂映射）
                bev = bev_fallback(start_choice).numpy()
                mask = np.zeros((self.image_size, self.image_size, 1), np.float32)
                pixel_goal = np.concatenate((bev, mask), axis=-1)

        # 差分 & 张量化
        pred_actions    = (pred_actions[1:]    - pred_actions[:-1]) * 4.0
        augment_actions = (augment_actions[1:] - augment_actions[:-1]) * 4.0

        end_time = time.time()
        self.item_cnt += 1
        self.batch_time_sum += (end_time - start_time)
        if self.item_cnt % self.batch_size == 0:
            avg_time = self.batch_time_sum / self.batch_size
            print(f'__getitem__ pid={os.getpid()}, avg_time(last {self.batch_size})={avg_time:.2f}s, cnt={self.item_cnt}')
            self.batch_time_sum = 0.0

        point_goal   = torch.tensor(point_goal, dtype=torch.float32)
        image_goal   = torch.tensor(image_goal, dtype=torch.float32)
        pixel_goal   = torch.tensor(pixel_goal, dtype=torch.float32)
        memory_images= torch.tensor(memory_images, dtype=torch.float32)
        depth_image  = torch.tensor(depth_image, dtype=torch.float32)
        pred_actions = torch.tensor(pred_actions, dtype=torch.float32)
        augment_actions = torch.tensor(augment_actions, dtype=torch.float32)
        pred_critic  = torch.tensor(pred_critic, dtype=torch.float32)
        augment_critic = torch.tensor(augment_critic, dtype=torch.float32)

        return (point_goal, image_goal, pixel_goal,
                memory_images, depth_image,
                pred_actions, augment_actions,
                pred_critic, augment_critic)

# collate 保持不变
def navdp_collate_fn(batch):
    collated = {
        "batch_pg": torch.stack([item[0] for item in batch]),
        "batch_ig": torch.stack([item[1] for item in batch]),
        "batch_tg": torch.stack([item[2] for item in batch]),
        "batch_rgb": torch.stack([item[3] for item in batch]),
        "batch_depth": torch.stack([item[4] for item in batch]),
        "batch_labels": torch.stack([item[5] for item in batch]),
        "batch_augments": torch.stack([item[6] for item in batch]),
        "batch_label_critic": torch.stack([item[7] for item in batch]),
        "batch_augment_critic": torch.stack([item[8] for item in batch]),
    }
    return collated
