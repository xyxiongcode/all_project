import numpy as np

# 添加 allow_pickle=True 参数
data = np.load("/media/gr-agv-x9xy/backup_xxy/camera_data/train/lidar/lidar_27566668104.npy", allow_pickle=True)
print("数据形状:", data.shape)
print("数据类型:", data.dtype)
print("数据内容:")
print(data)