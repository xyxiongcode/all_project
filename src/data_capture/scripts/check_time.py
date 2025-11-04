import json

# 查看轨迹文件中的时间戳
with open('/media/gr-agv-x9xy/backup_xxy/camera_data/train/trajectory/trajectory_44000000656.json', 'r') as f:
    data = json.load(f)

# 查看前几个点的时间戳
trajectory = data.get('recent_trajectory', [])
for i, point in enumerate(trajectory[:5]):
    timestamp = point.get('timestamp', 0)
    print(f"点{i}: 时间戳 = {timestamp}")

    # 计算时间间隔（如果是连续点）
    if i > 0:
        prev_timestamp = trajectory[i - 1].get('timestamp', 0)
        time_diff = timestamp - prev_timestamp
        print(f"  与前一点时间差: {time_diff} 单位")