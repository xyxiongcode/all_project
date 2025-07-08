import joblib
import numpy as np

def inspect_pkl(path):
    with open(path, 'rb') as f:
        data = joblib.load(f)

    if isinstance(data, (tuple, list)):
        print(f"[INFO] 数据类型: {type(data)}, 元素数量: {len(data)}")
        for i, item in enumerate(data):
            if isinstance(item, np.ndarray):
                print(f"  - 第 {i} 项: shape={item.shape}, dtype={item.dtype}")
                print(f"    示例值（前1个样本）: {item[0]}")
            else:
                print(f"  - 第 {i} 项: type={type(item)}")
    elif isinstance(data, dict):
        print(f"[INFO] 数据是字典，包含键：{list(data.keys())}")
        for k, v in data.items():
            if isinstance(v, np.ndarray):
                print(f"  - {k}: shape={v.shape}, dtype={v.dtype}")
                print(f"    示例值（前1个样本）: {v[0]}")
            else:
                print(f"  - {k}: type={type(v)}")
    else:
        print(f"[WARN] 不是 tuple 或 dict，而是 {type(data)}")

if __name__ == "__main__":
    pkl_path = "/home/gr-agv-x9xy/Downloads/pretraining_dataset/db_cnn_data/data_64.pkl"
    inspect_pkl(pkl_path)
