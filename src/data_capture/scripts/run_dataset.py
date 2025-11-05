# run_dataset.py
import torch
from torch.utils.data import DataLoader
from navdp_dataset import NavDP_Base_Datset, navdp_collate_fn   # ← 改成你的脚本文件名

if __name__ == "__main__":
    ds = NavDP_Base_Datset(
        root_dirs=[
            "/media/gr-agv-x9xy/backup_xxy/camera_data/train",
            "/media/gr-agv-x9xy/backup_xxy/camera_data/test",
        ],
        preload_path="/media/gr-agv-x9xy/backup_xxy/camera_data/dataset_index.json",  # 第一次会写入；以后可复用
        preload=False,                     # 第一次建立索引 => False；之后改 True 加速
        dataset_mode="camera_data_v1",    # 你的数据布局
        image_size=224,
        memory_size=16,
        predict_size=24,
        random_digit=False,
        prior_sample=False,
        debug=True
    )

    print("样本总数：", len(ds))
    dl = DataLoader(ds, batch_size=2, shuffle=True, num_workers=4,
                    collate_fn=navdp_collate_fn, drop_last=True)

    batch = next(iter(dl))
    # 打印关键张量形状，确认无误
    for k, v in batch.items():
        try:
            print(k, tuple(v.shape))
        except Exception:
            print(k, v)

