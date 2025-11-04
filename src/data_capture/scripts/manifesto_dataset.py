# build_manifest.py
from pathlib import Path
import re, csv, sys
from collections import defaultdict, Counter

ROOT = Path(__file__).resolve().parent  # camera_data/train
GLOB_DIRS = [
    "imu", "trajectory", "lidar", "goal_images", "depth", "depth_vis",
    "rgb", "actions", "trajectory_vis", "goal_data"
]
# 正则：前缀_时间戳.扩展名  e.g. imu_1000316718837.json / goal_123.png
PAT = re.compile(r"^(?P<prefix>[A-Za-z]+)_(?P<ts>\d+)\.[A-Za-z0-9]+$")

# 将子目录名映射到清晰的“模态列名”
CANON = {
    "imu": "imu",
    "trajectory": "trajectory",
    "lidar": "lidar",
    "goal_images": "goal_image",
    "goal_data": "goal",
    "depth": "depth",
    "depth_vis": "depth_vis",
    "rgb": "rgb",  # 注意：若 rgb 目录不是按时间戳命名的单帧，而是帧序列，请按需调整
    "actions": "actions",
    "trajectory_vis": "trajectory_vis",
}

def scan():
    by_ts = defaultdict(dict)
    counts = Counter()

    for d in GLOB_DIRS:
        p = ROOT / d
        if not p.exists():
            continue
        for fp in p.rglob("*"):
            if not fp.is_file():
                continue
            name = fp.name
            m = PAT.match(name)
            if not m:
                # 允许像 enhanced_navigation_data.csv 之类跳过
                continue
            prefix, ts = m.group("prefix"), m.group("ts")
            # 有些目录内文件前缀和目录名一致（最好优先以目录名判断模态）
            modality = CANON.get(d, prefix.lower())
            # 若一个时间戳下同模态出现多个文件，则仅记录第一个，额外计数
            if modality not in by_ts[ts]:
                by_ts[ts][modality] = str(fp.relative_to(ROOT))
            else:
                counts[(modality, "duplicates")] += 1
            counts[(modality, "files")] += 1
            counts[("ts_total", "files")] += 1

    return by_ts, counts

def main():
    by_ts, counts = scan()

    # 统计所有出现过的模态列，保证 CSV 列一致
    all_modalities = set()
    for ts, d in by_ts.items():
        all_modalities.update(d.keys())
    # 你最关心的一般是 imu 与 trajectory 是否齐
    preferred_cols = ["imu", "trajectory", "lidar", "goal", "goal_image",
                      "rgb", "depth", "depth_vis", "actions", "trajectory_vis"]
    cols = ["ts"] + [c for c in preferred_cols if c in all_modalities] + \
           sorted(list(all_modalities - set(preferred_cols)))

    out_csv = ROOT / "dataset_index.csv"
    with out_csv.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols + ["missing_modalities"])
        for ts in sorted(by_ts.keys(), key=lambda x: int(x)):
            row = [ts]
            present = set()
            for c in cols[1:]:
                v = by_ts[ts].get(c, "")
                row.append(v)
                if v:
                    present.add(c)
            missing = [c for c in cols[1:] if c not in present]
            row.append("|".join(missing))
            w.writerow(row)

    # 快速报告
    total_ts = len(by_ts)
    have_imu = sum(1 for ts in by_ts if "imu" in by_ts[ts])
    have_traj = sum(1 for ts in by_ts if "trajectory" in by_ts[ts])
    both_it = sum(1 for ts in by_ts if "imu" in by_ts[ts] and "trajectory" in by_ts[ts])

    print(f"[OK] Wrote {out_csv}")
    print(f"Unique timestamps: {total_ts}")
    print(f"Have imu: {have_imu}, have trajectory: {have_traj}, both: {both_it}")
    if both_it < max(have_imu, have_traj):
        print(f"WARNING: {max(have_imu, have_traj)-both_it} timestamps missing alignment between imu/trajectory")

    # 模态计数摘要
    per_mod = {}
    for (mod, kind), v in counts.items():
        if mod == "ts_total": continue
        per_mod.setdefault(mod, {})[kind] = v
    print("\nPer-modality counts:")
    for mod in sorted(per_mod):
        files = per_mod[mod].get("files", 0)
        dups  = per_mod[mod].get("duplicates", 0)
        print(f"  {mod:15s} files={files:7d}  duplicates={dups}")

    print("\nTip: open the CSV and筛掉 missing_modalities 为空的行，即‘完全对齐’样本。")

if __name__ == "__main__":
    sys.exit(main())
