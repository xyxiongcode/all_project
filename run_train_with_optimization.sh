#!/bin/bash
# 使用内存优化的训练启动脚本

echo "=== 设置内存优化环境变量 ==="

# PyTorch CUDA内存分配优化（减少碎片化，提高利用率）
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True

# 限制CUDA缓存分配（可选，可能会稍微降低性能）
# export CUDA_LAUNCH_BLOCKING=0

echo "PYTORCH_CUDA_ALLOC_CONF=$PYTORCH_CUDA_ALLOC_CONF"
echo ""

# 清理CUDA缓存（如果有残留进程）
echo "=== 清理CUDA缓存 ==="
python -c "import torch; torch.cuda.empty_cache() if torch.cuda.is_available() else None"
echo ""

# 运行训练
echo "=== 开始训练 ==="
cd /home/gr-agv-x9xy/isaac_sim_ws/src/data_capture/scripts
python train.py

