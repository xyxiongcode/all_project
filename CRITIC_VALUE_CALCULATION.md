# NavDP Critic值计算方法

## 根据NavDP论文的Critic值计算

根据[NavDP论文](https://wzcai99.github.io/navigation-diffusion-policy.github.io/)，critic值应该从轨迹到障碍物的距离计算。

### 原始NavDP公式

```python
pred_critic = -5.0 * (pdist[action_indexes[:-1]] < 0.1).mean() + 0.5 * (pdist[action_indexes][1:] - pdist[action_indexes][:-1]).sum()
augment_critic = -5.0 * (adist[action_indexes[:-1]] < 0.1).mean() + 0.5 * (adist[action_indexes][1:] - adist[action_indexes][:-1]).sum()
```

其中：
- `pdist`: 轨迹上每个点到障碍物的最小距离
- 第一项 `-5.0 * (pdist < 0.1).mean()`: 惩罚距离障碍物太近的点（< 0.1m）
- 第二项 `0.5 * (pdist[1:] - pdist[:-1]).sum()`: 鼓励轨迹逐渐远离障碍物

### 对于真实世界数据的简化方案

由于我们没有ESDF地图，可以使用以下方法：

1. **基于min_lidar_distance的简化方法**（当前推荐）：
   ```python
   min_distance = nav_semantics.get('min_lidar_distance', 2.0)
   if min_distance < 0.1:
       critic_value = -5.0
   elif min_distance < 0.5:
       critic_value = -2.0 + min_distance * 2.0
   else:
       critic_value = 0.5 + min_distance / 2.0
   ```

2. **完整的轨迹距离计算方法**（需要轨迹点坐标）：
   - 从actions重建轨迹点
   - 使用LiDAR数据提取障碍物点
   - 计算每个轨迹点到障碍物的距离
   - 应用原始公式

## 当前问题

在`navdp_train.py`中，critic值只是简单地基于`move_base_success`：
```python
batch_label_critic = torch.tensor([1.0 if move_base_success else 0.0], dtype=torch.float32)
```

这不符合NavDP的设计，应该基于轨迹安全性计算。

