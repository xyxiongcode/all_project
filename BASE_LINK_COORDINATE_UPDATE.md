# base_link坐标系更新说明

## 更新概述

已确认机器人本体中心为 `base_link` 坐标系，已更新所有相关代码和注释，统一使用 `base_link` 作为机器人本体中心坐标系。

---

## 更新的文件

### 1. pose_utils.py

**更新内容**：
- 明确注释说明使用 `base_link` 作为机器人本体中心
- 更新文档字符串，说明查询 `map->base_link` 变换
- 移除对 `base_footprint` 的引用

**关键修改**：
```python
class UnifiedPoseGetter:
    """
    统一的机器人位姿获取器（使用TF查询）
    直接查询 map->base_link 的变换
    机器人本体中心为 base_link 坐标系
    """
    
    def get_robot_pose_map(self):
        """
        获取机器人在map坐标系中的位姿（统一方法）
        直接查询 map->base_link 的变换（机器人本体中心）
        """
        # 直接查询map到base_link的变换（机器人本体中心）
        transform = self.tf_buffer.lookup_transform(
            self.map_frame,      # 目标坐标系: map
            self.base_frame,     # 源坐标系: base_link（机器人本体中心）
            rospy.Time(0),
            rospy.Duration(1.0)
        )
```

---

### 2. random_generate_goal.py

**更新内容**：
- 明确参数说明：`base_frame` 为机器人本体中心坐标系
- 更新日志输出，说明使用 `base_link`
- 更新函数注释，说明查询 `map->base_link` 变换

**关键修改**：
```python
self.base_frame = rospy.get_param('~base_frame', 'base_link')  # 机器人本体中心坐标系

# 日志输出
rospy.loginfo(f"[目标生成器] 机器人本体中心坐标系: {self.base_frame}")

# 函数注释
def odom_callback(self, msg: Odometry):
    """
    里程计回调 - 用于移动检测
    机器人位置通过TF直接查询map坐标系获取（查询 map->base_link 变换）
    """
    # 使用TF直接查询map坐标系中的机器人位姿（机器人本体中心 base_link）
    pose_dict = self.pose_getter.get_robot_pose_map()
```

---

### 3. navdp_generate_dataset.py

**更新内容**：
- 明确参数说明：`base_frame` 为机器人本体中心坐标系
- 更新日志输出，说明使用 `base_link` 和 `map->base_link` 变换
- 更新函数注释，说明查询 `map->base_link` 变换

**关键修改**：
```python
self.base_frame = rospy.get_param('~base_frame', 'base_link')  # 机器人本体中心坐标系

# 日志输出
rospy.loginfo(f"[数据收集器] 机器人本体中心坐标系: {self.base_frame} (map->base_link变换)")

# 函数注释
def _extract_robot_pose(self, odom_msg):
    """
    提取机器人位姿（统一使用TF查询map坐标系）
    直接查询 map->base_link 的变换（机器人本体中心）
    """
    # 使用TF直接查询map坐标系中的机器人位姿（机器人本体中心 base_link）
    pose_dict = self.pose_getter.get_robot_pose_map()
```

---

### 4. get_pose.py

**更新内容**：
- 修正注释，使其与实际代码一致
- 注释说明使用 `base_link`（机器人本体中心）

**关键修改**：
```python
try:
    # 获取从map到base_link的变换（机器人本体中心）
    (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
```

---

## 坐标系统一说明

### 统一的坐标系配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `base_frame` | `'base_link'` | 机器人本体中心坐标系 |
| `map_frame` | `'map'` | 地图坐标系 |
| `odom_frame` | `'odom'` | 里程计坐标系 |

### TF变换查询

**统一的查询方法**：
- 查询 `map -> base_link` 变换
- 获取机器人在 `map` 坐标系中的位姿
- 机器人本体中心为 `base_link`

---

## 关键改进

### ✅ 明确坐标系定义

- 统一使用 `base_link` 作为机器人本体中心
- 所有注释和文档已更新
- 不再引用 `base_footprint`

### ✅ 一致性保证

- 两个文件使用相同的坐标系定义
- 与 `get_pose.py` 保持一致
- 与 ROS 标准约定一致

### ✅ 代码清晰

- 注释明确说明使用 `base_link`
- 日志输出包含坐标系信息
- 便于后续维护和调试

---

## 测试验证

### 验证TF变换

```bash
# 验证 map->base_link 变换是否存在
rosrun tf tf_echo map base_link

# 应该能看到持续的变换输出：
# At time 1234.567
# - Translation: [x, y, z]
# - Rotation: [qx, qy, qz, qw]
```

### 验证坐标一致性

运行两个脚本后，检查：
- 终端输出的坐标与 `amcl_pose` 一致
- 两个文件输出的坐标一致
- 距离计算准确

---

## 总结

✅ **统一使用 `base_link`**：机器人本体中心坐标系  
✅ **查询方法统一**：`map -> base_link` TF变换  
✅ **代码注释清晰**：明确说明使用 `base_link`  
✅ **与标准一致**：符合ROS标准约定  

---

**文档创建时间**：2024年  
**相关文档**：`UNIFIED_COORDINATE_UPDATE.md`, `GET_ROBOT_POSE_ROS1.md`

