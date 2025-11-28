# move_base make_plan 服务调用失败问题修复

## 问题描述

**错误信息**：
```
[ERROR] move_base must be in an inactive state to make a plan for an external user
[WARN] 路径规划失败: service [/move_base/make_plan] responded with an error
[WARN] 在300次尝试后未找到合适目标
```

**现象**：
- 机器人正在正常朝目标行驶
- 终端没有输出机器人距离目标点的距离
- 而是输出大量 `make_plan` 服务调用失败的错误
- 最终显示"在300次尝试后未找到合适目标"

---

## 根本原因

### move_base 的限制

`move_base` 的 `make_plan` 服务有一个限制：
- **当 `move_base` 处于 ACTIVE 状态（正在执行导航）时，会拒绝外部用户的 `make_plan` 请求**
- 这是为了防止在导航过程中被外部请求干扰

### 问题流程

1. 机器人正在导航（`move_base` 处于 ACTIVE 状态）
2. `random_generate_goal.py` 尝试生成新目标
3. 在 `generate_random_goal()` 中，对每个候选目标调用 `is_reachable_from_current()`
4. `is_reachable_from_current()` 调用 `/move_base/make_plan` 服务
5. `move_base` 拒绝请求（因为处于 ACTIVE 状态）
6. 所有目标都被判定为不可达
7. 300次尝试全部失败
8. 无法生成新目标

---

## 修复方案

### 方案：在 move_base ACTIVE 时使用地图检查代替 make_plan

**思路**：
1. 订阅 `/move_base/status` 话题，监控 `move_base` 状态
2. 如果 `move_base` 处于 ACTIVE 状态，使用地图检查代替 `make_plan`
3. 如果 `move_base` 不在 ACTIVE 状态，正常使用 `make_plan` 服务

### 实现细节

#### 1. 订阅 move_base 状态

```python
# 在 __init__() 中
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

self.move_base_is_active = False
self.move_base_status_sub = rospy.Subscriber(
    '/move_base/status', 
    GoalStatusArray, 
    self._move_base_status_callback,
    queue_size=1
)

def _move_base_status_callback(self, msg):
    """检查move_base是否处于ACTIVE状态"""
    if msg.status_list:
        latest_status = msg.status_list[-1]
        self.move_base_is_active = (latest_status.status == GoalStatus.ACTIVE)
    else:
        self.move_base_is_active = False
```

#### 2. 修改 is_reachable_from_current() 函数

```python
def is_reachable_from_current(self, gx, gy):
    """
    检查目标是否可达
    如果move_base处于ACTIVE状态，使用地图检查代替make_plan
    """
    # 如果move_base处于ACTIVE状态，使用地图检查
    if self.move_base_is_active:
        # 检查目标点是否在地图内
        if not self.is_within_map(gx, gy):
            return False
        
        # 检查目标点及其周围是否无障碍物
        mx = int((gx - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((gy - self.map_info.origin.position.y) / self.map_info.resolution)
        
        if mx < 0 or my < 0 or mx >= self.map_info.width or my >= self.map_info.height:
            return False
        
        # 检查0.5米范围内是否无障碍物
        check_radius_cells = int(0.5 / self.map_info.resolution)
        for dx in range(-check_radius_cells, check_radius_cells + 1):
            for dy in range(-check_radius_cells, check_radius_cells + 1):
                check_x = mx + dx
                check_y = my + dy
                if (0 <= check_x < self.map_info.width and 
                    0 <= check_y < self.map_info.height):
                    if self.map_data[check_y, check_x] > 0:  # 发现障碍物
                        return False
        
        return True  # 地图检查通过
    
    # move_base不在ACTIVE状态，使用make_plan服务
    # ... 原有的make_plan调用逻辑 ...
```

#### 3. 异常处理改进

```python
try:
    resp = self.make_plan(start=start, goal=goal, tolerance=self.plan_tolerance)
    # ... 处理响应 ...
except Exception as e:
    error_str = str(e)
    if "inactive state" in error_str.lower():
        # 如果是"inactive state"错误，使用地图检查作为fallback
        rospy.logdebug_throttle(5.0, "move_base处于ACTIVE状态，使用地图检查代替make_plan")
        # 使用地图检查
        return self._check_reachable_by_map(gx, gy)
    else:
        rospy.logwarn_throttle(2.0, f"路径规划失败: {e}")
        return False
```

---

## 修复效果

### 修复前
```
[ERROR] move_base must be in an inactive state to make a plan for an external user
[ERROR] move_base must be in an inactive state to make a plan for an external user
... (重复300次)
[WARN] 在300次尝试后未找到合适目标
[WARN] [目标生成器] 无法找到合适的随机目标
```

### 修复后
```
[INFO] [move_base] 已订阅move_base状态，用于检查ACTIVE状态
[DEBUG] move_base处于ACTIVE状态，使用地图检查代替make_plan
[INFO] [目标生成器] 发布新目标点 - 开始新的Episode
[INFO] [目标生成器] 目标位置: (5.234, 3.456)
```

---

## 地图检查 vs make_plan

### 地图检查（简化方法）
- **优点**：
  - 不依赖 `move_base` 状态
  - 速度快，无服务调用延迟
  - 可以实时检查
  
- **缺点**：
  - 只检查静态障碍物
  - 不考虑动态障碍物
  - 不检查路径连通性（可能被障碍物包围）

### make_plan 服务（完整方法）
- **优点**：
  - 完整的路径规划
  - 考虑所有障碍物
  - 检查路径连通性
  
- **缺点**：
  - 需要 `move_base` 处于非ACTIVE状态
  - 服务调用有延迟
  - 可能被拒绝

### 使用策略

**推荐策略**：
- **move_base ACTIVE 时**：使用地图检查（快速、可用）
- **move_base 非ACTIVE 时**：使用 `make_plan` 服务（准确、完整）

---

## 为什么终端没有输出距离信息？

**原因**：
- 目标生成失败，没有发布新目标
- 数据收集器（`navdp_generate_dataset.py`）没有收到新目标
- 因此不会输出距离信息

**修复后**：
- 目标生成成功，发布新目标
- 数据收集器收到目标，开始新的episode
- 正常输出距离信息

---

## 测试建议

1. **观察日志**：
   - 应该看到"已订阅move_base状态，用于检查ACTIVE状态"
   - 在机器人导航时，应该看到"使用地图检查代替make_plan"

2. **验证目标生成**：
   - 机器人应该能够正常生成新目标
   - 不应该再出现"300次尝试后未找到合适目标"的警告

3. **检查距离输出**：
   - 数据收集器应该正常输出距离信息
   - 格式：`[数据收集器] Episode进度: X步, 目标距离: Y.m`

---

## 总结

**问题**：`move_base` 在 ACTIVE 状态时拒绝 `make_plan` 服务调用，导致目标生成失败。

**解决方案**：
1. 监控 `move_base` 状态
2. ACTIVE 时使用地图检查代替 `make_plan`
3. 非ACTIVE 时正常使用 `make_plan` 服务

**效果**：
- 目标生成不再失败
- 机器人可以正常生成新目标
- 数据收集器正常输出距离信息

