# MoveBase状态说明：PREEMPTED 含义

## 日志信息含义

**日志**：`=== MoveBase报告导航失败: PREEMPTED，终止episode ===`

**含义**：
- MoveBase 报告导航目标被**抢占（Preempted）**
- 数据收集器检测到此状态，判定为导航失败
- Episode 被终止，数据保存为失败状态

---

## PREEMPTED 状态详解

### 什么是 PREEMPTED？

**PREEMPTED** 是 ROS Action 的一个状态码，表示：
- **目标被新目标抢占**：当发送新目标时，旧目标会被取消（抢占）
- **主动取消**：通过 Action 客户端调用 `cancel_goal()` 取消目标
- **不是真正的失败**：通常不是导航错误，而是被其他操作中断

### PREEMPTED 的常见原因

1. **新目标发布**：
   - 当 `random_generate_goal.py` 发布新目标时
   - move_base 会自动取消（抢占）当前正在执行的目标
   - 开始执行新目标

2. **手动取消**：
   - 通过 Action 客户端调用 `cancel_all_goals()`
   - 代码中的 `_cancel_current_goal()` 方法会触发此状态

3. **目标超时后重新发布**：
   - 如果目标超时，生成器会发布新目标
   - 旧目标会被抢占

---

## 代码中的处理逻辑

### 1. 状态检测（navdp_generate_dataset.py，第317行）

```python
elif status_code in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED]:
    if not self.move_base_failure:
        status_text = self._get_status_text(status_code)
        rospy.loginfo(f"[数据收集器] === MoveBase报告导航失败: {status_text}，终止episode ===")
        self.move_base_failure = True
        # ...
```

**说明**：
- `PREEMPTED` 被归类为**失败状态**
- 与 `ABORTED`（中止）和 `REJECTED`（拒绝）一起处理
- 触发 episode 终止

### 2. Episode 终止（navdp_generate_dataset.py，第558行）

```python
elif self.move_base_failure:
    status_text = self._get_status_text(self.current_goal_status) if self.current_goal_status else "UNKNOWN"
    rospy.loginfo(f"[数据收集器] === MoveBase报告导航失败: {status_text}，终止episode ===")
    return f'move_base_failure_{status_text.lower()}'
```

**说明**：
- 终止原因记录为 `move_base_failure_preempted`
- Episode 被标记为失败（`success=False`）
- 数据仍然会保存

---

## 为什么会出现 PREEMPTED？

### 场景1：到达目标后取消（正常情况）

```python
# random_generate_goal.py - check_goal_reached_or_timeout()
if dist < self.reach_tolerance:
    # 先取消当前move_base目标，防止立即接受新目标
    self._cancel_current_goal()  # ← 这里会触发 PREEMPTED
    rospy.sleep(5)
    self.sample_and_publish_goal()
```

**流程**：
1. 机器人到达目标点
2. `_cancel_current_goal()` 取消当前目标 → 触发 `PREEMPTED`
3. 等待5秒
4. 发布新目标

**结果**：
- Episode 被标记为失败（因为检测到 PREEMPTED）
- 但实际上机器人已经到达目标（成功）
- **这是代码逻辑的问题！**

### 场景2：目标超时后重新发布

```python
# random_generate_goal.py - generate_goal_timer_cb()
if (rospy.Time.now() - self.last_goal_time).to_sec() > self.goal_timeout:
    rospy.logwarn("[目标生成器] 目标超时，重新采样")
    need_new = True
    self.sample_and_publish_goal()  # ← 新目标会抢占旧目标
```

**流程**：
1. 目标超时（30秒未到达）
2. 发布新目标
3. 新目标抢占旧目标 → 触发 `PREEMPTED`

**结果**：
- Episode 被标记为失败（超时）
- 这是合理的失败情况

---

## 问题分析

### 问题：成功到达却被标记为失败

**当前逻辑的问题**：
1. 机器人到达目标点（成功）
2. `_cancel_current_goal()` 取消目标 → `PREEMPTED`
3. 数据收集器检测到 `PREEMPTED` → 判定为失败
4. Episode 被保存为失败状态

**但实际上**：
- 机器人已经成功到达目标
- 应该标记为成功，而不是失败

---

## 修复建议

### 方案1：在取消前检查是否已到达（推荐）

```python
def check_goal_reached_or_timeout(self):
    if self.current_goal is None or self.current_pose_map is None:
        return
    cx = self.current_pose_map.position.x
    cy = self.current_pose_map.position.y
    gx, gy = self.current_goal
    dist = np.hypot(cx - gx, cy - gy)

    if dist < self.reach_tolerance:
        rospy.loginfo("[目标生成器] 机器人已到达目标点")
        
        # 修复：等待一小段时间，让move_base自然报告SUCCEEDED
        rospy.sleep(0.5)  # 等待move_base更新状态
        
        # 然后再取消（如果move_base还没报告成功）
        # 或者不取消，让move_base自然完成
        
        self.current_goal = None
        rospy.sleep(self.goal_wait_time)
        self.sample_and_publish_goal()
```

### 方案2：改进数据收集器的状态判断

在 `navdp_generate_dataset.py` 中，改进成功判断逻辑：

```python
def _check_episode_termination(self, current_pose, lidar_msg):
    # ... 现有代码 ...
    
    # 修复：如果距离目标很近，即使PREEMPTED也判定为成功
    if self.use_move_base_status:
        if self.move_base_success:
            return 'move_base_success'
        elif self.move_base_failure:
            # 检查是否实际上已经到达目标
            if self.current_goal_type == 'point' and self.episode_goal:
                dx = self.episode_goal['x'] - current_pose['x']
                dy = self.episode_goal['y'] - current_pose['y']
                distance = math.hypot(dx, dy)
                
                # 如果距离很近，判定为成功（即使被PREEMPTED）
                if distance < self.reach_tolerance:
                    rospy.loginfo(f"[数据收集器] 虽然PREEMPTED，但距离目标{distance:.3f}m，判定为成功")
                    return 'move_base_success_near_goal'
            
            status_text = self._get_status_text(self.current_goal_status)
            return f'move_base_failure_{status_text.lower()}'
```

### 方案3：延迟取消目标

```python
def check_goal_reached_or_timeout(self):
    if dist < self.reach_tolerance:
        rospy.loginfo("[目标生成器] 机器人已到达目标点")
        self.current_goal = None
        
        # 修复：等待move_base报告SUCCEEDED，再取消
        rospy.sleep(1.0)  # 给move_base时间报告成功
        
        # 然后取消（如果需要）
        self._cancel_current_goal()
        
        rospy.sleep(self.goal_wait_time)
        self.sample_and_publish_goal()
```

---

## MoveBase 状态码完整说明

| 状态码 | 含义 | 是否失败 | 说明 |
|--------|------|----------|------|
| **PENDING** | 等待执行 | ❌ | 目标已接收，等待执行 |
| **ACTIVE** | 正在执行 | ❌ | 正在导航中 |
| **PREEMPTED** | 被抢占 | ⚠️ | 被新目标或取消操作中断 |
| **SUCCEEDED** | 成功 | ✅ | 成功到达目标 |
| **ABORTED** | 中止 | ❌ | 导航失败（如碰撞、无法规划） |
| **REJECTED** | 拒绝 | ❌ | 目标被拒绝（如无效） |
| **PREEMPTING** | 抢占中 | ❌ | 正在被抢占 |
| **RECALLING** | 召回中 | ❌ | 正在召回目标 |
| **RECALLED** | 已召回 | ❌ | 目标已被召回 |
| **LOST** | 丢失 | ❌ | 目标状态丢失 |

---

## 当前代码的处理

### 成功状态
- `SUCCEEDED` → Episode 标记为成功 ✅

### 失败状态
- `PREEMPTED` → Episode 标记为失败 ❌（可能误判）
- `ABORTED` → Episode 标记为失败 ❌
- `REJECTED` → Episode 标记为失败 ❌

### 问题
- 如果机器人已经到达目标，但目标被取消（PREEMPTED），会被误判为失败

---

## 建议的修复

**推荐方案2**：在数据收集器中改进判断逻辑，如果距离目标很近（<容忍度），即使PREEMPTED也判定为成功。

这样可以：
1. 避免误判成功为失败
2. 保持数据质量
3. 提高成功率统计的准确性

