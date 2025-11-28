# Episode状态判断和Termination Reason逻辑详解

## 一、Episode状态管理

### 1.1 Episode状态定义

在 `navdp_generate_dataset.py` 中，episode有以下四种状态：

```python
self.episode_states = {
    'WAITING_GOAL': 0,    # 等待目标（初始状态）
    'COLLECTING': 1,      # 正在收集数据
    'COMPLETED': 2,       # 已完成（已保存）
    'FAILED': 3           # 已失败（已保存）
}
```

### 1.2 状态转换流程

```
[WAITING_GOAL] 
    ↓ (收到新目标 via /move_base/goal)
[COLLECTING] 
    ↓ (检查终止条件)
[COMPLETED] 或 [FAILED]
    ↓ (数据保存完成)
[WAITING_GOAL] (回到初始状态，准备接收下一个目标)
```

---

## 二、Episode终止条件判断（termination_reason）

### 2.1 主要判断逻辑

`termination_reason` 在 `_check_episode_termination()` 函数中判断，该函数在每次 `sync_callback()` 时被调用。

**函数位置**：`navdp_generate_dataset.py` 第750行

### 2.2 终止条件优先级

#### 优先级1：MoveBase Action状态（主要判断）

这是**最优先**的判断方式，基于 `MoveBaseActionServer` 的状态：

```python
def _check_episode_termination(self, current_pose, lidar_msg):
    # 1. 检查最小步数要求
    if len(self.episode_data) < self.min_episode_steps:
        if self.use_move_base_action and self.move_base_success:
            return 'move_base_success_early'  # 即使步数少，但已成功
        return None  # 继续收集
    
    # 2. 检查move_base Action状态（主要判断）
    if self.use_move_base_action:
        self._check_move_base_status()  # 更新状态
        
        if self.move_base_success:
            return 'move_base_success'  # ✅ 成功到达
        
        elif self.move_base_failure:
            status = self.current_goal_status
            
            # 特殊处理PREEMPTED：检查是否实际上已到达
            if status == GoalStatus.PREEMPTED:
                distance = math.hypot(
                    self.episode_goal['x'] - current_pose['x'],
                    self.episode_goal['y'] - current_pose['y']
                )
                if distance < self.reach_tolerance:
                    return 'move_base_success_preempted_near_goal'  # ✅ 成功（被取消但已到达）
                return 'move_base_failure_preempted'  # ❌ 失败
            
            # 其他失败状态
            return f'move_base_failure_{status_text.lower()}'  # ❌ 失败
```

#### 优先级2：次要检查（安全相关）

只有在move_base状态不明确时才检查：

- **最大步数**：达到 `max_episode_steps`（默认10000步）
  - `termination_reason = 'max_steps'`
- **碰撞**：LiDAR检测到障碍物 < 0.3米
  - `termination_reason = 'collision'`
- **停滞**：机器人15秒未移动
  - `termination_reason = 'stuck'`

---

## 三、MoveBase Action状态检查机制

### 3.1 状态检查函数

**函数位置**：`navdp_generate_dataset.py` 第316行

```python
def _check_move_base_status(self):
    """检查move_base状态（通过Action Client）"""
    if self.move_base_action_client is None:
        return
    
    state = self.move_base_action_client.get_state()  # 获取状态
    self.current_goal_status = state
    
    if state == GoalStatus.SUCCEEDED:
        # ✅ 成功到达
        self.move_base_success = True
        self.move_base_failure = False
        
    elif state == GoalStatus.ACTIVE:
        # 🔄 正在执行，继续收集
        self.move_base_success = False
        self.move_base_failure = False
        
    elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED]:
        # ❌ 失败状态
        
        if state == GoalStatus.PREEMPTED:
            # 特殊处理：检查距离，可能实际已到达
            distance = ...  # 计算到目标距离
            if distance < self.reach_tolerance:
                self.move_base_success = True  # 判定为成功
                return
        
        self.move_base_failure = True
        self.move_base_success = False
```

### 3.2 状态码说明

| 状态码 | 含义 | 处理方式 |
|--------|------|----------|
| `SUCCEEDED` | 目标成功到达 | ✅ 设置 `move_base_success = True` |
| `ACTIVE` | 正在执行导航 | 🔄 继续收集数据 |
| `PREEMPTED` | 目标被取消 | ⚠️ 检查距离，如果很近判定为成功 |
| `ABORTED` | 导航失败 | ❌ 设置 `move_base_failure = True` |
| `REJECTED` | 目标被拒绝 | ❌ 设置 `move_base_failure = True` |
| `PENDING` | 等待执行 | 🔄 继续等待 |

### 3.3 调用时机

`_check_move_base_status()` 在以下时机被调用：

1. **每次 `sync_callback()` 时**（第1078行）
   - 传感器数据同步回调
   - 通常10-30Hz频率

2. **收到新目标时**（第491行）
   - 检查旧episode是否已成功

3. **Episode终止检查时**（第707行）
   - 在 `_check_episode_termination()` 中调用

---

## 四、成功/失败判断（在_end_episode中）

### 4.1 判断逻辑

**函数位置**：`navdp_generate_dataset.py` 第910行

```python
def _end_episode(self, termination_reason):
    # 判断是否成功
    success = False
    
    # 优先级1：基于move_base Action状态
    if self.use_move_base_action:
        success = self.move_base_success
    
    # 优先级2：基于termination_reason中的"success"标识
    if 'success' in termination_reason.lower():
        success = True
    
    # 保存episode数据，success字段用于标记
    metadata = {
        'success': success,
        'termination_reason': termination_reason,
        ...
    }
```

### 4.2 Termination Reason与Success的映射

| termination_reason | 含义 | success |
|-------------------|------|---------|
| `'move_base_success'` | MoveBase报告成功 | ✅ True |
| `'move_base_success_early'` | 成功但步数较少 | ✅ True |
| `'move_base_success_preempted_near_goal'` | 被取消但已到达 | ✅ True |
| `'success_before_new_goal'` | 收到新目标前已成功 | ✅ True |
| `'success_by_distance_before_new_goal'` | 距离检查成功 | ✅ True |
| `'move_base_failure_aborted'` | 导航失败 | ❌ False |
| `'move_base_failure_rejected'` | 目标被拒绝 | ❌ False |
| `'move_base_failure_preempted'` | 被取消且未到达 | ❌ False |
| `'forced_end_due_to_new_goal'` | 被新目标强制结束 | ❌ False |
| `'max_steps'` | 达到最大步数 | ❌ False |
| `'collision'` | 碰撞 | ❌ False |
| `'stuck'` | 停滞 | ❌ False |

---

## 五、random_generate_goal.py 中的目标发送逻辑

### 5.1 MoveBaseActionServer使用流程

#### 步骤1：初始化Action Client

**位置**：`random_generate_goal.py` 第112-133行

```python
def __init__(self):
    # 创建Action Client
    self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    
    # 等待Action Server启动（最多5秒）
    if self.move_base_client.wait_for_server(rospy.Duration(5.0)):
        rospy.loginfo("[目标生成器] 已连接 move_base Action Server")
    else:
        rospy.logerr("[目标生成器] 未连接到 Action Server")
        self.move_base_client = None
```

#### 步骤2：生成随机目标坐标

**位置**：`random_generate_goal.py` 第416-453行

```python
def generate_random_goal(self):
    """生成随机目标坐标"""
    # 1. 获取当前位置
    cx = self.current_pose_map.position.x
    cy = self.current_pose_map.position.y
    
    # 2. 循环300次尝试
    for attempt in range(300):
        # 2.1 生成随机角度和距离
        angle = random.uniform(0.0, 2.0 * np.pi)
        dist = random.uniform(self.min_distance, self.max_distance)
        
        # 2.2 计算目标坐标
        gx = cx + dist * np.cos(angle)
        gy = cy + dist * np.sin(angle)
        
        # 2.3 验证目标
        if not self.is_within_map(gx, gy):
            continue  # 不在地图内，跳过
        
        obstacle_distance = self.get_min_obstacle_distance(gx, gy)
        if obstacle_distance < self.min_obstacle_distance:
            continue  # 太靠近障碍物，跳过
        
        # 2.4 检查可达性（调用/move_base/make_plan服务）
        if not self.is_reachable_from_current(gx, gy):
            continue  # 不可达，跳过
        
        # 2.5 找到有效目标
        return gx, gy
    
    return None  # 300次都失败
```

#### 步骤3：创建并发送MoveBaseGoal

**位置**：`random_generate_goal.py` 第600-679行

```python
def sample_and_publish_goal(self):
    """生成并发送目标"""
    # 1. 生成随机目标坐标
    goal = self.generate_random_goal()
    if goal is None:
        rospy.logwarn("无法找到合适的随机目标")
        return
    
    goal_x, goal_y = goal
    
    # 2. 计算目标朝向（朝向机器人当前位置）
    yaw = np.arctan2(
        goal_y - self.current_pose_map.position.y,
        goal_x - self.current_pose_map.position.x
    )
    
    # 3. 创建PoseStamped消息（目标位姿）
    goal_msg = PoseStamped()
    goal_msg.header = Header(
        stamp=rospy.Time.now(),
        frame_id=self.map_frame  # 使用'map'坐标系
    )
    goal_msg.pose.position.x = goal_x
    goal_msg.pose.position.y = goal_y
    goal_msg.pose.position.z = 0.0
    # 转换为四元数
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    goal_msg.pose.orientation.x = qx
    goal_msg.pose.orientation.y = qy
    goal_msg.pose.orientation.z = qz
    goal_msg.pose.orientation.w = qw
    
    # 4. 创建MoveBaseGoal并发送（关键步骤）
    if self.move_base_client is not None:
        try:
            action_goal = MoveBaseGoal()
            action_goal.target_pose = goal_msg  # 设置目标位姿
            
            # 通过Action Client发送目标
            self.move_base_client.send_goal(action_goal)
            
            # 保存goal handle（用于后续取消等操作）
            self.current_goal_handle = action_goal
            
            rospy.loginfo("[目标生成器] 已通过Action Client发送目标到move_base")
            
        except Exception as e:
            rospy.logerr(f"发送目标失败: {e}")
            return
    
    # 5. 更新内部状态
    self.current_goal = (goal_x, goal_y)
    self.last_goal_time = rospy.Time.now()
    self.goal_reached = False
    self.goal_failed = False
```

#### 步骤4：检查目标状态

**位置**：`random_generate_goal.py` 第295-414行

```python
def check_goal_reached_or_timeout(self):
    """检查目标是否到达或超时"""
    if self.current_goal is None:
        return
    
    if self.move_base_client is not None:
        # 获取Action状态
        state = self.move_base_client.get_state()
        
        if state == GoalStatus.SUCCEEDED:
            # ✅ 成功到达
            rospy.loginfo("[目标生成器] 机器人已到达目标点")
            
            # 取消当前目标（确保move_base停止）
            self._cancel_current_goal()
            
            # 更新状态
            self.current_goal = None
            self.goal_reached = True
            
            # 等待5秒后发送新目标
            rospy.sleep(self.goal_wait_time)
            self.sample_and_publish_goal()
            
        elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
            # ❌ 导航失败
            rospy.logwarn("[目标生成器] 目标失败")
            
            # 检查距离（可能实际已到达）
            dist = np.hypot(...)
            if dist < self.reach_tolerance:
                # 判定为成功
                ...
            else:
                # 确实失败，发送新目标
                self._cancel_current_goal()
                rospy.sleep(1.0)
                self.sample_and_publish_goal()
        
        elif state == GoalStatus.PREEMPTED:
            # ⚠️ 目标被取消
            dist = np.hypot(...)
            if dist < self.reach_tolerance:
                # 实际已到达，判定为成功
                ...
```

### 5.2 目标发送流程图

```
[定时器触发 / 目标到达]
    ↓
generate_random_goal()
    ├─ 随机采样坐标 (gx, gy)
    ├─ 验证：地图范围 ✓
    ├─ 验证：障碍物距离 ✓
    └─ 验证：路径可达性 (make_plan) ✓
    ↓
sample_and_publish_goal()
    ├─ 创建 PoseStamped (目标位姿)
    ├─ 创建 MoveBaseGoal
    ├─ move_base_client.send_goal(action_goal)  ← 发送到 /move_base Action Server
    └─ 更新内部状态
    ↓
[/move_base/goal topic 自动发布 MoveBaseActionGoal]  ← 数据收集器订阅此topic
    ↓
[navdp_generate_dataset.py 接收到目标]
    ├─ move_base_action_goal_callback()
    └─ point_goal_callback()
    ↓
[开始新的episode]
```

### 5.3 关键点说明

#### 1. Action Client vs Topic

- **Action Client** (`move_base_client.send_goal()`)：
  - 发送目标到 `MoveBaseActionServer`
  - 可以获取状态反馈（`get_state()`）
  - 可以取消目标（`cancel_goal()`）
  
- **Topic自动发布**：
  - `move_base` 收到Action Goal后，会自动发布到 `/move_base/goal` topic
  - 消息类型：`MoveBaseActionGoal`
  - 数据收集器订阅此topic接收目标

#### 2. 坐标系

- **目标坐标系**：`map` 坐标系
- **目标位置**：`(goal_x, goal_y)` 在地图中的绝对坐标
- **目标朝向**：朝向机器人当前位置的方向

#### 3. 状态检查频率

- **目标生成器**：每次 `odom_callback()` 时检查（通常10-20Hz）
- **数据收集器**：每次 `sync_callback()` 时检查（通常10-30Hz）

---

## 六、总结

### 6.1 Episode状态判断流程

```
收到目标 → WAITING_GOAL → COLLECTING
                              ↓
                    [持续检查终止条件]
                              ↓
                    ┌─────────┴─────────┐
                    ↓                   ↓
              move_base_success    move_base_failure
                    ↓                   ↓
                 COMPLETED            FAILED
                    ↓                   ↓
                 [保存数据]
                    ↓
              WAITING_GOAL (准备下一个目标)
```

### 6.2 成功判断优先级

1. **MoveBase Action状态**（最优先）
   - `GoalStatus.SUCCEEDED` → 成功
   - `GoalStatus.PREEMPTED` + 距离检查 → 可能成功
   
2. **Termination Reason标识**
   - 包含 "success" 的字符串 → 成功
   
3. **距离检查**（备用）
   - 距离 < `reach_tolerance` (0.5m) → 可能成功

### 6.3 关键代码位置

| 功能 | 文件 | 函数/位置 |
|------|------|-----------|
| Episode状态管理 | `navdp_generate_dataset.py` | `__init__()` 第94-100行 |
| 终止条件检查 | `navdp_generate_dataset.py` | `_check_episode_termination()` 第750行 |
| MoveBase状态检查 | `navdp_generate_dataset.py` | `_check_move_base_status()` 第316行 |
| 成功/失败判断 | `navdp_generate_dataset.py` | `_end_episode()` 第910行 |
| 目标发送 | `random_generate_goal.py` | `sample_and_publish_goal()` 第600行 |
| 状态检查 | `random_generate_goal.py` | `check_goal_reached_or_timeout()` 第295行 |

---

## 七、注意事项

### 7.1 MoveBase Action状态是主要判断依据

- ✅ **优先使用** `move_base_action_client.get_state()`
- ✅ **状态码 `SUCCEEDED` = 成功**
- ⚠️ **状态码 `PREEMPTED` 需要距离检查**

### 7.2 状态检查时机

- 数据收集器在每次传感器数据回调时检查
- 确保状态及时更新

### 7.3 坐标系一致性

- 目标位置和机器人位置都使用 `map` 坐标系
- 通过TF查询确保一致性

### 7.4 新目标到达时的处理

- 先检查旧episode是否已成功
- 如果已成功，标记为成功结束
- 如果未成功，标记为失败

---

此文档详细说明了episode状态判断和termination_reason的逻辑，以及如何使用MoveBaseActionServer来判断机器人是否成功到达目标。

