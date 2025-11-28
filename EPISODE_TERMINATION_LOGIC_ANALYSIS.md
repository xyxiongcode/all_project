# Episode终止逻辑分析

## 问题描述

用户反馈：机器人已经走了好几条路径，但数据只在一个episode文件夹里更新。

## Episode终止逻辑

### 1. Episode终止触发条件

Episode可以在以下情况终止：

#### a) 在 `sync_callback()` 中检测到终止条件

**位置**：`sync_callback()` 第1409行

```python
termination_reason = self._check_episode_termination(robot_pose, lidar_msg)
if termination_reason:
    self._end_episode(termination_reason)
```

**终止条件**（在 `_check_episode_termination()` 中检查）：

1. **MoveBase成功**（`move_base_success == True`）
   - 返回：`'move_base_success'`

2. **MoveBase失败**（`move_base_failure == True`）
   - 返回：`'move_base_failure_xxx'`

3. **达到最大步数**（`len(episode_data) >= max_episode_steps`）
   - 返回：`'max_steps_forced'`

4. **碰撞**（LiDAR检测到碰撞）
   - 返回：`'collision'`

5. **停滞**（机器人长时间不移动）
   - 返回：`'stuck'`

#### b) 收到新目标时强制终止

**位置**：`point_goal_callback()` 第586-636行

```python
if self.current_episode_state != self.episode_states['WAITING_GOAL']:
    # 当前正在收集数据，收到新目标
    if len(self.episode_data) > 0:
        # 检查是否已成功
        if self.current_episode_success:
            termination_reason = f'success_before_new_goal_{self.current_episode_success_reason}'
        elif self.move_base_success:
            termination_reason = 'success_before_new_goal_move_base'
        else:
            termination_reason = 'forced_end_due_to_new_goal'
        
        self._end_episode(termination_reason)
    else:
        # 数据为空，直接重置状态
        self.current_episode_state = self.episode_states['WAITING_GOAL']
        self.current_episode_id = None
```

---

## 问题分析

### 问题1：Episode未正确终止

**可能原因**：

1. **MoveBaseActionServer未报告状态**
   - 如果 `move_base_success` 和 `move_base_failure` 都没有被设置为 `True`
   - `_check_episode_termination()` 会返回 `None`
   - Episode会继续收集数据，不会终止

2. **终止检查时机问题**
   - `_check_episode_termination()` 在 `sync_callback()` 中调用
   - 如果传感器数据没有同步，`sync_callback()` 不会被调用
   - 终止条件就无法被检查

3. **状态未正确更新**
   - 如果MoveBaseActionServer的反馈话题订阅失败
   - 状态不会被更新
   - 终止条件不会被触发

### 问题2：多个路径只更新一个episode文件夹

**可能原因**：

1. **`_end_episode()` 执行时间过长**
   - `_end_episode()` 需要保存数据、生成样本等，可能耗时较长
   - 如果新目标在 `_end_episode()` 完成前到达，状态可能还没有重置为 `WAITING_GOAL`
   - 新目标可能被忽略，或者旧的episode_id被继续使用

2. **状态重置时机问题**
   - `_end_episode()` 在**保存数据之前**就重置了状态（第1185行）
   - 但如果保存过程中出错，状态可能已经被重置
   - 导致新目标到达时，状态已经是 `WAITING_GOAL`，但旧的episode数据还在保存

3. **Episode ID未正确清空**
   - `current_episode_id` 在 `_end_episode()` 末尾才被清空（第1187行）
   - 如果新目标在清空前到达，可能仍使用旧的episode_id

---

## 关键代码位置

### `_end_episode()` 的执行顺序

```python
def _end_episode(self, termination_reason):
    # 1. 保存本地变量（防止新目标到达时被覆盖）
    episode_id_to_save = self.current_episode_id
    episode_data_to_save = self.episode_data.copy()
    # ...
    
    # 2. 保存数据（耗时操作）
    self._save_episode_data(...)
    # ...
    
    # 3. 生成训练样本（耗时操作）
    training_samples = self._generate_training_samples_from_data(...)
    # ...
    
    # 4. 重置状态（在所有保存操作完成后）
    self.current_episode_state = self.episode_states['WAITING_GOAL']
    self.episode_data = []
    self.current_episode_id = None  # 清空episode ID
```

**问题**：状态重置在保存操作**之后**，但如果保存操作耗时，新目标可能在状态重置前到达。

---

## 解决方案

### 方案1：确保状态在保存前就重置

**修改**：在 `_end_episode()` 开始时立即重置状态（但要保存本地变量）

```python
def _end_episode(self, termination_reason):
    # 立即保存关键变量
    episode_id_to_save = self.current_episode_id
    episode_data_to_save = self.episode_data.copy()
    # ...
    
    # 立即重置状态（防止新目标到达时干扰）
    self.current_episode_state = self.episode_states['WAITING_GOAL']
    self.current_episode_id = None  # 立即清空，防止新目标使用旧ID
    
    # 然后进行保存操作（使用本地变量）
    # ...
```

**优点**：新目标到达时，状态已经是 `WAITING_GOAL`，可以立即开始新episode

**缺点**：如果保存失败，状态已经重置，可能丢失数据

### 方案2：添加episode锁定机制

**修改**：在保存过程中添加锁定标志，防止新目标干扰

```python
def _end_episode(self, termination_reason):
    # 设置锁定标志
    self.episode_saving = True
    
    # 保存数据
    # ...
    
    # 重置状态
    self.current_episode_state = self.episode_states['WAITING_GOAL']
    self.current_episode_id = None
    
    # 清除锁定标志
    self.episode_saving = False

def point_goal_callback(self, msg):
    # 如果正在保存，等待或拒绝新目标
    if self.episode_saving:
        rospy.logwarn("[数据收集器] Episode正在保存中，等待完成...")
        # 等待或返回
```

**优点**：防止保存过程中新目标干扰

**缺点**：可能延迟新episode的开始

### 方案3：异步保存

**修改**：将保存操作放在后台线程中，主线程立即重置状态

```python
import threading

def _end_episode(self, termination_reason):
    # 保存变量
    episode_id_to_save = self.current_episode_id
    episode_data_to_save = self.episode_data.copy()
    # ...
    
    # 立即重置状态
    self.current_episode_state = self.episode_states['WAITING_GOAL']
    self.current_episode_id = None
    
    # 在后台线程中保存
    save_thread = threading.Thread(
        target=self._save_episode_data_async,
        args=(episode_id_to_save, episode_data_to_save, ...)
    )
    save_thread.start()
```

**优点**：状态立即重置，不影响新episode开始

**缺点**：需要处理线程同步问题

---

## 推荐方案

**推荐使用方案1**：在 `_end_episode()` 开始时立即重置状态，但确保所有关键变量都已保存到本地变量中。

这样可以确保：
1. 新目标到达时，状态已经是 `WAITING_GOAL`
2. 可以立即开始新episode
3. 旧的episode数据仍在后台保存

---

## 诊断建议

1. **检查日志**：
   - 查找 `[数据收集器] === 收到新的点目标 ===` 日志
   - 检查状态是否已经是 `WAITING_GOAL`
   - 检查episode ID是否已清空

2. **检查episode文件夹**：
   - 查看每个episode文件夹的 `metadata.json`
   - 确认 `total_steps` 是否与预期一致
   - 确认是否有多个episode文件夹

3. **检查终止原因**：
   - 查看 `termination_reason`
   - 确认是否都是 `forced_end_due_to_new_goal`（表示未正常终止）

