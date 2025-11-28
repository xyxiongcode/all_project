# Episode文件夹更新问题修复

## 问题描述

**用户反馈**：机器人已经走了好几条路径，但数据只在一个episode文件夹里更新。

## 问题原因

### 核心问题：状态重置时机不正确

在原来的代码中，`_end_episode()` 的执行顺序是：

1. 保存本地变量（episode_id, episode_data等）
2. **执行耗时操作**（保存数据、生成样本等）
3. **最后才重置状态**（`current_episode_state`, `current_episode_id`等）

**问题**：
- 如果保存操作耗时（比如数据量大、磁盘IO慢），新目标到达时：
  - `current_episode_state` 仍然是 `COLLECTING`
  - `current_episode_id` 仍然是旧的ID
  - 新目标会被强制结束旧episode，但新episode可能仍使用旧的ID
  - 或者新目标被忽略，数据继续写入旧episode文件夹

---

## 修复方案

### 修改：提前重置状态

在 `_end_episode()` 中，**在所有关键变量保存到本地变量后，立即重置状态**。

**新的执行顺序**：

```python
def _end_episode(self, termination_reason):
    # 1. 保存所有关键变量到本地（防止被覆盖）
    episode_id_to_save = self.current_episode_id
    episode_data_to_save = list(self.episode_data)
    # ...
    
    # 2. ✅ 立即重置状态（新修改）
    self.current_episode_state = self.episode_states['WAITING_GOAL']
    self.current_episode_id = None  # 立即清空
    self.episode_data = []  # 清空数据
    
    # 3. 使用本地变量执行耗时操作（保存数据等）
    self._save_episode_data(...)  # 使用 episode_id_to_save, episode_data_to_save
    # ...
```

**优点**：
- ✅ 新目标到达时，状态已经是 `WAITING_GOAL`
- ✅ `current_episode_id` 已清空，新episode会生成新的ID
- ✅ 可以立即开始新episode，不受旧episode保存操作影响
- ✅ 旧episode数据仍在后台使用本地变量保存，不会丢失

---

## 修改位置

### 文件：`navdp_generate_dataset.py`

#### 修改1：在 `_end_episode()` 开始时立即重置状态

**位置**：第993-1000行（原位置）

**修改前**：
```python
# 检查episode是否有效
if episode_id_to_save is None:
    # ...
    
# 检查数据是否足够
# ...保存操作...
# 最后才重置状态（第1185行）
```

**修改后**：
```python
# ===== 修复：立即重置状态，防止新目标到达时干扰 =====
old_episode_id = self.current_episode_id
self.current_episode_state = self.episode_states['WAITING_GOAL']
self.current_episode_id = None  # 立即清空
self.episode_data = []  # 清空数据

# 检查episode是否有效
if episode_id_to_save is None:
    # ...
    
# 检查数据是否足够
# ...保存操作（使用本地变量）...
```

#### 修改2：移除末尾的重复状态重置

**位置**：第1184-1187行

**修改前**：
```python
# 重置状态
self.current_episode_state = self.episode_states['WAITING_GOAL']
self.episode_data = []
self.current_episode_id = None
```

**修改后**：
```python
# ===== 注意：状态已在函数开始时重置，这里只重置其他相关变量 =====
# （状态重置代码已移除）
```

---

## Episode终止逻辑总结

### Episode终止的两种方式

#### 1. 正常终止（在 `sync_callback()` 中检测）

```python
# 在 sync_callback() 中
termination_reason = self._check_episode_termination(robot_pose, lidar_msg)
if termination_reason:
    self._end_episode(termination_reason)
```

**触发条件**：
- MoveBaseActionServer报告成功（`move_base_success == True`）
- MoveBaseActionServer报告失败（`move_base_failure == True`）
- 达到最大步数
- 碰撞或停滞

#### 2. 强制终止（收到新目标时）

```python
# 在 point_goal_callback() 中
if self.current_episode_state != WAITING_GOAL:
    # 检查旧episode是否已成功
    if self.current_episode_success:
        termination_reason = 'success_before_new_goal_...'
    else:
        termination_reason = 'forced_end_due_to_new_goal'
    
    self._end_episode(termination_reason)  # 立即结束旧episode
    # 继续处理新目标（不return）
```

---

## 预期效果

修复后：

1. ✅ **每次收到新目标时，都会创建新的episode文件夹**
   - 因为状态已重置，`_start_new_episode()` 会生成新的ID

2. ✅ **新episode不会使用旧的episode_id**
   - `current_episode_id` 已在 `_end_episode()` 开始时清空

3. ✅ **旧episode数据不会丢失**
   - 所有关键变量已保存到本地变量中
   - 保存操作使用本地变量，不受状态重置影响

4. ✅ **多个路径会创建多个episode文件夹**
   - 每个路径对应一个独立的episode文件夹

---

## 验证方法

### 1. 检查日志

查找以下日志：

```
[数据收集器] ✓ Episode状态已重置为WAITING_GOAL（Episode {id}的数据将在后台保存）
[数据收集器] === 开始Episode {counter} ===
[数据收集器] Episode ID: {new_id}
```

**确认**：
- 每个新episode都有新的ID
- 状态在保存操作前已重置

### 2. 检查episode文件夹

```bash
ls -la <save_dir>/train/episodes/
```

**确认**：
- 有多个 `episode_*` 文件夹
- 每个文件夹对应一个导航路径

### 3. 检查metadata.json

查看每个episode文件夹的 `metadata.json`：

```json
{
  "episode_id": 1234567890,
  "total_steps": 500,
  "success": true,
  "termination_reason": "move_base_success"
}
```

**确认**：
- 每个episode有不同的 `episode_id`
- `total_steps` 与预期一致（对应一个路径）
- `termination_reason` 合理（不应该都是 `forced_end_due_to_new_goal`）

---

## 注意事项

1. **状态重置的时机很重要**
   - 必须在所有关键变量保存到本地后立即重置
   - 但不能在保存变量前重置，否则会丢失数据

2. **保存操作使用本地变量**
   - 所有保存操作必须使用 `episode_id_to_save`, `episode_data_to_save` 等本地变量
   - 不能直接使用 `self.current_episode_id` 等实例变量（因为已被重置）

3. **新目标到达时可能仍在保存**
   - 即使旧episode还在后台保存，新episode也可以立即开始
   - 这是预期行为，因为保存操作使用本地变量，不受影响

---

## 相关文件

- `navdp_generate_dataset.py`：主要修改文件
- `EPISODE_TERMINATION_LOGIC_ANALYSIS.md`：详细的终止逻辑分析

