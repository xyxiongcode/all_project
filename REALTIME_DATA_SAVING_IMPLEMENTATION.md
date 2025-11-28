# 实时数据保存实现说明

## 问题描述

用户反馈的问题：
1. 机器人移动时，episode文件夹内没有生成任何数据
2. 机器人到达目标点后发布下一个目标点，在机器人走的过程中，前一个episode内不断更新文件
3. 生成很多空的episode文件夹

**用户需求**：
- 接收到一个目标点就生成一个episode文件夹
- 随着机器人移动，episode文件夹内数据**实时更新**（不是定时，而是随着机器人移动逐步保存）
- 直到机器人接收到下一个新的目标点前，前一条episode文件夹更新完成

## 问题分析

### 原有逻辑的问题

**原来的数据保存方式**：
- 数据收集时：只将数据点追加到 `self.episode_data` 列表中
- Episode结束时：在 `_end_episode()` 中批量保存所有数据点到磁盘

**导致的问题**：
1. 机器人移动时，episode文件夹是空的（因为数据只在episode结束时才保存）
2. 如果episode还未结束，数据不会被写入磁盘
3. 无法实时查看数据收集进度

## 解决方案

### 修改：实时保存模式

**新的数据保存方式**：
- 数据收集时：在 `sync_callback()` 中，**每收集到一个数据点，立即保存到磁盘**
- Episode结束时：只需更新 `metadata.json`，不需要重新保存数据

### 具体修改

#### 1. 在 `_start_new_episode()` 中

- 添加了 `self.saved_data_point_count = 0`（已保存数据点计数）
- 添加了 `self.episode_dir`（当前episode目录路径）
- 创建初始 `metadata.json`（包含episode基本信息，total_steps=0，success=False等）

```python
# ===== 实时保存相关变量 =====
self.saved_data_point_count = 0  # 已保存到磁盘的数据点数量（用于实时保存）
self.episode_dir = None  # 当前episode目录路径（用于实时保存）

# 创建初始metadata.json
self._create_initial_metadata()
```

#### 2. 在 `sync_callback()` 中

- 每收集到一个数据点，立即调用 `_save_single_step_realtime()` 保存到磁盘
- 每100个数据点更新一次 `metadata.json` 中的 `total_steps`

```python
# ===== 修复：实时保存数据点到磁盘 =====
if self.current_episode_id is not None and self.episode_dir is not None:
    step_index = len(self.episode_data) - 1
    self._save_single_step_realtime(self.episode_dir, step_index, data_point)
    self.saved_data_point_count = len(self.episode_data)
    
    # 每100个数据点更新一次metadata
    if (step_index + 1) % 100 == 0:
        self._update_metadata_realtime()
        rospy.loginfo(f"[数据收集器] ✓ 已实时保存 {step_index + 1} 个数据点到episode文件夹")
```

#### 3. 新增函数

**`_create_initial_metadata()`**：
- 在episode开始时创建初始 `metadata.json`
- 包含episode基本信息，但 `total_steps=0`，`success=False`，`end_time=None` 等字段将在episode结束时更新

**`_save_single_step_realtime()`**：
- 实时保存单个数据点到磁盘
- 创建 `step_XXXX` 目录，保存 `rgb.png`、`depth.npy`、`data.json`

**`_update_metadata_realtime()`**：
- 实时更新 `metadata.json` 中的 `total_steps` 字段

**`_update_episode_metadata()`**：
- Episode结束时，更新 `metadata.json` 的最终信息（`end_time`、`duration`、`success`、`termination_reason` 等）

#### 4. 修改 `_end_episode()`

- 优先使用实时保存模式：如果 `old_episode_dir` 存在，只需更新 `metadata.json`
- Fallback：如果目录不存在，使用传统方式批量保存

```python
# 使用实时保存的episode目录，如果存在则只需更新metadata
if old_episode_dir and os.path.exists(old_episode_dir):
    # 数据已实时保存，只需更新metadata
    episode_path = self._update_episode_metadata(...)
else:
    # Fallback：使用传统保存方式
    episode_path = self._save_episode_data(...)
```

#### 5. 状态重置

- 在 `_end_episode()` 开始时立即重置状态，保存 `old_episode_dir` 用于更新metadata
- 确保新目标到达时可以立即开始新episode

```python
old_episode_id = self.current_episode_id
old_episode_dir = self.episode_dir  # 保存episode目录路径
self.current_episode_state = self.episode_states['WAITING_GOAL']
self.current_episode_id = None
self.episode_data = []
self.episode_dir = None
self.saved_data_point_count = 0
```

---

## 实现效果

### 数据保存流程

1. **收到新目标** → 创建episode文件夹 + 初始 `metadata.json`
2. **机器人移动** → 每收集到一个数据点，立即保存到 `step_XXXX` 目录
3. **每100个数据点** → 更新 `metadata.json` 中的 `total_steps`
4. **Episode结束** → 只需更新 `metadata.json` 的最终信息（成功/失败、结束时间等）

### 优势

1. ✅ **实时可见**：可以在episode进行中查看已保存的数据点
2. ✅ **数据不丢失**：即使程序意外终止，已保存的数据也不会丢失
3. ✅ **性能优化**：Episode结束时只需更新metadata，不需要重新保存所有数据
4. ✅ **状态清晰**：每个episode的数据独立，不会互相干扰

---

## 文件结构

每个episode文件夹的结构：

```
episode_1234567890/
├── metadata.json          # Episode元数据（实时更新）
├── step_0000/            # 第0个数据点（实时保存）
│   ├── rgb.png
│   ├── depth.npy
│   └── data.json
├── step_0001/            # 第1个数据点（实时保存）
│   ├── rgb.png
│   ├── depth.npy
│   └── data.json
├── step_0002/            # 第2个数据点（实时保存）
│   └── ...
└── goal_image.png        # 目标图像（如果捕获）
```

---

## 注意事项

1. **磁盘IO**：实时保存会增加磁盘IO，但对于数据收集来说这是必要的
2. **性能**：每个数据点的保存是同步的，如果保存失败会记录警告但不会中断数据收集
3. **Metadata更新**：每100个数据点更新一次 `total_steps`，避免频繁IO
4. **向后兼容**：如果episode目录不存在（fallback情况），仍然使用传统批量保存方式

---

## 相关文件

- `navdp_generate_dataset.py`：主要修改文件
- `random_generate_goal.py`：无需修改（目标发送逻辑已正确）

---

## 验证方法

1. **检查日志**：
   - 查找 `[数据收集器] ✓ 已实时保存 X 个数据点到episode文件夹`
   - 查找 `[数据收集器] ✓ 已创建初始metadata.json`

2. **检查文件夹**：
   ```bash
   ls -la <save_dir>/train/episodes/episode_*/
   ```
   - 应该能看到 `step_0000/`、`step_0001/` 等目录实时出现
   - `metadata.json` 应该实时更新 `total_steps`

3. **实时监控**：
   - 在另一个终端运行 `watch -n 1 'ls -l <episode_dir>/step_* | wc -l'`
   - 应该能看到数据点数量实时增加

