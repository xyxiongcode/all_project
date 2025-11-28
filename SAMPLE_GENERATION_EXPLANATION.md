# Sample文件夹生成机制和终端输出说明

## 一、Sample文件夹生成机制

### 1. 生成时机

**Sample文件夹在Episode结束时生成**，具体流程如下：

```
Episode结束 → 保存Episode数据 → 生成训练样本 → 保存Sample文件夹
```

**代码位置**：`_end_episode()` → `_generate_training_samples_from_data()`

### 2. 生成流程

#### 步骤1：Episode数据收集完成
- 机器人在导航过程中，每个时间步的数据被收集到 `self.episode_data` 列表中
- 每个数据点包含：RGB图像、深度图像、位姿、动作、目标表示等

#### 步骤2：Episode结束触发样本生成
```python
# 在 _end_episode() 中
training_samples = self._generate_training_samples_from_data(
    episode_data_to_save, 
    episode_id_to_save, 
    split_type_to_save
)
```

#### 步骤3：遍历Episode数据点，创建训练样本
```python
# 在 _generate_training_samples_from_data() 中
for i in range(len(episode_data)):
    # 检查是否有足够的历史数据
    if not episode_data[i]['has_sufficient_history']:
        continue
    
    # 检查是否有足够的未来动作
    if i + self.predict_size >= len(episode_data):
        continue  # 最后predict_size个数据点无法生成样本
    
    # 创建训练样本
    sample = self._create_training_sample_from_data(i, episode_data, episode_id, split_type)
    if sample:
        training_samples.append(sample)
```

**关键参数**：
- `memory_size = 8`：历史帧数（需要8帧历史数据）
- `predict_size = 24`：预测步数（需要24步未来动作）

**生成条件**：
1. 当前数据点必须有足够的历史数据（至少8帧）
2. 当前数据点之后必须有足够的未来动作（至少24步）
3. 因此，每个Episode的最后24个数据点无法生成样本

---

### 3. 单个Sample的结构

每个Sample包含以下内容：

#### 3.1 历史序列（History Sequence）
- **RGB序列**：8帧历史RGB图像（`rgb_00.png` ~ `rgb_07.png`）
- **深度序列**：8帧历史深度图像（保存在 `depth_sequence.npy` 中）

**获取方式**：
```python
# 获取历史序列
start_idx = max(0, current_index - self.memory_size + 1)
history_indices = list(range(start_idx, current_index + 1))

# 如果历史不足，用第一帧填充
while len(history_indices) < self.memory_size:
    history_indices.insert(0, history_indices[0])
```

#### 3.2 未来动作序列（Future Action Sequence）
- **动作序列**：24步未来动作（保存在 `action_sequence.npy` 中）
- 每步动作包含：`[linear_x, linear_y, angular_z]`

**获取方式**：
```python
# 获取未来动作序列
future_actions = []
for j in range(current_index, current_index + self.predict_size):
    if j < len(episode_data):
        action = episode_data[j]['action']
        future_actions.append([
            float(action['linear_x']),
            float(action['linear_y']),
            float(action['angular_z'])
        ])
    else:
        future_actions.append([0.0, 0.0, 0.0])  # 填充
```

#### 3.3 目标表示（Goal Representation）
- **点目标**：`point_goal = [x, y, theta]`（保存在 `metadata.json` 中）
- **图像目标**：`image_goal.png`（如果使用图像目标）
- **像素目标**：`pixel_goal.npy`（如果使用像素目标）

#### 3.4 机器人状态（Robot State）
- **位姿**：`robot_pose = {'x', 'y', 'theta'}`（保存在 `metadata.json` 中）
- **速度**：`velocity = {'linear_x', 'linear_y', 'angular_z'}`（保存在 `metadata.json` 中）

#### 3.5 语义信息（Navigation Semantics）
- **导航语义**：`nav_semantics`（保存在 `metadata.json` 中）
  - `has_goal`：是否有目标
  - `distance_to_goal`：到目标的距离
  - `heading_error`：航向误差
  - `min_lidar_distance`：最小LiDAR距离
  - `safety_score`：安全评分

---

### 4. Sample文件夹结构

每个Sample保存在独立的文件夹中：

```
{save_dir}/
  train/samples/
    sample_{episode_id}_{sample_index:04d}/
      rgb_00.png          # 历史RGB图像（第1帧）
      rgb_01.png          # 历史RGB图像（第2帧）
      ...
      rgb_07.png          # 历史RGB图像（第8帧）
      depth_sequence.npy   # 历史深度序列（8帧）
      action_sequence.npy # 未来动作序列（24步）
      image_goal.png      # 图像目标（如果有）
      pixel_goal.npy      # 像素目标（如果有）
      metadata.json       # 样本元数据
```

**命名规则**：
- `sample_{episode_id}_{sample_index:04d}`
- 例如：`sample_0_0000`, `sample_0_0001`, `sample_1_0000`

---

### 5. 保存过程

```python
def _save_training_sample(self, sample, sample_index):
    # 1. 创建Sample文件夹
    sample_dir = os.path.join(split_dirs['samples'], 
                              f"sample_{sample['episode_id']}_{sample_index:04d}")
    os.makedirs(sample_dir, exist_ok=True)
    
    # 2. 保存RGB序列（8帧）
    for i, rgb_img in enumerate(sample['rgb_sequence']):
        cv2.imwrite(os.path.join(sample_dir, f"rgb_{i:02d}.png"), rgb_img)
    
    # 3. 保存深度序列（8帧）
    depth_sequence = np.stack(sample['depth_sequence'])
    np.save(os.path.join(sample_dir, "depth_sequence.npy"), depth_sequence)
    
    # 4. 保存目标数据
    if sample['image_goal'] is not None:
        cv2.imwrite(os.path.join(sample_dir, "image_goal.png"), sample['image_goal'])
    if sample['pixel_goal'] is not None:
        np.save(os.path.join(sample_dir, "pixel_goal.npy"), sample['pixel_goal'])
    
    # 5. 保存动作序列（24步）
    np.save(os.path.join(sample_dir, "action_sequence.npy"), 
            np.array(sample['action_sequence']))
    
    # 6. 保存元数据
    with open(os.path.join(sample_dir, "metadata.json"), 'w') as f:
        json.dump(sample_metadata, f, indent=2)
    
    # 7. 更新CSV文件
    self._update_samples_csv(sample, sample_dir)
```

---

### 6. 训练/测试分割

**分割方式**：按Episode分割（`split_by_episode = True`）

```python
def _determine_split_type(self):
    if self.split_by_episode:
        rand_val = pyrand.random()
        if rand_val < self.train_test_split:  # 默认0.8
            return 'train'
        else:
            return 'test'
```

**说明**：
- 80%的Episode → 训练集（`train/samples/`）
- 20%的Episode → 测试集（`test/samples/`）
- 同一个Episode的所有Sample都属于同一个数据集（train或test）

---

## 二、终端输出的数据点

### 1. Episode结束时的输出

```
[数据收集器] === Episode {episode_counter} 结束 ===
[数据收集器] 原因: {termination_reason}, 成功: {success}
[数据收集器] 持续时间: {duration:.2f}秒, 步数: {len(episode_data)}
[数据收集器] 成功率: {success_rate:.2%} ({successful_episodes}/{episode_counter})
[数据收集器] 生成 {len(training_samples)} 个训练样本  ← 这里！
[数据收集器] 数据保存到: {episode_path}
```

**说明**：
- `生成 {len(training_samples)} 个训练样本`：表示从这个Episode生成了多少个Sample
- 这个数字 = Episode数据点总数 - `predict_size`（24）

**示例**：
- Episode有100个数据点
- 最后24个数据点无法生成样本（因为没有足够的未来动作）
- 因此生成 100 - 24 = **76个样本**

---

### 2. 样本生成过程中的输出

```
[数据收集器] 开始从episode生成训练样本...
[数据收集器] 开始保存 {len(training_samples)} 个训练样本...
[数据收集器] 已保存 {i + 1}/{len(training_samples)} 个训练样本  ← 每50个输出一次
[数据收集器] 训练样本生成完成，共 {len(training_samples)} 个样本
```

**说明**：
- 每保存50个样本，输出一次进度
- 最终输出总样本数

---

### 3. 调试状态报告中的输出

```
[数据收集器] === 调试状态报告 ===
...
总样本: {total_samples}
训练样本: {train_samples} ({train_samples / total_samples * 100:.1f}%)
测试样本: {test_samples} ({test_samples / total_samples * 100:.1f}%)
```

**说明**：
- 显示累计的样本统计信息
- 包括训练集和测试集的样本数量和比例

---

## 三、数据点与样本的关系

### 1. 数据点（Data Point）

**定义**：机器人在每个时间步收集的原始数据

**包含内容**：
- RGB图像
- 深度图像
- 位姿（x, y, theta）
- 动作（linear_x, linear_y, angular_z）
- 目标表示
- 导航语义

**保存位置**：`{save_dir}/train/episodes/episode_{id}/step_{index}/`

---

### 2. 训练样本（Training Sample）

**定义**：从数据点生成的、用于训练模型的样本

**包含内容**：
- **历史序列**：8帧历史RGB和深度图像
- **未来动作**：24步未来动作序列
- **目标表示**：点/图像/像素目标
- **机器人状态**：当前位姿和速度
- **导航语义**：安全评分、距离等

**保存位置**：`{save_dir}/train/samples/sample_{episode_id}_{sample_index}/`

---

### 3. 转换关系

```
数据点 (Data Point) → 训练样本 (Training Sample)
```

**转换规则**：
1. 每个数据点都可以作为样本的"当前时刻"
2. 需要8帧历史数据（从当前时刻往前推）
3. 需要24步未来动作（从当前时刻往后推）
4. 因此，最后24个数据点无法生成样本

**示例**：
- Episode有100个数据点（索引0-99）
- 可以生成样本的数据点：0-75（共76个）
- 无法生成样本的数据点：76-99（共24个，因为缺少未来动作）

---

## 四、关键参数说明

### 1. `memory_size = 8`
- **含义**：历史帧数
- **作用**：每个样本需要8帧历史RGB和深度图像
- **影响**：前7个数据点需要填充历史（用第一帧填充）

### 2. `predict_size = 24`
- **含义**：预测步数
- **作用**：每个样本需要24步未来动作
- **影响**：最后24个数据点无法生成样本

### 3. `train_test_split = 0.8`
- **含义**：训练集比例
- **作用**：80%的Episode用于训练，20%用于测试
- **影响**：按Episode分割，不是按样本分割

---

## 五、总结

### Sample生成机制
1. **时机**：Episode结束时
2. **方式**：遍历Episode数据点，为每个有效数据点创建样本
3. **条件**：需要8帧历史和24步未来动作
4. **结果**：每个Episode生成 `数据点数 - 24` 个样本

### 终端输出
1. **Episode结束时**：显示生成的样本数量
2. **保存过程中**：每50个样本输出一次进度
3. **状态报告**：显示累计的样本统计信息

### 数据点 vs 样本
- **数据点**：原始收集的数据，保存在 `episodes/` 目录
- **样本**：用于训练的样本，保存在 `samples/` 目录
- **关系**：样本从数据点生成，但需要历史序列和未来动作

