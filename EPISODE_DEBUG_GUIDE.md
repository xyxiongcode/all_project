# Episode目录生成问题排查指南

## 问题现象
机器人生成好几次轨迹后，但是不生成episode目录。

## 排查步骤

### 1. 查看实时状态报告
脚本会每30秒自动输出状态报告，包含：
- 收到目标数
- 忽略目标数
- Episode开始/结束/保存统计
- 数据收集统计

### 2. 手动查看当前状态
```bash
rosservice call /navdp_generate_dataset/print_debug_status
```

### 3. 检查日志中的关键信息

#### 3.1 检查目标接收情况
查找日志中的：
```
[数据收集器] === 收到新的点目标 (#X) ===
```
- 如果看不到这条日志 → **目标没有发送到数据收集器**
  - 检查目标生成器是否正常运行
  - 检查话题名称是否正确：`/move_base_simple/goal`

#### 3.2 检查状态异常
查找日志中的：
```
⚠️ 状态异常! 当前状态=X, 期望状态=0
⚠️ 当前正在采集episode，忽略新目标
```
- 如果频繁出现 → **上一个episode没有正确结束**
  - 检查是否有episode卡在COLLECTING状态
  - 检查move_base状态是否正常

#### 3.3 检查数据质量
查找日志中的：
```
数据质量检查失败，跳过此数据点
已拒绝数据点: X
```
- 如果拒绝率很高 → **传感器数据质量问题**
  - 检查RGB/深度图像话题是否正常
  - 检查LiDAR数据是否正常
  - 检查时间同步是否正常

#### 3.4 检查空Episode
查找日志中的：
```
⚠️ Episode X 数据为空，不保存
```
- 如果频繁出现 → **episode开始后没有收集到数据**
  - 检查数据质量检查是否过于严格
  - 检查episode是否太快结束

### 4. 检查文件系统

#### 4.1 检查是否有空episode标记文件
```bash
find <save_dir> -name "empty_episode.txt" -exec cat {} \;
```

#### 4.2 检查episode目录
```bash
ls -la <save_dir>/train/episodes/
ls -la <save_dir>/test/episodes/
```

### 5. 常见问题及解决方案

#### 问题1: 目标被忽略（状态异常）
**原因**: 上一个episode没有正确结束，状态卡在COLLECTING

**解决方案**:
1. 检查日志中是否有episode结束的日志
2. 检查move_base状态是否正常
3. 如果确认episode已结束但状态未重置，可以重启数据收集器

#### 问题2: 数据质量检查失败
**原因**: 传感器数据不合格

**解决方案**:
1. 检查传感器话题是否正常：
   ```bash
   rostopic echo /rgb_left -n 1
   rostopic echo /depth_left -n 1
   rostopic echo /scan -n 1
   ```
2. 检查时间同步是否正常
3. 可以临时放宽数据质量检查标准

#### 问题3: Episode数据为空
**原因**: episode开始后很快结束，没有收集到数据

**解决方案**:
1. 检查episode终止条件是否过于严格
2. 检查min_episode_steps参数
3. 检查机器人是否真的在移动

### 6. 调试命令汇总

```bash
# 查看当前状态
rosservice call /navdp_generate_dataset/print_debug_status

# 查看目标话题
rostopic echo /move_base_simple/goal -n 1

# 查看move_base状态
rostopic echo /move_base/status -n 1

# 查看数据收集器节点信息
rosnode info /navdp_generate_dataset

# 查看话题列表
rostopic list | grep -E "(rgb|depth|scan|odom|goal)"

# 查看episode目录
find <save_dir> -type d -name "episode_*" | wc -l
```

### 7. 关键日志模式

#### 正常流程日志：
```
[数据收集器] === 收到新的点目标 (#1) ===
[数据收集器] === 开始Episode 0 ===
[数据收集器] ✓ Episode文件夹已创建: ...
[数据收集器] Episode进度: 20步, 目标距离: X.XXXm
[数据收集器] === Episode终止原因: move_base_success ===
[数据收集器] === Episode 1 结束 ===
[数据收集器] 数据保存到: ...
```

#### 异常流程日志：
```
[数据收集器] === 收到新的点目标 (#2) ===
⚠️ 状态异常! 当前状态=1, 期望状态=0
⚠️ 当前正在采集episode，忽略新目标
```

### 8. 统计信息解读

状态报告中的关键指标：
- **收到目标数 vs 忽略目标数**: 如果忽略率>50%，说明状态管理有问题
- **已开始 vs 已保存**: 如果差异大，说明很多episode没有保存
- **数据点拒绝率**: 如果>30%，说明数据质量问题严重

## 快速诊断清单

- [ ] 目标生成器是否正常运行？
- [ ] 数据收集器是否收到目标？（查看日志）
- [ ] 是否有状态异常警告？
- [ ] 数据质量检查是否通过？
- [ ] Episode是否正常结束？
- [ ] 是否有保存错误的日志？
- [ ] 文件系统是否有写入权限？

