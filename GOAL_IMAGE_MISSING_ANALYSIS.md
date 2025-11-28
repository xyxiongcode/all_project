# Goal Image 缺失问题分析

## 问题描述

有些episode文件夹有 `goal_image.png`，有些没有，只有 `step` 文件夹和 `metadata.json`。

## 原因分析

### 1. 正常捕获条件（`_check_and_capture_goal_image`，1540-1599行）

**触发条件**（必须同时满足）：
- ✅ 距离目标点 <= `goal_image_capture_distance`（默认3.0米）
- ✅ 视线无遮挡（通过地图ray tracing检查）
- ✅ 是点目标（`current_goal_type == 'point'`）
- ✅ 有episode目标（`episode_goal` 不为空）

**可能失败的原因**：
1. **距离过远**：机器人从未进入3.0米范围内
2. **视线被遮挡**：即使距离足够，但中间有障碍物
3. **地图数据不可用**：如果 `map_data` 为 `None`，视线检查总是返回 `False`
4. **episode太短**：可能没有足够的数据点

### 2. Fallback机制（`_end_episode`，696-738行）

**触发条件**（必须同时满足）：
- ✅ `not goal_image_captured_to_save` - 还没捕获
- ✅ `success` - **episode必须成功**（这是问题！）
- ✅ `goal_type_to_save == 'point'` - 是点目标
- ✅ `episode_goal_to_save` - 有目标
- ✅ `final_distance < 2.0` - **最终距离必须<2.0米**（这是问题！）

**问题**：
1. **只对成功的episode生效**：如果episode失败（如超时、碰撞、导航失败），fallback不会触发
2. **距离限制太严格**：如果最终距离>=2.0米，即使很接近也不会捕获
3. **依赖 `rgb_original`**：如果数据点中没有保存原始RGB，fallback可能失败

## 修复方案

### 修复1：改进Fallback机制

**问题**：只对成功的episode生效，且距离限制太严格

**修复**：
- 即使episode失败，如果距离足够近（<3.0米），也尝试捕获
- 放宽距离限制（从2.0米增加到3.0米或更大）
- 添加更详细的日志

### 修复2：在地图不可用时放宽条件

**问题**：如果地图数据不可用，视线检查总是失败

**修复**：
- 如果地图不可用，跳过视线检查，只检查距离
- 或者使用更宽松的视线检查（允许少量障碍物）

### 修复3：在episode结束时总是尝试捕获

**问题**：如果正常捕获和fallback都失败，就没有goal_image

**修复**：
- 在episode结束时，遍历所有数据点，找到距离目标最近的点
- 如果最近距离<5.0米，使用该点的图像作为goal_image
- 添加 `capture_reason` 标记，说明捕获方式

### 修复4：添加诊断日志

**问题**：难以诊断为什么没有捕获goal_image

**修复**：
- 在episode结束时，输出详细的goal_image捕获诊断信息
- 包括：距离、视线检查结果、地图可用性、episode成功状态等

