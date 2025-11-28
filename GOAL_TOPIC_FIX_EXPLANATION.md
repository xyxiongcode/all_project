# Goal Topic 问题修复说明

## 问题发现

在 `random_generate_goal.py` 文件中发现了一个**消息类型不匹配**的问题：

- **第48行**：默认话题是 `/move_base/goal`
- **第93行**：发布的消息类型是 `PoseStamped`
- **问题**：`/move_base/goal` 是 Action Goal Topic，需要 `MoveBaseActionGoal` 消息类型，而不是 `PoseStamped`

## 消息类型对应关系

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 简单Topic，接受PoseStamped |
| `/move_base/goal` | `move_base_msgs/MoveBaseActionGoal` | Action Goal Topic，需要ActionGoal消息 |

## 当前代码的问题

```python
# 第48行：默认话题
self.goal_topic = rospy.get_param('~goal_topic', '/move_base/goal')

# 第93行：发布PoseStamped消息
self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
```

**问题**：如果使用默认值 `/move_base/goal`，但发布的是 `PoseStamped`，消息类型不匹配！

## 解决方案

### 方案1：改回 `/move_base_simple/goal`（推荐）

**优点**：
- 简单直接
- 消息类型匹配（PoseStamped）
- 数据收集器订阅的就是这个话题
- 代码改动最小

**缺点**：
- 无法直接取消目标（但可以通过Action客户端取消）

**修改**：
```python
self.goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')
```

### 方案2：使用 Action 客户端（更强大但复杂）

**优点**：
- 可以取消目标
- 可以获取状态反馈
- 更精确的控制

**缺点**：
- 代码更复杂
- 需要修改发布逻辑

**修改**：需要改用 Action 客户端发送目标

## 推荐方案

**推荐使用方案1**，原因：
1. 数据收集器订阅的是 `/move_base_simple/goal`
2. 代码已经通过 Action 客户端实现了取消目标的功能
3. 简单直接，消息类型匹配

## 修复代码

将默认话题改回 `/move_base_simple/goal`：

```python
self.goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')
```

这样：
- 发布 `PoseStamped` 到 `/move_base_simple/goal` ✓（消息类型匹配）
- 数据收集器可以收到目标 ✓
- 仍然可以通过 Action 客户端取消目标 ✓

