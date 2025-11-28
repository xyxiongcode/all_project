# move_base/goal 与 move_base_simple/goal 的区别

## 问题说明

在ROS导航系统中，存在两个与目标点相关的话题：
- `/move_base_simple/goal`
- `/move_base/goal`

## 详细区别

### 1. `/move_base_simple/goal` (Topic)

**类型**：ROS Topic  
**消息类型**：`geometry_msgs/PoseStamped`  
**特点**：
- 简单的发布-订阅模式
- 发布目标后，move_base会**立即接受**并开始规划
- 无法获取目标执行状态反馈
- 无法取消已发布的目标
- 适用于快速、简单的目标设置
- **Rviz的"2D Nav Goal"工具使用此话题**

**使用场景**：
- 手动设置目标点（如Rviz中点击）
- 简单的自动化脚本
- 不需要状态反馈的场景

**代码示例**：
```python
from geometry_msgs.msg import PoseStamped
goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
goal_pub.publish(goal_msg)  # 立即接受，无法取消
```

### 2. `/move_base/goal` (Action Goal Topic)

**类型**：ROS Action Goal Topic  
**消息类型**：`move_base_msgs/MoveBaseActionGoal`  
**特点**：
- 基于Actionlib的Action接口
- 可以获取目标执行状态（PENDING, ACTIVE, SUCCEEDED, ABORTED, REJECTED等）
- **可以取消正在执行的目标**
- 提供更丰富的反馈信息
- 适用于需要精确控制的场景

**使用场景**：
- 需要监控目标执行状态
- 需要取消正在执行的目标
- 需要处理目标失败的情况
- 复杂的导航控制逻辑

**代码示例**：
```python
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

# 创建Action客户端
client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
client.wait_for_server()

# 发送目标
goal = MoveBaseGoal()
goal.target_pose = goal_pose
client.send_goal(goal)

# 可以取消目标
client.cancel_all_goals()

# 可以获取状态
state = client.get_state()
```

## 内部关系

实际上，`/move_base_simple/goal` 是 move_base 节点提供的一个**便捷接口**。当收到 `/move_base_simple/goal` 的消息时，move_base 内部会：

1. 将 `PoseStamped` 消息包装成 `MoveBaseActionGoal`
2. 发布到 `/move_base/goal` 话题
3. 立即接受并开始执行

从 move_base 的源代码可以看到：

```cpp
void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    // 包装成ActionGoal
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.goal.target_pose = *goal;
    // 发布到 /move_base/goal
    action_goal_pub_.publish(action_goal);
}
```

## 修复后的代码改进

### 问题1：机器人翻倒未复位

**原因**：
- 侧翻检测只在定时器中检查，有冷却时间限制
- 如果机器人翻倒后IMU数据暂时稳定，可能不会持续触发检测

**修复**：
- 在 `imu_callback` 中添加了**侧翻持续时间检测**
- 如果侧翻持续时间超过阈值（默认2秒），**立即触发复位**，绕过冷却时间
- 添加了 `rollover_duration_threshold` 参数（默认2.0秒）

```python
# 检查侧翻持续时间
if self.rollover_start_time is not None:
    duration = (now - self.rollover_start_time).to_sec()
    if duration > self.rollover_duration_threshold:
        # 立即触发复位，绕过冷却时间
        rospy.logerr(f"侧翻持续时间 {duration:.1f}s 超过阈值，立即执行复位！")
        self.is_stuck = True
        self.perform_reset()
```

### 问题2：到达目标后未等待5秒

**原因**：
- 使用 `/move_base_simple/goal` 发布目标时，move_base会立即接受新目标
- 即使代码中有 `rospy.sleep(5)`，如果在这5秒内发布了新目标，move_base会立即开始执行

**修复**：
- 添加了 `_cancel_current_goal()` 方法
- 在到达目标后，**先取消当前move_base目标**，再等待5秒
- 使用 move_base 的 Action 接口来取消目标

```python
def _cancel_current_goal(self):
    """取消当前move_base目标"""
    if self.move_base_client is not None:
        try:
            self.move_base_client.cancel_all_goals()
            rospy.loginfo("[目标生成器] 已取消当前move_base目标")
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[目标生成器] 取消move_base目标失败: {e}")

def check_goal_reached_or_timeout(self):
    if dist < self.reach_tolerance:
        # 先取消当前目标
        self._cancel_current_goal()
        self.current_goal = None
        # 等待5秒
        rospy.sleep(self.goal_wait_time)
        self.sample_and_publish_goal()
```

## 新增参数

1. **`rollover_duration_threshold`** (默认2.0秒)
   - 侧翻持续时间阈值
   - 超过此时间立即触发复位，绕过冷却时间

2. **`goal_wait_time`** (默认5.0秒)
   - 到达目标后等待时间
   - 可配置，方便调整

## 使用建议

1. **如果只需要简单发布目标**：使用 `/move_base_simple/goal`
2. **如果需要取消目标或监控状态**：使用 `/move_base/goal` Action接口
3. **修复后的代码**：同时支持两种方式，优先使用Action接口来取消目标

## 验证方法

1. **验证侧翻检测**：
   - 观察日志，当机器人翻倒超过2秒时，应该看到"侧翻持续时间超过阈值，立即执行复位"的日志
   - 机器人应该立即执行复位操作

2. **验证等待时间**：
   - 在Rviz中观察机器人到达目标点后
   - 应该看到"已取消当前move_base目标"的日志
   - 机器人应该等待5秒后才开始移动向新目标

