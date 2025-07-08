import geometry_msgs.msg
from isaac_sim.msg import ResetPosesAction, ResetPosesGoal
import rospy
from actionlib import SimpleActionClient

rospy.init_node("test_reset_action")
client = SimpleActionClient("/reset", ResetPosesAction)
client.wait_for_server()
goal = ResetPosesGoal()
goal.prefix.append(0)
pose = geometry_msgs.msg.PoseStamped()
pose.pose.position.x = 1.5
pose.pose.position.y = 10.0
pose.pose.orientation.w = 1.0
goal.poses.append(pose)
client.send_goal(goal)
client.wait_for_result()
print("success")
