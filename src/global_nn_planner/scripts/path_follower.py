#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import tf
import numpy as np
import json
import os

class PathFollower:
    def __init__(self):
        rospy.init_node("path_follower")

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/predicted_path", PoseStamped, self.path_callback)

        self.current_pose = None
        self.predicted_path = []
        self.goal_reached = False
        self.rate = rospy.Rate(10)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def path_callback(self, msg):
        self.predicted_path.append(msg)
        rospy.loginfo(f"[Follower] Received predicted path point. Total: {len(self.predicted_path)}")

    def move_to_goal(self):
        while not rospy.is_shutdown() and not self.goal_reached:
            if self.current_pose is None or not self.predicted_path:
                self.rate.sleep()
                continue

            goal = self.predicted_path[0].pose.position
            pos = self.current_pose.position

            dx = goal.x - pos.x
            dy = goal.y - pos.y
            distance = np.hypot(dx, dy)
            rospy.loginfo(f"[Follower] Robot at: {pos.x:.2f}, {pos.y:.2f}, goal at {goal.x:.2f}, {goal.y:.2f}, distance={distance:.2f}")

            if distance < 0.2:
                self.predicted_path.pop(0)
                if not self.predicted_path:
                    rospy.loginfo("Goal reached.")
                    self.goal_reached = True
                    self.cmd_pub.publish(Twist())  # stop
                    break
                continue

            angle_to_goal = np.arctan2(dy, dx)
            orientation = self.current_pose.orientation
            yaw = tf.transformations.euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w])[2]

            angle_diff = angle_to_goal - yaw
            angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

            cmd = Twist()
            cmd.linear.x = 0.2
            rospy.loginfo(f"[Follower] Angle: {angle_diff:}, publishing cmd_vel")
            cmd.angular.z = angle_diff
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        follower = PathFollower()
        follower.move_to_goal()
    except rospy.ROSInterruptException:
        pass
