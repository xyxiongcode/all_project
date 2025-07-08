#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from global_nn_planner.srv import PlanPath, PlanPathResponse
from db_cnn_infer import DBCNNPlanner

planner = DBCNNPlanner("/home/gr-agv-x9xy/isaac_sim_ws/src/global_nn_planner/db_cnn_planner_logs")  #  SavedModel

def handle_request(req):
    input_map = np.array(req.map.data).reshape((64, 64))
    start = (int(req.start.pose.position.x), int(req.start.pose.position.y))
    goal = (int(req.goal.pose.position.x), int(req.goal.pose.position.y))

    path_points = planner.predict_path(input_map, start, goal)

    ros_path = Path()
    ros_path.header.frame_id = "map"
    for x, y in path_points:
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        ros_path.poses.append(pose)

    return PlanPathResponse(path=ros_path)

if __name__ == "__main__":
    rospy.init_node("dbc_path_server")
    rospy.Service("dbc_make_plan", PlanPath, handle_request)
    rospy.loginfo("DB-CNN service ready.")
    rospy.spin()
