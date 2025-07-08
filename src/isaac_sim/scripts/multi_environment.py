# This is an Isaac Sim Connection Scripts for multi Carter-v1 robot
# The TF Tree is organized as /World ---> /Map ---> /base_link ---> /chassis_link

import os

linux_user = os.getlogin()
CARTER_USD_PATH = f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/isaac/carter.usd"

# pkg
from environment import IsaacSimConnection

# utils
import sys
import numpy as np
import carb
import time

# isaac
import omni.graph.core as og
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.cloner import GridCloner
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.articulations import ArticulationView, Articulation
from omni.isaac.core.prims import XFormPrim, GeometryPrim
from omni.isaac.core.utils.extensions import enable_extension

# ros
import rospy
import rosgraph
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

interval = 5.0


class MultiIsaacSimConnection(IsaacSimConnection):
    def __init__(self, training_scene="env"):
        # wait for ros master
        enable_extension("omni.isaac.ros_bridge")
        while not rosgraph.is_master_online():
            carb.log_error("Please run roscore before executing this script")
            time.sleep(2.0)

        # get some parameters from ros master
        params = ["robots_rows", "robots_columns", "map_height", "map_width"]
        self.config = {}
        for param in params:
            while not rospy.has_param(param):
                rospy.logerr(f"do not has param:{param} yet")
                rospy.sleep(2.0)
            self.config[param] = rospy.get_param(param)
        self.height = self.config["map_height"] + interval
        self.width = self.config["map_width"] + interval
        self.robot_number = self.config["robots_rows"] * self.config["robots_columns"]

        # call init function
        super().__init__(training_scene)

        # setup environment
        # self._clone_robot(self.robot_number)
        # self._init_robot_pose()

        # setup ros
        # A strange problem is that if using tf2 static br, only the trans from world to carter_3 could be pub
        self.tf_pub = rospy.Publisher("/tf_static", TFMessage, queue_size=100, latch=True)
        self._pub_static_tf()
        self._set_namespace(self.robot_number)

    def _pub_static_tf(self):
        msg = TFMessage()
        for i in range(self.config["robots_rows"]):
            for j in range(self.config["robots_columns"]):
                prefix = i * self.config["robots_columns"] + j
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = "/World"
                transform.child_frame_id = f"/Carter_{prefix}/map"
                transform.transform.translation.x = j * self.width
                transform.transform.translation.y = i * self.height
                transform.transform.translation.z = 0
                transform.transform.rotation.w = 1.0
                transform.transform.rotation.x = 0.0
                transform.transform.rotation.y = 0.0
                transform.transform.rotation.z = 0.0
                msg.transforms.append(transform)
        self.tf_pub.publish(msg)

    def _init_robot_pose(self):
        for i in range(self.config["robots_rows"]):
            for j in range(self.config["robots_columns"]):
                self.robots[i * self.config["robots_columns"] + j].set_world_pose(
                    position=np.array([j * self.width + 1.5, i * self.height + 1.5, 0.0]),
                    orientation=np.array([1.0, 0.0, 0.0, 0.0])
                )
                print(np.array([j * self.width + 1.5, i * self.height + 1.5, 0.0]))

    def _add_robot(self):
        wheel_dof_names = ["left_wheel", "right_wheel"]
        for i in range(self.config["robots_rows"]):
            for j in range(self.config["robots_columns"]):
                prefix = i * self.config["robots_columns"] + j
                robot = self.world.scene.add(
                    WheeledRobot(
                        prim_path=f"/World/Carters/Carter_{prefix}",
                        name=f"Carter_{prefix}",
                        wheel_dof_names=wheel_dof_names,
                        create_robot=True,
                        usd_path=CARTER_USD_PATH,
                        position=np.array([j * self.width + 1.5, i * self.height + 1.5, 0.0]),
                        orientation=np.array([1.0, 0.0, 0.0, 0.0])
                    )
                )
                self.robots.append(robot)

    def _add_env(self):
        env_usd_path = f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/isaac/{self.training_scene}.usd"
        for i in range(self.config["robots_rows"]):
            for j in range(self.config["robots_columns"]):
                add_reference_to_stage(usd_path=env_usd_path, prim_path=f"/World/Envs/Env_{i}_{j}")
                self.world.scene.add(
                    GeometryPrim(
                        prim_path=f"/World/Envs/Env_{i}_{j}",
                        name=f"Env_{i}_{j}",
                        collision=True,
                        position=np.array([j * self.width, i * self.height, 0.0]),
                    )
                )

    def _set_namespace(self, number):
        self.world.reset()
        for prefix in range(number):
            graph = og.Controller.graph(f"/World/Carters/Carter_{prefix}/Carter_Control_Graph")
            og.GraphController.set_variable_default_value(variable_id=(graph, "namespace"), value=f"Carter_{prefix}")
            graph = og.Controller.graph(f"/World/Carters/Carter_{prefix}/Carter_Sensor_Graph")
            og.GraphController.set_variable_default_value(variable_id=(graph, "namespace"), value=f"Carter_{prefix}")
            graph = og.Controller.graph(f"/World/Carters/Carter_{prefix}/Carter_Camera_Graph")
            og.GraphController.set_variable_default_value(variable_id=(graph, "namespace"), value=f"Carter_{prefix}")
        self.world.reset()

    def _clone_robot(self, number):
        cloner = GridCloner(spacing=50)
        target_path = cloner.generate_paths("/World/Carters/Carter", number)
        position_offset = np.array([[0, 0, 0]] * number)
        cloner.clone(
            source_prim_path="/World/Carters/Carter_0",
            prim_paths=target_path,
            position_offsets=position_offset,
            replicate_physics=True,
            base_env_path="/World/Carters",
        )
        self.robots.append(self.robot)
        for i in range(1, number):
            self.robots.append(
                Articulation(
                    prim_path=f"/World/Carters/Carter_{i}/chassis_link",
                    name=f"Carter_{i}"
                )
            )


if __name__ == "__main__":
    rospy.init_node("MultiIsaacSimConnection")
    if len(sys.argv) > 1:
        scene = sys.argv[1]
        assert scene in ["env", "warehouse"]
        connection = MultiIsaacSimConnection(scene)
    else:
        connection = MultiIsaacSimConnection()
    connection.cycle()
    rospy.spin()
