from omni.isaac.kit import SimulationApp
import os

linux_user = os.getlogin()
CARTER_USD_PATH = f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/isaac/carter.usd"
config = {
    "headless": False
}
simulation_app = SimulationApp(config)

# utils
import numpy as np
from enum import Enum
import carb
import time
import threading
import argparse

# isaac
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.prims import XFormPrim, GeometryPrim
import omni.graph.core as og
from omni.isaac.core.utils.stage import add_reference_to_stage

# ros
import rospy
from actionlib import SimpleActionServer
from std_srvs.srv import Empty, EmptyResponse
import rosgraph
from isaac_sim.msg import ResetPosesAction
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion


class SimulationState(Enum):
    RESET = 0
    NORMAL = 1
    CLOSE = 2


class IsaacSimConnection:
    def __init__(self, training_scene):
        enable_extension("omni.isaac.ros_bridge")
        while not rosgraph.is_master_online():
            carb.log_error("Please run roscore before executing this script")
            time.sleep(2.0)

        # scene
        self.training_scene = training_scene

        # multi threading lock
        self.reset_lock = threading.Lock()
        self.state_lock = threading.Lock()

        # reset pose containers
        self.reset_prefix = []
        self.reset_poses = []
        self.init_pose = (1.5, 1.5, 0.0)

        # robot view
        self.robots = []

        # simulation time
        self.time = None

        # ros
        self.close_server = rospy.Service("/close", Empty, self._close_callback)
        self.reset_server = SimpleActionServer("/reset", ResetPosesAction, self._reset_callback, auto_start=False)
        self.reset_server.start()

        # scene
        self.world = World(stage_units_in_meters=1.0)
        if self.training_scene == "env":
            self.world.scene.add_default_ground_plane()
        self._add_robot()
        self._add_env()
        self._add_action_graph()

        # simulation state
        self.state = SimulationState.NORMAL

    def cycle(self):
        self.world.reset()
        self.world.initialize_physics()
        simulation_app.update()
        self.world.play()
        simulation_app.update()
        while simulation_app.is_running:
            if self.state == SimulationState.NORMAL:
                self.world.step()
            elif self.state == SimulationState.RESET:
                self.world.pause()
                self._reset_process()
                self.world.play()
                self.world.step()
                with self.state_lock:
                    self.state = SimulationState.NORMAL
            elif self.state == SimulationState.CLOSE:
                break
        print("simulation app is out of running")
        self.world.stop()
        simulation_app.close()

    def _reset_process(self):
        with self.reset_lock:
            for i, prefix in enumerate(self.reset_prefix):
                try:
                    x, y, yaw = self.reset_poses[i]
                    position = np.array([x, y, 0.3])
                    orientation = np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])
                    self.robots[prefix].set_world_pose(
                        position=position, orientation=orientation
                    )
                except IndexError:
                    rospy.logerr("reset msg contains error prefix, reset later")
            self.reset_prefix.clear()
            self.reset_poses.clear()

    def _add_robot(self):
        wheel_dof_names = ["left_wheel", "right_wheel"]
        self.robot = self.world.scene.add(
            WheeledRobot(
                prim_path="/World/Carters/Carter_0",
                name="Carter_0",
                wheel_dof_names=wheel_dof_names,
                create_robot=True,
                usd_path=CARTER_USD_PATH,
                position=np.array([self.init_pose[0], self.init_pose[1], 0.3]),
                orientation=np.array([np.cos(self.init_pose[2] / 2), 0.0, 0.0, np.sin(self.init_pose[2] / 2)])
            )
        )
        self.robots.append(self.robot)

    def _add_env(self):
        env_usd_path = f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/isaac/{self.training_scene}.usd"
        add_reference_to_stage(usd_path=env_usd_path, prim_path="/World/Envs/Env_0")
        self.world.scene.add(
            GeometryPrim(
                prim_path="/World/Envs/Env_0",
                name="Env",
                collision=True,
                position=np.array([0.0, 0.0, 0.0]),
                orientation=np.array([1.0, 0.0, 0.0, 0.0])
            )
        )

    @staticmethod
    def _add_action_graph():
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/World/clock_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("SimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("ClockPub", "omni.isaac.ros_bridge.ROS1PublishClock"),
                ],
                keys.SET_VALUES: [
                    ("ClockPub.inputs:topicName", "clock"),
                ],
                keys.CONNECT: [
                    ("SimTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
                    ("OnTick.outputs:tick", "ClockPub.inputs:execIn"),
                ]
            }
        )

    @staticmethod
    def _get_yaw(quaternion: Quaternion):
        _, _, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw

    def _reset_callback(self, msg):
        with self.reset_lock and self.reset_lock:
            self.state = SimulationState.RESET
            for i, prefix in enumerate(msg.prefix):
                pose = msg.poses[i]
                self.reset_prefix.append(prefix)
                yaw = self._get_yaw(pose.pose.orientation)
                self.reset_poses.append((pose.pose.position.x, pose.pose.position.y, yaw))
            self.reset_server.set_succeeded()

    def _close_callback(self, msg):
        with self.state_lock:
            self.state = SimulationState.CLOSE
        return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("IsaacSimConnection")
    parser = argparse.ArgumentParser(description="launch the isaac sim environment for test or trajectory recording")
    parser.add_argument("-s", "--scene", type=str, help="Name of Scene", default="office")
    args = parser.parse_args()
    connection = IsaacSimConnection(args.scene)
    connection.cycle()
