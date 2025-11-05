# This is an Isaac Sim Connection Scripts for single Carter-v1 robot
# Multi Robots simulation can inherit Class IsaacConnection
import geometry_msgs.msg
from omni.isaac.kit import SimulationApp
import os

linux_user = os.getlogin()
CARTER_USD_PATH = f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/isaac/carter.usd"
config = {
    "headless": False,
    # "active_gpu": 0,
    # "physics_gpu": 0,
    # "multi_gpu": False
}
simulation_app = SimulationApp(config)

# utils
import sys
import numpy as np
from enum import Enum
import carb
import time
import threading

# isaac
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.prims import XFormPrim, GeometryPrim
import omni.graph.core as og
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import RotatingLidarPhysX
from omni.isaac.range_sensor import _range_sensor

# ros
import rospy
from actionlib import SimpleActionServer
from std_srvs.srv import Empty, EmptyResponse
import rosgraph
from isaac_sim.msg import ResetPosesAction, ResetPosesResult, StepAction, StepResult
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion


class SimulationState(Enum):
    RESET = 0
    PAUSE = 1
    STEP = 2
    NORMAL = 4
    CLOSE = 8


class IsaacSimConnection:
    def __init__(self, training_scene="env"):
        enable_extension("omni.isaac.ros_bridge")
        while not rosgraph.is_master_online():
            carb.log_error("Please run roscore before executing this script")
            time.sleep(2.0)

        # scene
        self.training_scene = training_scene

        # multi threading lock
        self.reset_lock = threading.Lock()
        self.state_lock = threading.Lock()

        # laser
        self.lidar_pool = []
        self.lidar_capacity = 100

        # reset pose containers
        self.reset_prefix = []
        self.reset_poses = []
        self.init_pose = (1.5, 1.5, 0.0)
        self.step_vel = np.array((0.0, 0.0))
        self.step_flag = False
        self.reset_flag = False

        # robot view
        self.robots = []

        # simulation time
        self.time = None

        # ros
        self.close_server = rospy.Service("/close", Empty, self._close_callback)
        self.reset_server = SimpleActionServer("/reset", ResetPosesAction, self._reset_callback, auto_start=False)
        self.step_server = SimpleActionServer("/step", StepAction, self._step_callback, auto_start=False)
        self.reset_server.start()
        self.step_server.start()

        # scene
        self.world = World(stage_units_in_meters=1.0, physics_dt=0.05, rendering_dt=0.05)
        if self.training_scene == "env":
            self.world.scene.add_default_ground_plane()
        self._add_robot()
        self._add_env()
        self._add_action_graph()

        # simulation state
        self.state = SimulationState.NORMAL

    def cycle(self):
        self.world.reset()
        self.carter_lidar.enable_visualization()
        self.carter_lidar.add_depth_data_to_frame()
        while simulation_app.is_running():
            # rospy.logfatal(self.state)
            if self.state == SimulationState.NORMAL:
                self.world.step()
                self._store_laser()
            elif self.state == SimulationState.PAUSE:
                self.world.pause()
            elif self.state == SimulationState.STEP:
                self.robot.apply_wheel_actions(self.controller.forward(command=self.step_vel))
                self.world.play()
                self._store_laser()
                with self.state_lock:
                    self.state = SimulationState.PAUSE
                    self.step_flag = True
            elif self.state == SimulationState.RESET:
                self.world.pause()
                self._reset_process()
                self.robot.apply_wheel_actions(self.controller.forward(command=np.array((0., 0.))))
                self.world.play()
                self._store_laser()
                with self.state_lock:
                    self.state = SimulationState.NORMAL
                    self.reset_flag = True
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
            self.lidar_pool.clear()

    def _store_laser(self):
        cur_laser = self.lidarInterface.get_linear_depth_data(self.lidar_path)
        if len(self.lidar_pool) >= self.lidar_capacity:
            self.lidar_pool.pop(0)
        self.lidar_pool.append(cur_laser)

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
        self.lidar_path = "/World/Carters/Carter_0/chassis_link/lidar"
        self.carter_lidar = self.world.scene.add(
            RotatingLidarPhysX(
                prim_path=self.lidar_path,
                name="lidar",
                rotation_frequency=0,
                translation=np.array([-0.06, 0, 0.38]),
                fov=(270, 0.0), resolution=(0.25, 0.0),
                valid_range=(0.4, 10.0)
            )
        )
        self.lidarInterface = _range_sensor.acquire_lidar_sensor_interface()
        self.controller = DifferentialController(name="simple_control", wheel_radius=0.24, wheel_base=0.56)
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

    def _step_callback(self, msg):
        with self.state_lock:
            self.step_vel = np.array((msg.forward, msg.angular))
            self.state = SimulationState.STEP
            self.step_flag = False
        # rospy.loginfo("finish lock")
        while not self.step_flag:
            time.sleep(0.05)
        # rospy.loginfo("finish stepping")
        result = StepResult()
        result.frames = 6
        result.ranges = np.array(self._get_lidar()).flatten().tolist()
        self.step_server.set_succeeded(result=result)

    def _reset_callback(self, msg):
        with self.reset_lock and self.state_lock:
            self.state = SimulationState.RESET
            self.reset_flag = False
            for i, prefix in enumerate(msg.prefix):
                pose = msg.poses[i]
                self.reset_prefix.append(prefix)
                yaw = self._get_yaw(pose.pose.orientation)
                self.reset_poses.append((pose.pose.position.x, pose.pose.position.y, yaw))
        while not self.reset_flag:
            time.sleep(0.05)
        result = ResetPosesResult()
        result.frames = 6
        result.ranges = np.array(self._get_lidar()).flatten().tolist()
        self.reset_server.set_succeeded(result=result)

    def _get_lidar(self, frames=6):
        lasers = [self.lidar_pool[-1] for _ in range(frames)]
        for i in range(frames - 1, 0, -1):
            prefix = len(self.lidar_pool) - i * 10
            if prefix < 0:
                continue
            else:
                lasers[frames - i - 1] = self.lidar_pool[prefix]
        return lasers

    def _close_callback(self, msg):
        with self.state_lock:
            self.state = SimulationState.CLOSE
        return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("IsaacSimConnection")
    if len(sys.argv) > 1:
        scene = sys.argv[1]
        assert scene in ["env", "warehouse", "hospital", "office"]
        connection = IsaacSimConnection(scene)
    else:
        connection = IsaacSimConnection()
    connection.cycle()
