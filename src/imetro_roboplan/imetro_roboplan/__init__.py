# Copyright (c) 2026, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# This software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

"""
Helpers for the RoboPlan planning server: scene loading and ROS spinning utilities.
"""

import os
import re
import threading
import time

import rclpy
import xacro
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from roboplan.core import Scene, loadJointLimitsConfig, loadUrdfSceneDescriptionFromXml
from sensor_msgs.msg import JointState
from std_msgs.msg import String

BEST_EFFORT_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def process_xacro(path: str) -> str:
    """Returns the XML of a xacro file, or of a plain URDF or SRDF file."""
    return xacro.process_file(path).toxml()


def package_paths_from_urdf(urdf_xml: str) -> list[str]:
    """
    Returns the folders containing the packages that the URDF refers to with package:// URIs,
    which RoboPlan needs to find the meshes.
    """
    packages = sorted(set(re.findall(r"package://([^/\"']+)/", urdf_xml)))
    return sorted({os.path.dirname(get_package_share_directory(package)) for package in packages})


def create_scene(urdf_xml: str, srdf_xml: str, joint_limits_file: str = "", name: str = "roboplan_scene") -> Scene:
    """
    Builds a RoboPlan scene from URDF and SRDF XML strings, optionally overriding the
    joint limits from a RoboPlan joint limits config file.
    """
    scene = Scene(
        name=name,
        description=loadUrdfSceneDescriptionFromXml(urdf_xml, package_paths_from_urdf(urdf_xml)),
    )
    if joint_limits_file:
        scene.importJointLimitsFromConfig(loadJointLimitsConfig(joint_limits_file))
    scene.importSrdf(srdf_xml)
    return scene


def get_robot_description(topic: str, timeout_sec: float) -> str | None:
    """
    Waits for a URDF on the (latched) robot description topic.

    Returns the URDF XML string, or None if nothing was received in time.
    """
    listener_node = Node("robot_description_listener")
    executor = SingleThreadedExecutor()
    executor.add_node(listener_node)

    received = {}
    latched_qos = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
    listener_node.create_subscription(
        String,
        topic,
        lambda msg: received.setdefault("urdf", msg.data),
        latched_qos,
    )

    end_time = time.monotonic() + timeout_sec
    while ("urdf" not in received) and (time.monotonic() < end_time):
        executor.spin_once(timeout_sec=0.1)

    executor.shutdown()
    listener_node.destroy_node()
    return received.get("urdf")


def run_node(node: Node) -> None:
    """Spins a node using a MultiThreadedExecutor and handles exceptions and shutdowns."""
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


def spin_executor(executor, logger=None) -> None:
    """Spin an executor until shutdown, swallowing expected exceptions."""
    try:
        executor.spin()
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        if logger:
            logger.warning(f"Executor caught exception, shutting down: {e}")


class JointStateSubscriber:
    """
    Subscribes and manages a spinner to the specified joint states topic.

    JointStates are often published at the control loop rate, and a subscriber's
    callback functions can eat time in an executor's spinner. This class handles
    standing up the required infrastructure to subscribe to a JointState topic,
    spinning the subscriber, and providing access to the latest message on the
    topic.
    """

    def __init__(self, node_name: str = "joint_state_listener", topic: str = "/joint_states"):
        self.last_joint_state = None

        def joint_state_cb(msg):
            self.last_joint_state = msg

        self._js_node = Node(node_name)
        self._js_sub = self._js_node.create_subscription(
            JointState,
            topic,
            joint_state_cb,
            BEST_EFFORT_QOS,
        )

        self._js_executor = SingleThreadedExecutor()
        self._js_executor.add_node(self._js_node)
        self._js_thread = threading.Thread(target=spin_executor, daemon=True, args=(self._js_executor,))
        self._js_thread.start()

    def shutdown(self):
        self._js_node.destroy_node()
        self._js_executor.shutdown()
        self._js_thread.join()
        self._js_node = None
