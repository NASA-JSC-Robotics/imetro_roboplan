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
Behaviors for motion planning with a RoboPlan planning server and execution
via ROS 2 controllers.

Planning behaviors call the services offered by a RoboPlan planning server
node (see the imetro_roboplan package) and write
the resulting trajectory_msgs/JointTrajectory to the blackboard.

Execution sends that trajectory directly to a joint trajectory controller's
FollowJointTrajectory action.
"""

from typing import Any

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from imetro_behavior.helpers import set_ros_node
from imetro_behavior.ros_behaviors.action_client import RosActionClientBase
from imetro_behavior.ros_behaviors.service_client import RosServiceClientBase
from py_trees.common import Status
from py_trees.ports import PortInformation
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory

from imetro_roboplan_msgs.srv import (
    PlanCartesianPath,
    PlanToJointState,
    PlanToPose,
    SetCollisions,
)

# Error code names for control_msgs/FollowJointTrajectory results.
FOLLOW_JOINT_TRAJECTORY_ERROR_DICT = {
    FollowJointTrajectory.Result.SUCCESSFUL: "SUCCESSFUL",
    FollowJointTrajectory.Result.INVALID_GOAL: "INVALID_GOAL",
    FollowJointTrajectory.Result.INVALID_JOINTS: "INVALID_JOINTS",
    FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP: "OLD_HEADER_TIMESTAMP",
    FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED: "PATH_TOLERANCE_VIOLATED",
    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED: "GOAL_TOLERANCE_VIOLATED",
}


class RoboplanPlanToJointState(RosServiceClientBase):
    """
    Uses a RoboPlan planning server to plan a free-space motion to a target
    joint configuration.
    """

    def __init__(self, name: str, **kwargs: Any):
        super().__init__(name, service_type=PlanToJointState, **kwargs)

    INPUT_PORTS = {
        "group_name": PortInformation(data_type=str, required=False),
        "joint_names": PortInformation(data_type=list[str], required=True),
        "joint_positions": PortInformation(data_type=list[float], required=True),
        "velocity_scaling": PortInformation(data_type=float, required=False),
        "acceleration_scaling": PortInformation(data_type=float, required=False),
    }

    OUTPUT_PORTS = {"trajectory": PortInformation(data_type=JointTrajectory)}

    def create_request(self) -> PlanToJointState.Request:
        """Create the planning service request."""
        joint_names = self.get_input("joint_names")
        joint_positions = self.get_input("joint_positions")
        if len(joint_positions) != len(joint_names):
            raise RuntimeError("Joint names and joint positions must have the same length.")

        return PlanToJointState.Request(
            group_name=self.get_input("group_name", ""),
            joint_names=joint_names,
            joint_positions=joint_positions,
            velocity_scaling=self.get_input("velocity_scaling", 0.0),
            acceleration_scaling=self.get_input("acceleration_scaling", 0.0),
        )

    def process_response(self, response: PlanToJointState.Response) -> Status:
        """Process the planning service response."""
        if response.success:
            self.node.get_logger().info(f"Motion plan succeeded: {response.message}")
            self._set_output("trajectory", response.trajectory)
            return Status.SUCCESS
        else:
            self.node.get_logger().error(f"Motion plan failed: {response.message}")
            return Status.FAILURE


class RoboplanPlanToPose(RosServiceClientBase):
    """
    Uses a RoboPlan planning server to plan a free-space motion to a target
    end effector pose (IK to find a goal configuration, then planning).
    """

    def __init__(self, name: str, **kwargs: Any):
        super().__init__(name, service_type=PlanToPose, **kwargs)

    INPUT_PORTS = {
        "group_name": PortInformation(data_type=str, required=False),
        "target_pose": PortInformation(data_type=PoseStamped, required=True),
        "velocity_scaling": PortInformation(data_type=float, required=False),
        "acceleration_scaling": PortInformation(data_type=float, required=False),
        "constrain_gripper_top_down": PortInformation(data_type=bool, required=False),
    }

    OUTPUT_PORTS = {"trajectory": PortInformation(data_type=JointTrajectory)}

    def create_request(self) -> PlanToPose.Request:
        """Create the planning service request."""
        return PlanToPose.Request(
            group_name=self.get_input("group_name", ""),
            target_pose=self.get_input("target_pose"),
            velocity_scaling=self.get_input("velocity_scaling", 0.0),
            acceleration_scaling=self.get_input("acceleration_scaling", 0.0),
            constrain_gripper_top_down=self.get_input("constrain_gripper_top_down", False),
        )

    def process_response(self, response: PlanToPose.Response) -> Status:
        """Process the planning service response."""
        if response.success:
            self.node.get_logger().info(f"Motion plan succeeded: {response.message}")
            self._set_output("trajectory", response.trajectory)
            return Status.SUCCESS
        else:
            self.node.get_logger().error(f"Motion plan failed: {response.message}")
            return Status.FAILURE


class RoboplanPlanCartesianPath(RosServiceClientBase):
    """
    Uses a RoboPlan planning server to plan a straight-line Cartesian motion
    through one or more target poses from the current end effector pose.

    The target_poses port accepts a single PoseStamped or a list of
    PoseStamped. A single pose produces a straight line to one target;
    a list is used for multi-waypoint paths.

    If start_trajectory is set, the motion starts from that trajectory's last
    point instead of the measured joint states. This keeps the command continuous
    under compliant control, where the commanded and measured positions differ.
    """

    def __init__(self, name: str, **kwargs: Any):
        super().__init__(name, service_type=PlanCartesianPath, **kwargs)

    INPUT_PORTS = {
        "group_name": PortInformation(data_type=str, required=False),
        "target_poses": PortInformation(data_type=PoseStamped | list[PoseStamped], required=True),
        "max_linear_speed": PortInformation(data_type=float, required=False),
        "max_angular_speed": PortInformation(data_type=float, required=False),
        "start_trajectory": PortInformation(data_type=JointTrajectory, required=False),
    }

    OUTPUT_PORTS = {"trajectory": PortInformation(data_type=JointTrajectory)}

    def create_request(self) -> PlanCartesianPath.Request:
        """Create the planning service request."""
        target = self.get_input("target_poses")
        if isinstance(target, PoseStamped):
            target = [target]
        request = PlanCartesianPath.Request(
            group_name=self.get_input("group_name", ""),
            target_poses=target,
            max_linear_speed=self.get_input("max_linear_speed", 0.0),
            max_angular_speed=self.get_input("max_angular_speed", 0.0),
        )
        start_trajectory = self.get_input("start_trajectory", JointTrajectory())
        if start_trajectory.points:
            request.start_joint_names = list(start_trajectory.joint_names)
            request.start_positions = list(start_trajectory.points[-1].positions)
        return request

    def process_response(self, response: PlanCartesianPath.Response) -> Status:
        """Process the planning service response."""
        if response.success:
            self.node.get_logger().info(f"Motion plan succeeded: {response.message}")
            self._set_output("trajectory", response.trajectory)
            return Status.SUCCESS
        else:
            self.node.get_logger().error(f"Motion plan failed: {response.message}")
            return Status.FAILURE


class ExecuteJointTrajectory(RosActionClientBase):
    """
    Executes a joint trajectory by sending it directly to a ROS 2 joint
    trajectory controller's FollowJointTrajectory action.

    The action name is an input port, so that subtrees can pass it through, and the
    action client is created when the behavior starts with a new action name.
    """

    def __init__(self, name: str, **kwargs: Any):
        super().__init__(name, action_type=FollowJointTrajectory, action_name="", **kwargs)

    INPUT_PORTS = {
        "trajectory": PortInformation(data_type=JointTrajectory, required=True),
        "action_name": PortInformation(
            data_type=str,
            required=True,
            description="The FollowJointTrajectory action of the controller to execute on.",
        ),
    }

    OUTPUT_PORTS = {}

    def setup(self, **kwargs):
        """Gets the ROS node. The action client is created in initialise(), once the action name is known."""
        set_ros_node(self, **kwargs)
        self.action_client = None
        self.client_ready = False
        self.goal_handle = None
        self.get_result_future = None
        self.action_start_time = None
        self.client_start_time = None

    def initialise(self):
        """Creates the action client if the action name changed, then resets the internal variables."""
        action_name = self.get_input("action_name")
        if self.action_client is None or action_name != self.action_name:
            if self.action_client is not None:
                self.action_client.destroy()
            self.action_name = action_name
            self.action_client = ActionClient(node=self.node, action_type=self.action_type, action_name=action_name)
        super().initialise()

    def shutdown(self):
        """Cleans up the action client, if it was created."""
        if self.action_client is not None:
            super().shutdown()

    def create_goal(self) -> FollowJointTrajectory.Goal:
        """Create a trajectory execution goal."""
        return FollowJointTrajectory.Goal(trajectory=self.get_input("trajectory"))

    def process_result(self, result: Any) -> Status:
        """Process the trajectory execution action result."""
        error_code = result.error_code
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.node.get_logger().info("Trajectory execution succeeded!")
            return Status.SUCCESS
        else:
            error_code_str = FOLLOW_JOINT_TRAJECTORY_ERROR_DICT.get(error_code, "UNKNOWN")
            self.node.get_logger().error(f"Trajectory execution failed with error code: {error_code_str}")
            self.node.get_logger().error(f"Message: {result.error_string}")
            return Status.FAILURE


class SetAllowedCollisions(RosServiceClientBase):
    """
    Toggle collision checking for one or more body pairs via the RoboPlan
    planning server's ~/set_collisions service.

    Ports
    -----
    Inputs:
        body1   (list[str]) – first body in each pair
        body2   (list[str]) – second body in each pair
        allowed (bool)      – True to allow collisions (disable checking),
                              False to disallow (enable checking)
    """

    def __init__(self, name: str, **kwargs):
        super().__init__(name, service_type=SetCollisions, **kwargs)

    INPUT_PORTS = {
        "body1": PortInformation(data_type=list, required=True),
        "body2": PortInformation(data_type=list, required=True),
        "allowed": PortInformation(data_type=bool, required=True),
    }

    OUTPUT_PORTS = {}

    def create_request(self) -> SetCollisions.Request:
        return SetCollisions.Request(
            body1=self.get_input("body1"),
            body2=self.get_input("body2"),
            enable=not self.get_input("allowed"),
        )

    def process_response(self, response: SetCollisions.Response) -> Status:
        if response.success:
            self.node.get_logger().info(response.message)
            return Status.SUCCESS
        else:
            self.node.get_logger().error(response.message)
            return Status.FAILURE
