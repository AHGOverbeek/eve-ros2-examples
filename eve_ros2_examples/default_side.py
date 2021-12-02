#!/usr/bin/env python3

# Copyright 2021 Halodi Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import uuid

import numpy as np
import rclpy
import rclpy.qos
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from halodi_msgs.msg import (
    JointName,
    JointSpaceCommand,
    ReferenceFrameName,
    TaskSpaceCommand,
    TrajectoryInterpolation,
    WholeBodyTrajectory,
    WholeBodyTrajectoryPoint,
)
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from unique_identifier_msgs.msg import UUID


def generate_uuid_msg():
    """Generates a UUID msg based on the current time.

    Parameters: None

    Returns: UUID msg
    """
    return UUID(uuid=np.asarray(list(uuid.uuid1().bytes)).astype(np.uint8))

def generate_joint_space_command_msg(
    joint_id, q_desired, qd_desired=0.0, qdd_desired=0.0
):
    """Generates a joint space command msg.
    This msg has additional gains fields. If you do not wish to set these yourself,
    please ensure that the use_default_gains bool is set to True.
    Msgs generated by this function have use_default_gains set to True.

    Parameters:
    - joint_id (enum): joint to be moved, e.g. JointName.NECK_PITCH
    - q_desired (float): desired final joint position
    - q_desired (float): desired final joint velocity. Default: 0.0
    - q_desired (float): desired final joint acceleration. Default: 0.0

    Returns: JointSpaceCommand msg
    """

    msg_ = JointSpaceCommand(joint=JointName(joint_id=joint_id), use_default_gains=True)
    msg_.q_desired = q_desired
    msg_.qd_desired = qd_desired
    msg_.qdd_desired = qdd_desired

    return msg_


class WholeBodyTrajectoryPublisher(Node):
    """A helper/example class to publish whole body trajectory messages.

    Constructor parameters:
    - initial_trajectory_msg (WholeBodyTrajectory): if not None, this is published first.
      Default: None
    - periodic_trajectory_msg (WholeBodyTrajectory): if not None, this is published
      on a loop upon completion of initial_trajectory_msg if it was provided. Default: None
    """

    def __init__(self, initial_trajectory_msg=None, periodic_trajectory_msg=None):
        super().__init__(
            "all_joint_move"
        )  # initialize the underlying Node with the name all_joint_move

        # 10 is overloaded for being 10 deep history QoS
        # Sometimes QoS = 10 does not work (no idea why, seems platform dependent), if not, use rclpy.qos.qos_profile_action_status_default instead
        self._publisher = self.create_publisher(
            WholeBodyTrajectory, "/eve/whole_body_trajectory", rclpy.qos.qos_profile_action_status_default 
        )

        self._subscriber = self.create_subscription(
            GoalStatus, "/eve/whole_body_trajectory_status", self.goal_status_cb, 10
        )  # create a GoalStatus subscriber with inbound queue size of 10

        if initial_trajectory_msg is not None:
            initial_trajectory_msg.trajectory_id = generate_uuid_msg()  # populate UUID
            self.get_logger().info("Publishing initial trajectory ...")
            self._publisher.publish(
                initial_trajectory_msg
            )  # publish initial_trajectory_msg
        else:
            periodic_trajectory_msg.trajectory_id = generate_uuid_msg()  # populate UUID
            self.get_logger().info("Publishing first periodic trajectory ...")
            self._publisher.publish(
                periodic_trajectory_msg
            )  # publish periodic_trajectory_msg instead

        # store periodic_trajectory_msg for re-publishing in goal_status_cb
        self._periodic_trajectory_msg = periodic_trajectory_msg

        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.status_msg_received_ever = False

    def timer_callback(self):
        if not self.status_msg_received_ever:
            self.get_logger().info("Publishing msg from timer")
            self._publisher.publish(self._periodic_trajectory_msg)

    def goal_status_cb(self, msg):
        """GoalStatus callback. Logs/prints some statuses and re-pubishes
           periodic_trajectory_msg if it was provided to the constructor.

        Parameters:
        - msg (GoalStatus): msg from a GoalStatus subscription

        Returns: None
        """

        if not self.status_msg_received_ever:
            self.timer.cancel()
            self.get_logger().info("Timer is cancelled")
            self.status_msg_received_ever = True

        if msg.status == GoalStatus.STATUS_ACCEPTED:
            self.get_logger().info("Goal accepted")
        elif msg.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Goal canceled")
        elif msg.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Goal aborted")
        elif msg.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
            if self._periodic_trajectory_msg is not None:
                self.get_logger().info("Republishing periodic trajectory ...")
                self._periodic_trajectory_msg.trajectory_id = generate_uuid_msg()
                self._publisher.publish(self._periodic_trajectory_msg)


def run_warmup_loop(args=None):
    """An example function that moves all the joints in a repeated movement sequence.

    Parameters:
    - args (?): for rclpy.init(). Default: None

    Returns: None
    """

    cumulative_seconds_from_start_ = 0

    cumulative_seconds_from_start_ = cumulative_seconds_from_start_ + 1
    periodic_trajectory_pt_msg_1_ = WholeBodyTrajectoryPoint(
        time_from_start=Duration(sec=cumulative_seconds_from_start_)
    )  # create a trajectory point msg, timestamped for 3 seconds in the future
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.RIGHT_SHOULDER_PITCH, 0.2)
    )  # append a desired joint position of 0.5 radians for the pitch of the right shoulder
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.RIGHT_SHOULDER_ROLL, -1.0)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.RIGHT_SHOULDER_YAW, 1.5)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.RIGHT_ELBOW_PITCH, -1.5)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.RIGHT_ELBOW_YAW, -0.0)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.RIGHT_WRIST_PITCH, 0.0)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.RIGHT_WRIST_ROLL, -0.0)
    )

    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.LEFT_SHOULDER_PITCH, 0.2)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.LEFT_SHOULDER_ROLL, +1.0)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.LEFT_SHOULDER_YAW, -1.5)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.LEFT_ELBOW_PITCH, -1.5)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.LEFT_ELBOW_YAW, 0.0)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.LEFT_WRIST_PITCH, 0.0)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.LEFT_WRIST_ROLL, 0.0)
    )

    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.NECK_PITCH, 0.0)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.HIP_PITCH, 0.0)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.HIP_ROLL, 0.0)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.HIP_YAW, 0.0)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.KNEE_PITCH ,0.0)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.ANKLE_PITCH, 0.0)
    )
    periodic_trajectory_pt_msg_1_.joint_space_commands.append(
        generate_joint_space_command_msg(JointName.ANKLE_ROLL, 0.0)
    )


    periodic_trajectory_msg_ = WholeBodyTrajectory(
        append_trajectory=False
    )  # create a whole body trajectory msg that will
    # override any trajectory currently being executed
    periodic_trajectory_msg_.interpolation_mode.value = (
        TrajectoryInterpolation.MINIMUM_JERK_CONSTRAINED
    )  # choose an interpolation mode
    periodic_trajectory_msg_.trajectory_points.append(periodic_trajectory_pt_msg_1_)

    rclpy.init(args=args)  # initialize rclpy

    wbtp_ = WholeBodyTrajectoryPublisher(
        None, periodic_trajectory_msg_
    )  # create the helper class
    rclpy.spin(
        wbtp_
    )  # spin the node in the WholeBodyTrajectoryPublisher for blocking and pub/sub functionality

    wbtp_.destroy_node()  # shut down the node
    rclpy.shutdown()  # shut down rclpy


if __name__ == "__main__":
    run_warmup_loop()
