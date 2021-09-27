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
from geometry_msgs.msg import Vector3
from halodi_msgs.msg import (FeedbackParameters3D, JointName,
                             JointSpaceCommand, ReferenceFrameName,
                             TaskSpaceCommand, TrajectoryInterpolation,
                             WholeBodyControllerCommand, WholeBodyTrajectory,
                             WholeBodyTrajectoryPoint)
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from unique_identifier_msgs.msg import UUID
from geometry_msgs.msg import Vector3

def generate_uuid_msg():
    """Generates a UUID msg based on the current time.

    Parameters: None

    Returns: UUID msg
    """
    return UUID(uuid=np.asarray(list(uuid.uuid1().bytes)).astype(np.uint8))


def generate_task_space_acc_command_msg(
    body_frame_id, expressed_in_frame_id, xyzrpydd, z_up=True
):
    """Generates a task space command msg.

    """

    msg_ = TaskSpaceCommand(express_in_z_up=z_up)
    msg_.body_frame.frame_id = body_frame_id
    msg_.expressed_in_frame.frame_id = expressed_in_frame_id

    # Initialize zero PD controllers
    fbpar = FeedbackParameters3D()
    vec0 = Vector3(x = 0.0, y = 0.0, z = 0.0)
    fbpar.proportional = vec0
    fbpar.derivative = vec0
    # Disable PD the controllers
    msg_.position_feedback_parameters = [fbpar]
    msg_.orientation_feedback_parameters = [fbpar]

    msg_.linear_acceleration.x = xyzrpydd[0]
    msg_.linear_acceleration.y = xyzrpydd[1]
    msg_.linear_acceleration.z = xyzrpydd[2]
    msg_.angular_acceleration.x = xyzrpydd[3]
    msg_.angular_acceleration.y = xyzrpydd[4]
    msg_.angular_acceleration.z = xyzrpydd[5]

    return msg_


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


class WholeBodyCommandPublisher(Node):
    """A helper/example class to publish whole body controller messages.
    """

    def __init__(self, whole_body_command_msg=None):
        super().__init__(
            "right_hand_accleration_rt"
        )  # initialize the underlying Node with the name whole_body_robot_bringup

        # 10 is overloaded for being 10 deep history QoS
        self._publisher = self.create_publisher(
            WholeBodyControllerCommand, "/eve/whole_body_command", rclpy.qos.qos_profile_action_status_default 
        )

        self._subscriber = self.create_subscription(
            GoalStatus, "/eve/whole_body_trajectory_status", self.goal_status_cb, 10
        )  # create a GoalStatus subscriber with inbound queue size of 10

        # # store periodic_trajectory_msg for re-publishing in goal_status_cb
        self._whole_body_command_msg = whole_body_command_msg

        timer_period = 0.002  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self._publisher.publish(self._whole_body_command_msg)

    def goal_status_cb(self, msg):
        """GoalStatus callback. Logs/prints some statuses and re-pubishes
           periodic_trajectory_msg if it was provided to the constructor.

        Parameters:
        - msg (GoalStatus): msg from a GoalStatus subscription

        Returns: None
        """

def run_warmup_loop(args=None):
    """An example function that moves all the joints in a repeated movement sequence.

    Parameters:
    - args (?): for rclpy.init(). Default: None

    Returns: None
    """


    rclpy.init(args=args)  # initialize rclpy


    whole_body_command_msg_ = WholeBodyControllerCommand();
    whole_body_command_msg_.task_space_commands.append(generate_task_space_acc_command_msg(
        ReferenceFrameName.RIGHT_HAND, ReferenceFrameName.PELVIS, [-0.0, 0.0, 0.5, 0.0, 0.0, 0.0]
        ))

    wbcp_ = WholeBodyCommandPublisher(
        whole_body_command_msg_
    )  # create the helper class
    rclpy.spin(
        wbcp_
    )  # spin the node in the WholeBodyTrajectoryPublisher for blocking and pub/sub functionality

    wbcp_.destroy_node()  # shut down the node
    rclpy.shutdown()  # shut down rclpy


if __name__ == "__main__":
    run_warmup_loop()
