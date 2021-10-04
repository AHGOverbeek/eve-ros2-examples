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
    WholeBodyState,
    WholeBodyControllerCommand
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


def generate_task_space_command_msg(
    body_frame_id, expressed_in_frame_id, xyzrpy, z_up=True
):
    """Generates a task space command msg.

    Parameters:
    - body_frame_id (enum): body part to be moved, e.g. ReferenceFrameName.PELVIS
    - expressed_in_frame_id (enum): reference frame for body_frame_id, e.g. ReferenceFrameName.BASE
    - xyzrpy (array of 6 floats): desired pose of body_frame_id relative to expressed_in_frame_id
      , as a list/tuple/1D np.array of [ posX, posY, posZ, rotX, rotY, rotZ ]
    - z_up (bool): whether or not xyzrpy follows the Z-up co-ordinate convention. Default: True

    Returns: TaskSpaceCommand msg
    """

    msg_ = TaskSpaceCommand(express_in_z_up=z_up)
    msg_.body_frame.frame_id = body_frame_id
    msg_.expressed_in_frame.frame_id = expressed_in_frame_id

    msg_.pose.position.x = xyzrpy[0]
    msg_.pose.position.y = xyzrpy[1]
    msg_.pose.position.z = xyzrpy[2]
    quat_ = Rotation.from_euler("xyz", xyzrpy[3:]).as_quat()  # Euler to quaternion
    msg_.pose.orientation.x = quat_[0]
    msg_.pose.orientation.y = quat_[1]
    msg_.pose.orientation.z = quat_[2]
    msg_.pose.orientation.w = quat_[3]

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

def generate_joint_space_acc_command_msg(
    joint_id, qdd_desired
):
    """Generates a joint space acc command msg.

    """

    msg_ = JointSpaceCommand(joint=JointName(joint_id=joint_id))
    # Disable PD control such that only feedforward acc control
    msg_.use_default_gains = False
    msg_.stiffness = 0.0
    msg_.damping = 0.0
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
            WholeBodyControllerCommand, "/eve/whole_body_command", rclpy.qos.qos_profile_system_default
        )

        self._subscriber = self.create_subscription(
            WholeBodyState, "/eve/whole_body_state", self.whole_body_state_cb, rclpy.qos.qos_profile_sensor_data
        )  # create a WholeBodyState subscriber with inbound queue size of 10

        # # store periodic_trajectory_msg for re-publishing in goal_status_cb
        self._whole_body_command_msg = whole_body_command_msg

        timer_period = 0.002  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self._publisher.publish(self._whole_body_command_msg)

    def whole_body_state_cb(self, msg):
        """WholeBodyState callback. 

        Parameters:
        - msg (GoalStatus): msg from a GoalStatus subscription

        Returns: None
        """
        diff = msg.joint_states[JointName.RIGHT_ELBOW_PITCH].desiredEffort - msg.joint_states[JointName.RIGHT_ELBOW_PITCH].measuredEffort
        self.get_logger().info("Received whole_body_state, difference between desired and actual force is: {0}"\
            .format(str(diff)))


def run_warmup_loop(args=None):
    """An example function that moves all the joints in a repeated movement sequence.

    Parameters:
    - args (?): for rclpy.init(). Default: None

    Returns: None
    """


    rclpy.init(args=args)  # initialize rclpy


    whole_body_command_msg_ = WholeBodyControllerCommand();

    whole_body_command_msg_.joint_space_commands.append(generate_joint_space_acc_command_msg(
        JointName.RIGHT_ELBOW_PITCH, -0.5
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
