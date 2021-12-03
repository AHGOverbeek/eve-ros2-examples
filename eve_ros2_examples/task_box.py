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
    JointNullSpaceConfiguration
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

class WholeBodyTrajectoryPublisher(Node):

    def __init__(self, periodic_trajectory_msg=None):
        super().__init__(
            "task_box"
        )  # initialize the underlying Node with the name whole_body_robot_bringup

        # 10 is overloaded for being 10 deep history QoS
        self._publisher = self.create_publisher(
            WholeBodyTrajectory, "/eve/whole_body_trajectory", rclpy.qos.qos_profile_action_status_default 
        )

        self._subscriber = self.create_subscription(
            GoalStatus, "/eve/whole_body_trajectory_status", self.goal_status_cb, 10
        )  # create a GoalStatus subscriber with inbound queue size of 10

        # Create publisher to the nullspace configurator
        self._publisher_ns = self.create_publisher(
            JointNullSpaceConfiguration, "/eve/joint_null_space_configuration", rclpy.qos.qos_profile_action_status_default 
        )

        ms = JointNullSpaceConfiguration()
        joint = JointName()
        joint.joint_id = JointName.LEFT_SHOULDER_PITCH
        ms.joint = joint
        ms.q_nullspace = -1.5
        self._publisher_ns.publish(ms)  

        # store periodic_trajectory_msg for re-publishing in goal_status_cb
        self._periodic_trajectory_msg = periodic_trajectory_msg

        # Trajectory messages need an uuid, not properly using it here
        self._periodic_trajectory_msg.trajectory_id = generate_uuid_msg()  # populate UUID
        self.get_logger().info("Publishing first periodic trajectory ...")
        self._publisher.publish(self._periodic_trajectory_msg)  

    def goal_status_cb(self, msg):

        if msg.status == GoalStatus.STATUS_ACCEPTED:
            self.get_logger().info("Goal accepted")
        elif msg.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Goal canceled")
        elif msg.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Goal aborted")
        elif msg.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
            self.get_logger().info("Republishing periodic trajectory ...")
            self._publisher.publish(self._periodic_trajectory_msg)


def main():


    x = 0.25
    y1 = 0.3
    y2 = 0.5
    z1 = 0.25
    z2 = 0.0
    dt = 1

    cumulative_seconds_from_start_ = 0

    cumulative_seconds_from_start_ = cumulative_seconds_from_start_ + dt
    periodic_trajectory_pt_msg_1_ = WholeBodyTrajectoryPoint(
        time_from_start=Duration(sec=cumulative_seconds_from_start_)
    )  # create a trajectory point msg, timestamped for 3 seconds in the future
    periodic_trajectory_pt_msg_1_.task_space_commands.append(
        generate_task_space_command_msg(
            ReferenceFrameName.RIGHT_HAND,
            ReferenceFrameName.PELVIS,
            [x, -y1, z1, 0.0, -np.deg2rad(90.0), 0.0],
        )
    )  # append a desired task space pose for the pelvis WRT base
    # [posX, posY, posZ, roll, pitch, yaw]
    periodic_trajectory_pt_msg_1_.task_space_commands.append(
        generate_task_space_command_msg(
            ReferenceFrameName.LEFT_HAND,
            ReferenceFrameName.PELVIS,
            [x, y1, z1, 0.0, -np.deg2rad(90.0), 0.0],
        )
    )  # append a desired task space pose for the pelvis WRT base
    # [posX, posY, posZ, roll, pitch, yaw]

    cumulative_seconds_from_start_ = cumulative_seconds_from_start_ + dt
    periodic_trajectory_pt_msg_2_ = WholeBodyTrajectoryPoint(
        time_from_start=Duration(sec=cumulative_seconds_from_start_)
    )  # create another trajectory point msg, 1 additional second in the future
    periodic_trajectory_pt_msg_2_.task_space_commands.append(
        generate_task_space_command_msg(
            ReferenceFrameName.RIGHT_HAND,
            ReferenceFrameName.PELVIS,
            [x, -y1, z2, 0.0, -np.deg2rad(90.0), 0.0],
        )
    )  # append a desired task space pose for the pelvis WRT base
    # [posX, posY, posZ, roll, pitch, yaw]
    periodic_trajectory_pt_msg_2_.task_space_commands.append(
        generate_task_space_command_msg(
            ReferenceFrameName.LEFT_HAND,
            ReferenceFrameName.PELVIS,
            [x, y1, z2, 0.0, -np.deg2rad(90.0), 0.0],
        )
    )  # append a desired task space pose for the pelvis WRT base
    # [posX, posY, posZ, roll, pitch, yaw]

    cumulative_seconds_from_start_ = cumulative_seconds_from_start_ + dt
    periodic_trajectory_pt_msg_3_ = WholeBodyTrajectoryPoint(
        time_from_start=Duration(sec=cumulative_seconds_from_start_)
    )  # create another trajectory point msg, 1 additional second in the future
    periodic_trajectory_pt_msg_3_.task_space_commands.append(
        generate_task_space_command_msg(
            ReferenceFrameName.RIGHT_HAND,
            ReferenceFrameName.PELVIS,
            [x, -y2, z2, 0.0, -np.deg2rad(90.0), 0.0],
        )
    )  # append a desired task space pose for the pelvis WRT base
    # [posX, posY, posZ, roll, pitch, yaw]
    periodic_trajectory_pt_msg_3_.task_space_commands.append(
        generate_task_space_command_msg(
            ReferenceFrameName.LEFT_HAND,
            ReferenceFrameName.PELVIS,
            [x, y2, z2, 0.0, -np.deg2rad(90.0), 0.0],
        )
    )  # append a desired task space pose for the pelvis WRT base
    # [posX, posY, posZ, roll, pitch, yaw]

    cumulative_seconds_from_start_ = cumulative_seconds_from_start_ + dt
    periodic_trajectory_pt_msg_4_ = WholeBodyTrajectoryPoint(
        time_from_start=Duration(sec=cumulative_seconds_from_start_)
    )  # create another trajectory point msg, 1 additional second in the future
    periodic_trajectory_pt_msg_4_.task_space_commands.append(
        generate_task_space_command_msg(
            ReferenceFrameName.RIGHT_HAND,
            ReferenceFrameName.PELVIS,
            [x, -y2, z1, 0.0, -np.deg2rad(90.0), 0.0],
        )
    )  # append a desired task space pose for the pelvis WRT base
    # [posX, posY, posZ, roll, pitch, yaw]
    periodic_trajectory_pt_msg_4_.task_space_commands.append(
        generate_task_space_command_msg(
            ReferenceFrameName.LEFT_HAND,
            ReferenceFrameName.PELVIS,
            [x, y2, z1, 0.0, -np.deg2rad(90.0), 0.0],
        )
    )  # append a desired task space pose for the pelvis WRT base
    # [posX, posY, posZ, roll, pitch, yaw]

    periodic_trajectory_msg_ = WholeBodyTrajectory(
        append_trajectory=False
    )  # create a whole body trajectory msg that will
    # override any trajectory currently being executed
    periodic_trajectory_msg_.interpolation_mode.value = (
        TrajectoryInterpolation.MINIMUM_JERK_CONSTRAINED
    )  # choose an interpolation mode
    periodic_trajectory_msg_.trajectory_points.append(periodic_trajectory_pt_msg_1_)
    periodic_trajectory_msg_.trajectory_points.append(periodic_trajectory_pt_msg_2_)
    periodic_trajectory_msg_.trajectory_points.append(periodic_trajectory_pt_msg_3_)
    periodic_trajectory_msg_.trajectory_points.append(periodic_trajectory_pt_msg_4_)

    rclpy.init()  # initialize rclpy

    node = WholeBodyTrajectoryPublisher(periodic_trajectory_msg_)

    # Spin means wait and listen for what you're subscribed to, otherwise the script would just end
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
