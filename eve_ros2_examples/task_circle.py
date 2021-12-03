#!/usr/bin/env python3

import numpy as np
import rclpy
import rclpy.qos
from halodi_msgs.msg import (
    ReferenceFrameName,
    TaskSpaceCommand,
    WholeBodyControllerCommand
)
from rclpy.node import Node
from scipy.spatial.transform import Rotation

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

class WholeBodyCommandPublisher(Node):
    """A helper/example class to publish whole body controller messages.
    """

    def __init__(self, whole_body_command_msg=None):
        super().__init__(
            "task_circle"
        )  # initialize the underlying Node with the name hands_position_circle_rt

        # Publisher for WholeBodyControllerCommand, 10 is overloaded for being 10 deep history QoS
        self._publisher = self.create_publisher(
            WholeBodyControllerCommand, "/eve/whole_body_command", 10
        )

        # Keep time for position control
        self.t = 0.0
        self.dt = 0.002

        # Create timer that calls every self.dt
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
        
        whole_body_command_msg_ = WholeBodyControllerCommand();

        # Append two half circle tracing position control with both hands using some trigioiniometry
        # Don't get too close with hands together because otherwise eve hits herself
        whole_body_command_msg_.task_space_commands.append(generate_task_space_command_msg(
            ReferenceFrameName.RIGHT_HAND, 
            ReferenceFrameName.PELVIS, 
            [0.4, -np.cos(2*np.pi*self.t/5)/10-0.3, np.sin(2*np.pi*self.t/10)/5+0.2, 0.0, -np.deg2rad(90.0), 0.0]
            ))
        whole_body_command_msg_.task_space_commands.append(generate_task_space_command_msg(
            ReferenceFrameName.LEFT_HAND, 
            ReferenceFrameName.PELVIS, 
            [0.4, np.cos(2*np.pi*self.t/5)/10+0.3, np.sin(2*np.pi*self.t/10)/5+0.2, 0.0, -np.deg2rad(90.0), 0.0]
            ))

        self.t += self.dt

        self._publisher.publish(whole_body_command_msg_)

def main():

    rclpy.init()

    node = WholeBodyCommandPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
