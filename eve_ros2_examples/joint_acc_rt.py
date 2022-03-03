#!/usr/bin/env python3

import rclpy
import rclpy.qos
from halodi_msgs.msg import (
    JointName,
    JointSpaceCommand,
    WholeBodyControllerCommand
)
from rclpy.node import Node



def generate_joint_space_acc_command_msg(joint_id, qdd_desired):

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
            "joint_acc_rt"
        )  # initialize the underlying Node with the name whole_body_robot_bringup

        # Create publisher to be send to at 500 Hz
        # Proper implementation subscribes and waits for WholeBodyState and then sends its commands
        self._publisher = self.create_publisher(
            WholeBodyControllerCommand, "/eve/whole_body_command", rclpy.qos.qos_profile_system_default
        )

        # The joint to set torque to
        self.joint = JointName.LEFT_ELBOW_PITCH
        self.acc_ref = -0.5

        self.dt = 0.002  

        self.timer = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
        
        # Create message
        self._whole_body_command_msg = WholeBodyControllerCommand();
        self._whole_body_command_msg.joint_space_commands.append(generate_joint_space_acc_command_msg(self.joint, self.acc_ref))
        self.get_logger().info("Set acceleration reference")

        # Send message 
        self._publisher.publish(self._whole_body_command_msg)

    def reset(self):
        # Create message
        self._whole_body_command_msg = WholeBodyControllerCommand();
        self._whole_body_command_msg.joint_space_commands.append(generate_joint_space_acc_command_msg(self.joint, 0.0))
        self.get_logger().info("Set zero acceleration reference, stopped")

        # Send message 
        self._publisher.publish(self._whole_body_command_msg)


def main():

    rclpy.init()  

    node = WholeBodyCommandPublisher()

    # Spin here keeps the program running instead of finishing, such that the ticker can interrupt still, don't know why this doesn't work with a while True loop
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.reset()
        pass
        
    rclpy.shutdown()

if __name__ == "__main__":
    main()
