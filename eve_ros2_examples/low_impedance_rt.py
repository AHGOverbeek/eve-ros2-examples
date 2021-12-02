#!/usr/bin/env python3

import rclpy
import rclpy.qos
from halodi_msgs.msg import (
    JointName,
    JointSpaceCommand,
    WholeBodyControllerCommand
)
from rclpy.node import Node
from time import sleep


def generate_joint_space_acc_command_msg(joint_id, qdd_desired):

    msg_ = JointSpaceCommand(joint=JointName(joint_id=joint_id))

    # Disable PD control such that only feedforward acc control is used, setting the acceleration command to zero then sets low impedance joints
    msg_.use_default_gains = False
    msg_.stiffness = 0.0
    msg_.damping = 0.0
    # Not sure what dampingscale does
    msg_.motorDampingScale = 0.0
    msg_.qdd_desired = qdd_desired

    return msg_

class WholeBodyCommandPublisher(Node):

    def __init__(self, whole_body_command_msg=None):
        super().__init__(
            "low_impedance_rt"
        )  # initialize the underlying Node with the name low_impedance_rt

        # 10 is overloaded for being 10 deep history QoS
        self._publisher = self.create_publisher(
            WholeBodyControllerCommand, "/eve/whole_body_command", rclpy.qos.qos_profile_system_default
        )

        # Attach the message defined in main to self
        self._whole_body_command_msg = whole_body_command_msg

    def run(self):
        # Send the message
        self._publisher.publish(self._whole_body_command_msg)


def main():

    # Initialize rclpy
    rclpy.init()  

    # Initialize 
    whole_body_command_msg_ = WholeBodyControllerCommand();

    # Make left and/or right low impedance
    left = True 
    right = True

    # The joint are just numbers
    for joint in left*list(range(JointName.LEFT_SHOULDER_PITCH, JointName.LEFT_WRIST_ROLL + 1)) + right*list(range(JointName.RIGHT_SHOULDER_PITCH, JointName.RIGHT_WRIST_ROLL + 1)):
        # Append a zero desired acceleration command for each joint in the loop
        whole_body_command_msg_.joint_space_commands.append(generate_joint_space_acc_command_msg(joint, 0.0))

    # Initialize the node, sleep to allow setup of the ros node or rclpy (takes some asynchornous time), then send the message once
    node = WholeBodyCommandPublisher(whole_body_command_msg_)
    sleep(0.01)
    node.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
