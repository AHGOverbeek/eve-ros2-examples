#!/usr/bin/env python3

import rclpy
import rclpy.qos
from geometry_msgs.msg import Vector3
from halodi_msgs.msg import (FeedbackParameters3D,
                             ReferenceFrameName,
                             TaskSpaceCommand,
                             WholeBodyControllerCommand,
                             WholeBodyState
                             )
from rclpy.node import Node
from geometry_msgs.msg import Vector3

def generate_task_space_acc_command_msg(
    body_frame_id, expressed_in_frame_id, xyzrpydd, z_up=True
):
    """Generates a task space command msg.

    """

    msg_ = TaskSpaceCommand(express_in_z_up=z_up)
    msg_.body_frame.frame_id = body_frame_id
    msg_.expressed_in_frame.frame_id = expressed_in_frame_id

    # Initialize taskspace zero PD controllers
    fbpar = FeedbackParameters3D()
    vec0 = Vector3(x = 0.0, y = 0.0, z = 0.0)
    fbpar.proportional = vec0
    fbpar.derivative = vec0
    # Disable PD the taskspace controllers
    msg_.position_feedback_parameters = [fbpar]
    msg_.orientation_feedback_parameters = [fbpar]

    # Then only the taskspace acceleration controller remains
    msg_.linear_acceleration.x = xyzrpydd[0]
    msg_.linear_acceleration.y = xyzrpydd[1]
    msg_.linear_acceleration.z = xyzrpydd[2]
    msg_.angular_acceleration.x = xyzrpydd[3]
    msg_.angular_acceleration.y = xyzrpydd[4]
    msg_.angular_acceleration.z = xyzrpydd[5]

    return msg_

class WholeBodyCommandPublisher(Node):

    def __init__(self, whole_body_command_msg=None):
        super().__init__(
            "task_acceleration_rt"
        )

        # Proper implementation subscribes and waits for WholeBodyState and then sends its WholeBodyControllerCommand
        self._publisher = self.create_publisher(
            WholeBodyControllerCommand, "/eve/whole_body_command", rclpy.qos.qos_profile_action_status_default 
        )

        # Subscribe to WholeBodyState, which should return at around 500Hz
        self._subscriber = self.create_subscription(
            WholeBodyState, "/eve/whole_body_state", self.whole_body_state_cb, rclpy.qos.qos_profile_sensor_data
        )

        # Relative frames to set force to
        self.frame = ReferenceFrameName.RIGHT_HAND
        self.reference_frame = ReferenceFrameName.PELVIS


    def whole_body_state_cb(self, msg):
        self._whole_body_command_msg = WholeBodyControllerCommand();
        self._whole_body_command_msg.task_space_commands.append(generate_task_space_acc_command_msg(
            self.frame, self.reference_frame, [0.0, 0.0, -0.2, 0.0, 0.0, 0.0]
            ))
        self._publisher.publish(self._whole_body_command_msg)
        self.get_logger().info("Set acceleration reference")
        
    def reset(self):
        self._whole_body_command_msg = WholeBodyControllerCommand();
        self._whole_body_command_msg.task_space_commands.append(generate_task_space_acc_command_msg(
            self.frame, self.reference_frame, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            ))
        self._publisher.publish(self._whole_body_command_msg)
        self.get_logger().info("Set zero acceleration reference, stopping")

def main():

    rclpy.init()

    node = WholeBodyCommandPublisher()

    # Spin here keeps the script waiting for what its subscribed to instead of finishing
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.reset()
        pass
        
    rclpy.shutdown()

if __name__ == "__main__":
    main()
