#!/usr/bin/env python3

import rclpy
import rclpy.qos
from geometry_msgs.msg import Vector3
from halodi_msgs.msg import (FeedbackParameters3D,
                             ReferenceFrameName,
                             TaskSpaceCommand,
                             WholeBodyControllerCommand,
                             WholeBodyState,
                             JointNullSpaceConfiguration,
                             JointName
                             )
from rclpy.node import Node
from scipy.spatial.transform import Rotation

def generate_task_space_command_msg(
    body_frame_id, expressed_in_frame_id, xyzrpy, wbs_msg, z_up=True
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


    # Set nullspace of shoulder not to collide with eve's side, not working as intended yet
    jnsc0 = JointNullSpaceConfiguration()
    jnsc0.joint = JointName()
    jnsc0.joint.joint_id = JointName.RIGHT_SHOULDER_PITCH
    jnsc0.q_nullspace = -1.0

    jnsc1 = JointNullSpaceConfiguration()
    jnsc1.joint = JointName()
    jnsc1.joint.joint_id = JointName.RIGHT_SHOULDER_ROLL
    jnsc1.q_nullspace = 0.0

    jnsc2 = JointNullSpaceConfiguration()
    jnsc2.joint = JointName()
    jnsc2.joint.joint_id = JointName.RIGHT_SHOULDER_YAW
    jnsc2.q_nullspace = 0.0

    jnsc3 = JointNullSpaceConfiguration()
    jnsc3.joint = JointName()
    jnsc3.joint.joint_id = JointName.RIGHT_ELBOW_PITCH
    jnsc3.q_nullspace = 0.0

    jnsc4 = JointNullSpaceConfiguration()
    jnsc4.joint = JointName()
    jnsc4.joint.joint_id = JointName.RIGHT_ELBOW_YAW
    jnsc4.q_nullspace = 0.0

    jnsc5 = JointNullSpaceConfiguration()
    jnsc5.joint = JointName()
    jnsc5.joint.joint_id = JointName.RIGHT_WRIST_PITCH
    jnsc5.q_nullspace = 0.0

    jnsc6 = JointNullSpaceConfiguration()
    jnsc6.joint = JointName()
    jnsc6.joint.joint_id = JointName.RIGHT_WRIST_ROLL
    jnsc6.q_nullspace = 0.0

    # Must be a list of 7, prolly for each degree/joint of freedom in the arm
    msg_.nullspace_command = [jnsc0,jnsc1,jnsc2,jnsc3,jnsc4,jnsc5,jnsc6]


    # for joint in range(JointName.RIGHT_SHOULDER_PITCH, JointName.RIGHT_WRIST_ROLL + 1):
    #     msg_.nullspace_command[joint - JointName.RIGHT_WRIST_PITCH] = wbs_msg.JointMeasurement[JointName.RIGHT_WRIST_PITCH].position
        # if joint is JointName.RIGHT_SHOULDER_PITCH:
        #     msg_.nullspace_command[joint - JointName.RIGHT_WRIST_PITCH] = 0.0

    return msg_

class WholeBodyCommandPublisher(Node):

    def __init__(self, whole_body_command_msg=None):
        super().__init__(
            "task_pos_null_rt"
        )

        # Proper implementation subscribes and waits for WholeBodyState and then sends its WholeBodyControllerCommand
        self._publisher = self.create_publisher(
            WholeBodyControllerCommand, "/eve/whole_body_command", rclpy.qos.qos_profile_action_status_default 
        )

        # Subscribe to WholeBodyState, which should return at around 500Hz
        self._subscriber = self.create_subscription(
            WholeBodyState, "/eve/whole_body_state", self.whole_body_state_cb, rclpy.qos.qos_profile_sensor_data
        )
        
        # Relative frames to set position to
        self.frame = ReferenceFrameName.RIGHT_HAND
        self.reference_frame = ReferenceFrameName.PELVIS


    def whole_body_state_cb(self, msg):
        self._whole_body_command_msg = WholeBodyControllerCommand();
        self._whole_body_command_msg.task_space_commands.append(generate_task_space_command_msg(
            self.frame, self.reference_frame, [0.15, -0.25, 0.25, 0.0, -3.14/2, 0.0], msg
            ))
        self._publisher.publish(self._whole_body_command_msg)
        self.get_logger().info("Set position reference")


    def reset(self):
        self._whole_body_command_msg = WholeBodyControllerCommand();
        self._whole_body_command_msg.task_space_commands.append(generate_task_space_command_msg(
            self.frame, self.reference_frame, [0.15, -0.25, 0.25, 0.0, -3.14/2, 0.0]
            ))
        self._publisher.publish(self._whole_body_command_msg)
        self.get_logger().info("Set default reference, stopping")

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
