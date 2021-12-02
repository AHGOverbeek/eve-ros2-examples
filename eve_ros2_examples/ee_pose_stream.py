#!/usr/bin/env python3

import rclpy
import rclpy.qos
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TFMessage
from rclpy.node import Node
from numpy import arccos
from numpy import sin


def dprint(arg):
    print(['{:.3f}'.format(x) for x in arg])

class EndEffectorPoseListener(Node):

    def __init__(self):
        super().__init__(
            "end_effector_pose_listener"
        )  
 
        # Create a listener/subscriber to the tf topic (done inside TransformListener) and save in a buffer, these are functions made for this purpose 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create another subscriber, here purely to have a reason to print on callback as soon as arrives
        self._subscriber = self.create_subscription(TFMessage, "/tf", self.tf_cb, 10)

        # Frames that we want to know
        self.to_frame_rel = 'World'
        self.from_frame_rel = 'l_palm'

    def tf_cb(self, msg):

        # This reads zero always, and therefore probably does not get the latest value in the buffer, find an alternative way to get the time
        now = rclpy.time.Time() 

        # Try because it doesn't work the very first time
        try:
            trans = self.tf_buffer.lookup_transform(self.to_frame_rel, self.from_frame_rel, now)
            translation = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]

            # Print translatino and rotation of the quaternions to check
            dprint(translation)

            # dprint(rotation) 
            # quaternion rotation can be hard to interpret, but the parametrization from https://eater.net/quaternions is more intuitive
            dprint(x/sin(arccos(rotation[-1])) for x in rotation[0:-1])
            dprint([2/2*arccos(rotation[-1])*180/3.14])
        except:
            pass

def main():

    rclpy.init()
    node = EndEffectorPoseListener()

    # Spin means wait and listen for what you're subscribed to, otherwise the script would just end
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
