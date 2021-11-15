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
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TFMessage
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from unique_identifier_msgs.msg import UUID
from numpy import arccos
from numpy import sin


def dprint(arg):
    print(['{:.3f}'.format(x) for x in arg])

class EndEffectorPoseListener(Node):

    def __init__(self):
        super().__init__(
            "end_effector_pose_listener"
        )  
 
        # Create a listener (subscribes as well) to tf and save in a buffer 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create another subscriber, here purely to have a reason to print on callback as soon as arrives
        self._subscriber = self.create_subscription(TFMessage, "/tf", self.cb, 10)

    def cb(self, msg):
        now = rclpy.time.Time()
        to_frame_rel = 'World'
        from_frame_rel = 'l_palm'
        try:
            trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, now)
            translation = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]

            dprint(translation)

            # pure
            # dprint(rotation) 
            # 3b1b vector and degrees to be filled in
            dprint(x/sin(arccos(rotation[-1])) for x in rotation[0:-1])
            dprint([2/2*arccos(rotation[-1])*180/3.14])
        except:
            pass

def run_warmup_loop():

    rclpy.init()
    node = EndEffectorPoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    run_warmup_loop()
