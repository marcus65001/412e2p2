#!/usr/bin/env python3
import math

import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import LEDPattern, ColorRGBA
# from e2.srv import LEDSet
from duckietown_msgs.srv import SetCustomLEDPatternResponse
from std_msgs.msg import ColorRGBA

class LedNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(LedNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters


        # Subscribers

        # Publishers
        self.pub_led_cmd = rospy.Publisher(
            "~led_cmd", LEDPattern, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        # Service
        self.srv_set_camera_info = rospy.Service(
            "~set_led", ColorRGBA, self.srv_set_led
        )

        self.log("Initialized")

    def on_shutdown(self):
        pass

    def srv_set_led(self,msg):
        ledp=LEDPattern()
        for i in range(5):
            ledp.rgb_vals.append(msg)
        self.pub_led_cmd.publish(ledp)
        return None


if __name__ == '__main__':
    node = LedNode(node_name='led_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")
