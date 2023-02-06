#!/usr/bin/env python3
import math

import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import LEDPattern
from e2.srv import LEDSet,LEDSetResponse
from duckietown_msgs.srv import SetCustomLEDPattern, SetCustomLEDPatternResponse
from std_msgs.msg import ColorRGBA

class LedNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(LedNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # properties
        self._led_pattern=LEDPattern()
        for i in range(5):
            color=ColorRGBA()
            color.r=255
            color.g=255
            color.b=255
            color.a=1.0
            self._led_pattern.rgb_vals.append(color)

        # Subscribers

        # Publishers
        self.pub_led_cmd = rospy.Publisher(
            "~led_cmd", LEDPattern, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        # Service
        self.srv_set_camera_info = rospy.Service(
            "~set_led", LEDSet, self.srv_set_led
        )

        self.srvp_led_emitter = rospy.ServiceProxy(
            "~led_emitter", SetCustomLEDPattern
        )

        self.log("Initialized")

    def on_shutdown(self):
        pass

    def srv_set_led(self, msg):
        ledp=LEDPattern()
        for i in range(5):
            color=ColorRGBA()
            color.r=msg.r
            color.g=msg.g
            color.b=msg.b
            color.a=1.0
            ledp.rgb_vals.append(color)
        # self.pub_led_cmd.publish(ledp)
        self.srvp_led_emitter(ledp)
        # self._led_pattern=ledp
        return LEDSetResponse()

    def pubPattern(self):
        self.pub_led_cmd.publish(self._led_pattern)

    def run(self):
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.pubPattern()
            r.sleep()



if __name__ == '__main__':
    node = LedNode(node_name='led_node')
    # Keep it spinning to keep the node alive
    rospy.loginfo("led_node is up and running...")
    # node.run()
    rospy.spin()
