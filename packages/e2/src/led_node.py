#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import LEDPattern
from e2.srv import LEDSet,LEDSetResponse
from duckietown_msgs.srv import ChangePattern, ChangePatternResponse
from std_msgs.msg import String


class LedNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(LedNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")


        # Subscribers

        # Publishers
        self.pub_led_cmd = rospy.Publisher(
            "~led_cmd", LEDPattern, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        # Service
        self.srv_led = rospy.Service(
            "~set_led", LEDSet, self.srv_set_led
        )

        self.srvp_led_emitter = rospy.ServiceProxy(
            "~set_pattern", ChangePattern
        )

        self.log("Initialized")

    def on_shutdown(self):
        pass

    def srv_set_led(self, msg):
        self.log(msg)
        msgp = String()
        msgp.data=msg.pattern_name.data
        self.srvp_led_emitter(msgp)
        return LEDSetResponse()



if __name__ == '__main__':
    node = LedNode(node_name='led_node')
    # Keep it spinning to keep the node alive
    rospy.loginfo("led_node is up and running...")
    rospy.spin()
