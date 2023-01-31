#!/usr/bin/env python3
import math

import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32, Int32
import rosbag


class TaskStraightNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(TaskStraightNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)
        self._speed_gain = rospy.get_param("~speed_gain")
        self._steer_gain = rospy.get_param("~steer_gain")
        self._simulated_vehicle_length = rospy.get_param("~simulated_vehicle_length")

        self.last_vel=None
        self.stat_idle,self.stat_fwd,self.stat_bwd,self.stat_stop=range(4)
        self.status=self.stat_idle
        self.remaining_dist=0

        # Subscribing to the wheel encoders
        # self.sub_encoder_ticks_left = rospy.Subscriber("~tick_l", WheelEncoderStamped, self.cb_enc_l)
        # self.sub_encoder_ticks_right = rospy.Subscriber("~tick_r",WheelEncoderStamped, self.cb_enc_r)
        # self.sub_executed_commands = rospy.Subscriber("~cmd", WheelsCmdStamped, self.cb_executed_commands)
        self.sub_velocity = rospy.Subscriber(
            "~velocity", Twist2DStamped, self.velocity_callback, queue_size=1
        )

        # Publishers
        self.pub_car_cmd = rospy.Publisher(
            "~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        self.log("Initialized")

    def on_shutdown(self):
        pass

    def velocity_callback(self,msg):
        if (not self.status==self.stat_idle) and (self.last_vel):
            dt=(msg.header.stamp-self.last_vel.header.stamp).to_sec()
            delta_x=self.last_vel.v*dt
            self.remaining_dist+=delta_x
            if self.status==self.stat_fwd and self.remaining_dist>0:
                self.status=self.stat_idle
                self.remaining_dist=0
            if self.status==self.stat_bwd and self.remaining_dist<0:
                self.status=self.stat_idle
                self.remaining_dist=0
        self.last_vel=msg

    def state_loop(self, state, dist):
        self.status = state
        self.remaining_dist = -dist if state==self.stat_fwd else dist
        while self.status == state:
            car_cmd_msg = Twist2DStamped()
            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.v = 0.41 * self._speed_gain
            if state == self.stat_bwd:
                car_cmd_msg.v = -car_cmd_msg.v
            self.pub_car_cmd.publish(car_cmd_msg)


    def run(self):
        self.state_loop(self.stat_fwd,1.25)
        self.state_loop(self.stat_bwd, 1.25)


if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    # rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")
    node.run()