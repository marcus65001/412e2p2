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
        # self._speed_gain = rospy.get_param("~speed_gain")
        # self._steer_gain = rospy.get_param("~steer_gain")
        self._speed_gain = 0.41
        self._steer_gain = 0.41
        self._dist_factor = 1.9
        self._dist=rospy.get_param("~dist", 1.25)*self._dist_factor
        # self._simulated_vehicle_length = rospy.get_param("~simulated_vehicle_length")

        self.last_vel = None
        self.stat_idle, self.stat_fwd, self.stat_bwd, self.stat_stop = range(4)
        self.status = self.stat_idle
        self.queue = []
        self.remaining_dist = 0

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

    def velocity_callback(self, msg):
        if (not self.status == self.stat_stop) and (self.last_vel):
            dt = (msg.header.stamp - self.last_vel.header.stamp).to_sec()
            delta_x = self.last_vel.v * dt
            # delta_theta = self.last_vel.theta * dt
            self.remaining_dist -= np.abs(delta_x)
            print(delta_x, self.remaining_dist)
            if self.remaining_dist < 0:
                self.state_pop()
        self.last_vel = msg

    def pub_command(self):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.get_rostime()
        car_cmd_msg.v = self._speed_gain
        if self.status in {self.stat_stop,self.stat_idle}:
            car_cmd_msg.v = 0
        if self.status == self.stat_bwd:
            car_cmd_msg.v = -car_cmd_msg.v
        self.pub_car_cmd.publish(car_cmd_msg)
    def state_pop(self):
        print('pop')
        if self.queue:
            self.status, self.remaining_dist = self.queue.pop()
            print(self.status, self.remaining_dist)
        else:
            self.status = self.stat_stop
            self.pub_command()
            rospy.sleep(1.0)
            rospy.signal_shutdown("done")

    def run(self):
        r = rospy.Rate(5)
        self.queue.append((self.stat_bwd, self._dist))
        self.queue.append((self.stat_fwd, self._dist))
        self.state_pop()
        while not rospy.is_shutdown():
            self.pub_command()
            r.sleep()


if __name__ == '__main__':
    node = TaskStraightNode(node_name='task_straight_node')
    # Keep it spinning to keep the node alive

    rospy.loginfo("task_straight_node is up and running...")
    node.run()
    rospy.spin()