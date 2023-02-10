#!/usr/bin/env python3
import math

import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32, Int32
import rosbag


class TaskRotationNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(TaskRotationNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)
        # self._speed_gain = rospy.get_param("~speed_gain")
        # self._steer_gain = rospy.get_param("~steer_gain")
        self._speed_gain = 0.41
        self._steer_gain = rospy.get_param("/e2/steer_gain", 6.5)
        self._angle_factor = rospy.get_param("/e2/rot_factor", 5.0)
        self._loop_rate = rospy.get_param("/e2/loop_rate", 15)
        # self._simulated_vehicle_length = rospy.get_param("~simulated_vehicle_length")

        self.last_vel = None
        self.stat_idle, self.stat_rot, self.stat_rot_counter, self.stat_stop = range(4)
        self.status = self.stat_idle
        self.queue = []
        self.remaining_angle = 0

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

        self.log(f"Spd gain: {self._speed_gain}")
        self.log(f"Steer gain: {self._steer_gain}")
        self.log(f"Angle factor: {self._angle_factor}")
        self.log(f"Loop rate: {self._loop_rate}")
        self.log("Initialized")

    def on_shutdown(self):
        self.status = self.stat_stop
        self.pub_command()

    def velocity_callback(self, msg):
        self.log(msg)
        if (not self.status in {self.stat_stop, self.stat_idle}) and (self.last_vel):
            dt = (msg.header.stamp - self.last_vel.header.stamp).to_sec()
            # delta_x = self.last_vel.v * dt
            delta_omega = self.last_vel.omega * dt
            self.remaining_angle -= np.abs(delta_omega)
            print(delta_omega, self.remaining_angle)
            if self.remaining_angle < 0:
                self.state_pop()
        self.last_vel = msg

    def pub_command(self, event=None):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.get_rostime()
        car_cmd_msg.omega = self._steer_gain
        if self.status in {self.stat_stop, self.stat_idle}:
            car_cmd_msg.omega = 0
        if self.status == self.stat_rot:
            car_cmd_msg.omega = -car_cmd_msg.omega
        self.pub_car_cmd.publish(car_cmd_msg)

    def state_pop(self):
        print('pop')
        if self.queue:
            self.status = self.stat_idle
            self.pub_command()
            rospy.sleep(0.5)
            self.last_vel=None
            self.status, self.remaining_angle = self.queue.pop()
            self.remaining_angle *= self._angle_factor
            self.pub_command()
            print(self.status, self.remaining_angle)
        else:
            self.status = self.stat_stop
            print(self.status, self.remaining_angle)
            self.pub_command()
            rospy.sleep(1.)
            rospy.signal_shutdown("done")

    def run(self):
        # r = rospy.Rate(self._loop_rate)
        self.queue.append((self.stat_rot, np.pi/2))
        self.state_pop()
        # while not rospy.is_shutdown():
            # self.pub_command()
            # pass
            # r.sleep()


if __name__ == '__main__':
    node = TaskRotationNode(node_name='task_rotation_node')
    # Keep it spinning to keep the node alive
    rospy.loginfo("task_rotation_node is up and running...")
    rospy.Timer(rospy.Duration(0.05), node.pub_command)
    node.run()
    rospy.spin()