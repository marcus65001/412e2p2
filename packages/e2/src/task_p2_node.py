#!/usr/bin/env python3
import math

import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32, Int32
import rosbag
from enum import Enum, auto


class TaskStraightNode(DTROS):

    class State(Enum):
        IDLE=auto()
        WAIT=auto()
        ROT=auto()
        FW=auto()
        ROTC=auto()

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
        # self._simulated_vehicle_length = rospy.get_param("~simulated_vehicle_length")

        self.last_vel = None
        # self.stat_idle, self.stat_fwd, self.stat_bwd, self.stat_stop = range(4)

        self.status = self.State.IDLE
        self.queue = []
        self.remaining = 0

        # Subscribing to the wheel encoders
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
            if self.status in {self.State.ROTC,self.State.ROT}:
                delta_omega = self.last_vel.omega * dt
                self.remaining -= delta_omega
            else:
                delta_x = self.last_vel.v * dt
                self.remaining -= delta_x
            print(delta_x, delta_omega, self.remaining)
            if self.remaining < 0:
                self.state_pop()
        self.last_vel = msg
        # print(self.status, self.last_vel)


    def carcmd(self,v,omega):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.get_rostime()
        car_cmd_msg.v = v*self._speed_gain
        car_cmd_msg.omega = omega*self._steer_gain
        self.pub_car_cmd.publish(car_cmd_msg)

    def pub_command(self):
        def wait():
            self.carcmd(0,0)
            self.state_pop()
            rospy.sleep(5.)
        def rot():
            self.carcmd(0,-1)

        def rotc():
            self.carcmd(0,1)

        def fwd():
            self.carcmd(1,0)

        def idle():
            self.carcmd(0,0)

        switch={
            self.State.IDLE: idle,
            self.State.ROT: rot,
            self.State.ROTC: rotc,
            self.State.WAIT: wait,
            self.State.FW: fwd,
        }


    def state_pop(self):
        print('pop')
        if self.queue:
            self.status = self.queue.pop()
            print(self.status, self.remaining)
        else:
            self.status = self.stat_stop
            rospy.signal_shutdown("done")

    def run(self):
        r = rospy.Rate(5)
        self.queue=[self.State.WAIT,
                    self.State.ROT,
                    self.State.FW,
                    self.State.ROTC,
                    self.State.FW,
                    self.State.ROTC,
                    self.State.FW,
                    self.State.WAIT,
                    self.State.ROTC,
                    self.State.FW,
                    self.State.WAIT]
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