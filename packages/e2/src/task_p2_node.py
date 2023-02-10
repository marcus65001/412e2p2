#!/usr/bin/env python3
import math

import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
# from duckietown_msgs.srv import ChangePattern, ChangePatternResponse
from e2.srv import LEDSet,LEDSetResponse
from std_msgs.msg import String

from std_msgs.msg import Header, Float32, Int32
import rosbag
from enum import Enum, auto


class TaskP2Node(DTROS):

    class State(Enum):
        IDLE=auto()
        WAIT=auto()
        ROT=auto()
        FW=auto()
        ROTC=auto()
        CIR=auto()
        LED=auto()

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(TaskP2Node, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        # self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)
        # self._speed_gain = rospy.get_param("~speed_gain")
        # self._steer_gain = rospy.get_param("~steer_gain")
        self._speed_gain = rospy.get_param("/e2/speed_gain", 0.41)
        self._steer_gain = rospy.get_param("/e2/steer_gain", 6.5)
        self._factor={
            self.State.ROT: rospy.get_param("/e2/steer_factor", 1.0),
            self.State.ROTC: rospy.get_param("/e2/steer_factor", 1.0),
            self.State.WAIT: rospy.get_param("/e2/wait_factor", 5.0),
            self.State.FW: rospy.get_param("/e2/speed_factor", 1.25),
            self.State.CIR: rospy.get_param("/e2/circle_factor", 1),
            self.State.LED: 0
        }
        self.log(self._factor)
        # self._simulated_vehicle_length = rospy.get_param("~simulated_vehicle_length")

        self.last_vel = None
        # self.stat_idle, self.stat_fwd, self.stat_bwd, self.stat_stop = range(4)

        self.status = self.State.IDLE
        self.queue = []
        self.remaining = 0
        self.wait_start = None
        self.LEDlist = ["WHITE", "BLUE"]

        # Subscribing to the wheel encoders
        self.sub_velocity = rospy.Subscriber(
            "~velocity", Twist2DStamped, self.velocity_callback, queue_size=1
        )

        # Publishers
        self.pub_car_cmd = rospy.Publisher(
            "~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        # Service Proxy
        self.changePattern = rospy.ServiceProxy(
            "~set_pattern", LEDSet
        )

        self.log("Initialized")

    def on_shutdown(self):
        self.status = self.State.IDLE
        self.pub_command()

    def velocity_callback(self, msg):
        if (not self.status in {self.State.IDLE,self.State.WAIT}) and (self.last_vel):
            dt = (msg.header.stamp - self.last_vel.header.stamp).to_sec()
            if self.status in {self.State.ROTC, self.State.ROT}:
                delta_omega = self.last_vel.omega * dt
                self.remaining -= abs(delta_omega)
                print(delta_omega, self.remaining)
            if self.status == self.State.FW:
                delta_x = self.last_vel.v * dt
                self.remaining -= delta_x
                print(delta_x, self.remaining)
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

    def pub_command(self, event=None):
        def wait():
            if not self.wait_start:
                self.wait_start = rospy.get_time()
                print("wait start", self.wait_start)
            else:
                if (rospy.get_time()-self.wait_start)>self.remaining:
                    print("wait end")
                    self.state_pop()
                    self.wait_start = None
            self.carcmd(0,0)

        def rot():
            self.carcmd(0,-1)

        def rotc():
            self.carcmd(0,1)

        def fwd():
            self.carcmd(1,0)

        def cir():
            self.carcmd(1,self._factor[self.State.CIR])

        def idle():
            self.carcmd(0,0)

        def LED():
            try:
                msg=String()
                msg.data=self.LEDlist.pop()
                self.changePattern(msg)
            except Exception as e:
                self.log("LED pattern change failed")
                self.log(e)
            self.state_pop()

        switch={
            self.State.IDLE: idle,
            self.State.ROT: rot,
            self.State.ROTC: rotc,
            self.State.WAIT: wait,
            self.State.FW: fwd,
            self.State.CIR: cir,
            self.State.LED: LED
        }

        switch[self.status]()


    def state_pop(self):
        print('pop')
        if self.queue:
            self.last_vel = None
            self.status = self.queue.pop()
            self.remaining = self._factor[self.status]
            print(self.status, self.remaining)
        else:
            self.status = self.State.IDLE
            rospy.signal_shutdown("done")

    def run(self):
        r = rospy.Rate(5)
        # self.queue=[self.State.WAIT,
        #             self.State.ROT,
        #             self.State.FW,
        #             self.State.ROTC,
        #             self.State.FW,
        #             self.State.ROTC,
        #             self.State.FW,
        #             self.State.WAIT,
        #             self.State.ROTC,
        #             self.State.FW,
        #             self.State.WAIT]
        self.queue = [self.State.ROT,self.State.CIR, self.State.WAIT,
                      self.status.LED]
        self.state_pop()


if __name__ == '__main__':
    node = TaskP2Node(node_name='task_p2_node')
    # Keep it spinning to keep the node alive

    rospy.loginfo("task_p2_node is up and running...")
    rospy.Timer(rospy.Duration(0.02), node.pub_command)
    node.run()
    rospy.spin()