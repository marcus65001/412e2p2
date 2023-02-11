#!/usr/bin/env python3
import math

import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
# from duckietown_msgs.srv import ChangePattern, ChangePatternResponse
from e2.srv import LEDSet, LEDSetResponse
from std_msgs.msg import String

from std_msgs.msg import Header, Float32, Int32
import rosbag
from enum import Enum, auto


class TaskP2Node(DTROS):
    class State(Enum):
        IDLE = auto()
        WAIT = auto()
        ROT = auto()
        FW = auto()
        ROTC = auto()
        CIR = auto()
        CIRP = auto()
        LED = auto()

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(TaskP2Node, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        # self._speed_gain = rospy.get_param("~speed_gain")
        # self._steer_gain = rospy.get_param("~steer_gain")
        # self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 0.0318)
        self._radius = 0.207345  # r=33mm
        self._baseline = 0.102  # l=102mm
        self._resolution = 135
        self._angle_to_tick = 135/(2*np.pi)


        self._speed_gain = rospy.get_param("/e2/speed_gain", 0.41)
        self._steer_gain = rospy.get_param("/e2/steer_gain", 6.5)
        self._circle_ratio = rospy.get_param("/e2/circle_ratio", 0.5)
        self._rot_factor = (np.pi*self._baseline/(2*self._radius)) * self._angle_to_tick
        self._rot_factor *= rospy.get_param("/e2/rot_factor", 1.0)
        self._fwd_factor = (1.25/self._radius)*self._resolution
        self._fwd_factor *= rospy.get_param("/e2/fwd_factor", 1.0)

        self.log(self._speed_gain)
        self.log(self._steer_gain)

        self.log(self._rot_factor)
        self.log(self._fwd_factor)

        self._factor = {
            self.State.ROT: (self._rot_factor, self._rot_factor),
            self.State.ROTC: (self._rot_factor, self._rot_factor),
            self.State.WAIT: (rospy.get_param("/e2/wait_factor", 5.0),0),
            self.State.FW: (self._fwd_factor, self._fwd_factor),
            self.State.CIR: (self._fwd_factor, self._fwd_factor),
            self.State.CIRP: (0.5*self._fwd_factor,0.5*self._fwd_factor),
            self.State.IDLE: (rospy.get_param("/e2/idle_factor", 0.6),0),
            self.State.LED: (0,0)
        }
        self._print_odometry = rospy.get_param("/e2/print_odometry", False)
        self.log(self._factor)
        # self._simulated_vehicle_length = rospy.get_param("~simulated_vehicle_length")

        self.last_vel = None
        # self.stat_idle, self.stat_fwd, self.stat_bwd, self.stat_stop = range(4)

        self.status = self.State.IDLE
        self.queue = []
        self.remaining_l = 0
        self.remaining_r = 0
        self.l_tick=0
        self.r_tick=0
        self.wait_start = None
        self.LEDlist = ["WHITE", "BLUE", "RED", "GREEN"]

        # Subscribing to the wheel encoders
        # self.sub_velocity = rospy.Subscriber(
        #     "~velocity", Twist2DStamped, self.velocity_callback, queue_size=1
        # )

        self.sub_encoder_ticks_left = rospy.Subscriber("~tick_l", WheelEncoderStamped, self.cb_enc_l)
        self.sub_encoder_ticks_right = rospy.Subscriber("~tick_r", WheelEncoderStamped, self.cb_enc_r)

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

    def cb_enc_l(self, msg):
        if self.l_tick and self.status not in {self.State.LED,self.State.IDLE,self.State.WAIT}:
            delta_l = msg.data - self.l_tick
            self.remaining_l -= -delta_l if self.status==self.State.ROTC else delta_l
            if min(self.remaining_l,self.remaining_r)<0:
                self.command_end()
        self.l_tick=msg.data

    def cb_enc_r(self,msg):
        if self.r_tick and self.status not in {self.State.LED,self.State.IDLE,self.State.WAIT}:
            delta_r=msg.data-self.r_tick
            self.remaining_r-=-delta_r if self.status==self.State.ROT else delta_r
            if min(self.remaining_l,self.remaining_r)<0:
                self.command_end()
            print(self.remaining_l,self.remaining_r)
        self.r_tick=msg.data



    # def velocity_callback(self, msg):
    #     if (not self.status in {self.State.IDLE, self.State.WAIT}) and (self.last_vel):
    #         dt = (msg.header.stamp - self.last_vel.header.stamp).to_sec()
    #         if self.status in {self.State.ROTC, self.State.ROT}:
    #             delta_omega = self.last_vel.omega * dt
    #             self.remaining -= abs(delta_omega)
    #             if self._print_odometry:
    #                 print(delta_omega, self.remaining)
    #         if self.status in {self.State.FW, self.State.CIR, self.State.CIRP}:
    #             delta_x = self.last_vel.v * dt
    #             self.remaining -= delta_x
    #             if self._print_odometry:
    #                 print(delta_x, self.remaining)
    #         if self.remaining < 0:
    #             self.command_end()
    #     self.last_vel = msg
    #     # print(self.status, self.last_vel)

    def carcmd(self, v, omega):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.get_rostime()
        car_cmd_msg.v = v * self._speed_gain
        car_cmd_msg.omega = omega * self._steer_gain
        self.pub_car_cmd.publish(car_cmd_msg)

    def pub_command(self, event=None):
        def wait():
            self.carcmd(0, 0)
            if not self.wait_start:
                self.wait_start = rospy.get_time()
                print("wait start", self.wait_start, self.remaining_l)
            else:
                if (rospy.get_time() - self.wait_start) > self.remaining_l:
                    print("wait end")
                    self.state_pop()
                    self.wait_start = None

        def rot():
            self.carcmd(0, -1)

        def rotc():
            self.carcmd(0, 1)

        def fwd():
            self.carcmd(1, 0)

        def cir():
            self.carcmd(1, -self._circle_ratio)

        def cir_pre():
            fwd()

        def idle():
            self.carcmd(0, 0)
            self.status = self.State.WAIT
            self.remaining_l,self.remaining_r = self._factor[self.State.IDLE]

        def LED():
            try:
                msg = String()
                msg.data = self.LEDlist.pop()
                self.changePattern(msg)
            except Exception as e:
                self.log("LED pattern change failed")
                self.log(e)
            self.command_end()

        switch = {
            self.State.IDLE: idle,
            self.State.ROT: rot,
            self.State.ROTC: rotc,
            self.State.WAIT: wait,
            self.State.FW: fwd,
            self.State.CIR: cir,
            self.State.CIRP: cir_pre,
            self.State.LED: LED
        }

        switch[self.status]()

    def command_end(self):
        print("command end")
        self.remaining_l=0
        self.remaining_r = 0
        self.status=self.State.IDLE
        self.remaining_l,self.remaining_r=self._factor[self.State.IDLE]

    def state_pop(self):
        print('pop')
        if self.queue:
            self.last_vel = None
            self.status = self.queue.pop()
            self.remaining_l,self.remaining_r = self._factor[self.status]
            print(self.status, self.remaining_l,self.remaining_r)
        else:
            self.status = self.State.IDLE
            rospy.signal_shutdown("done")

    def run(self):
        self.queue = [self.State.CIR,
                      self.State.CIRP,
                      self.State.WAIT,
                      self.State.LED,
                      self.State.ROTC,
                      self.State.ROTC,
                      self.State.FW,
                      self.State.ROTC,
                      self.State.WAIT,
                      self.State.LED,
                      self.State.FW,
                      self.State.ROTC,
                      self.State.FW,
                      self.State.ROTC,
                      self.State.FW,
                      self.State.ROT,
                      self.State.WAIT,
                      self.State.LED]
        # self.queue = [self.State.ROTC, self.State.FW, self.State.ROT, self.State.LED]
        self.state_pop()


if __name__ == '__main__':
    node = TaskP2Node(node_name='task_p2_node')
    # Keep it spinning to keep the node alive

    rospy.loginfo("task_p2_node is up and running...")
    rospy.Timer(rospy.Duration(0.005), node.pub_command)
    node.run()
    rospy.spin()
