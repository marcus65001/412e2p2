#!/usr/bin/env python3
import math

import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Pose2DStamped, Twist2DStamped
from std_msgs.msg import Header, Float32, Int32
import rosbag


class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        self.last_vel = None
        self.xw=0.
        self.yw=0.
        self.thetaw=0.
        self.pose_robot=Pose2DStamped()
        self.pose_robot.x = 0
        self.pose_robot.y = 0
        self.pose_robot.theta = 0

        self.bag=rosbag.Bag('/data/bags/task.bag')

        # Subscribers
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
        self.bag.close()

    def velocity_callback(self, msg):
        if self.last_vel:
            dt = (msg.header.stamp - self.last_vel.header.stamp).to_sec()
            delta_theta = self.last_vel.omega * dt
            # delta_x = self.last_vel.v * dt
            if np.abs(self.last_theta_dot) < 0.000001:
                # straight line
                delta_x = self.last_v * dt
                delta_y = 0
            else:
                # arc of circle
                radius = self.last_v / self.last_theta_dot
                delta_x = radius * np.sin(delta_theta)
                delta_y = radius * (1.0 - np.cos(delta_theta))
            self.xw+=delta_x
            self.yw+=delta_y
            self.thetaw+=delta_theta
            msg = Pose2DStamped()
            msg.header.stamp = rospy.Time.now()
            self.pose_robot.header.stamp = msg.header.stamp
            msg.x=self.xw
            msg.y=self.yw
            msg.theta=self.thetaw
            self.log(msg)

            try:
                self.bag.write("world_frame_pose", msg)
            except Exception as e:
                self.log("bag closed")

            # self.bag.write("robot_frame_pose", self.pose_robot)
            print(delta_x, delta_y, delta_theta)
        self.last_vel = msg


if __name__ == '__main__':
    node = OdometryNode(node_name='odometry_node')
    # Keep it spinning to keep the node alive

    rospy.loginfo("odometry_node is up and running...")
    rospy.spin()