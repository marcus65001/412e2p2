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
        self.pose_robot=Pose2DStamped()
        self.pose_world=Pose2DStamped()

        self.rot_factor= 0.266
        self.fw_factor = 0.592


        self.bag=rosbag.Bag('/data/bags/task.bag','w')

        # Subscribers
        self.sub_velocity = rospy.Subscriber(
            "~velocity", Twist2DStamped, self.velocity_callback, queue_size=1
        )

        self.print_odometry = rospy.get_param("/e2/print_odometry", False)


        self.log("Initialized")

    def on_shutdown(self):
        self.bag.close()

    def velocity_callback(self, msg):
        if self.last_vel:
            dt = (msg.header.stamp - self.last_vel.header.stamp).to_sec()
            self.last_vel.v*=self.fw_factor
            delta_theta = self.last_vel.omega*self.rot_factor * dt
            # delta_x = self.last_vel.v * dt
            if np.abs(self.last_vel.omega) < 0.000001:
                # straight line
                delta_y = self.last_vel.v * dt
                delta_x = 0
            else:
                # arc of circle
                radius = self.last_vel.v / self.last_vel.omega
                delta_x = radius * (1.0 - np.cos(delta_theta))
                delta_y = radius * np.sin(delta_theta)

            # self.pose_robot.x+=delta_x
            # self.pose_robot.y+=delta_y
            self.pose_robot.theta+=delta_theta

            delta_xw = delta_x * np.cos(self.pose_robot.theta) - delta_y*np.sin(self.pose_robot.theta)
            delta_yw = delta_x * np.sin(self.pose_robot.theta) + delta_y*np.cos(self.pose_robot.theta)

            self.pose_world.x+=delta_xw
            self.pose_world.y+=delta_yw
            self.pose_world.theta+=delta_theta

            self.pose_world.header.stamp = rospy.Time.now()
            self.pose_robot.header.stamp = self.pose_world.header.stamp
            if self.print_odometry:
                self.log('world {}'.format(self.pose_world))
                self.log('robot {}'.format(self.pose_robot))

            try:
                self.bag.write("world_frame_pose",self.pose_world)
                self.bag.write("robot_frame_pose", self.pose_robot)
            except Exception as e:
                self.log("bag closed")

            # self.bag.write("robot_frame_pose", self.pose_robot)
            print(delta_x, delta_y, delta_theta)
            print(delta_xw, delta_yw, delta_theta)
        self.last_vel = msg


if __name__ == '__main__':
    node = OdometryNode(node_name='odometry_node')
    # Keep it spinning to keep the node alive

    rospy.loginfo("odometry_node is up and running...")
    rospy.spin()