cmake_minimum_required(VERSION 2.8.3)
project(e2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  duckietown_msgs
)

add_service_files(
  FILES
  LEDSet.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  duckietown_msgs
)

catkin_package()