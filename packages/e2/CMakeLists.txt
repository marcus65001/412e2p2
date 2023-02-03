cmake_minimum_required(VERSION 2.8.3)
project(e2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  LEDSet.srv
)

catkin_package()