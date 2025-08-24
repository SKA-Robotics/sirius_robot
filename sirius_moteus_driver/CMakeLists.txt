cmake_minimum_required(VERSION 3.0.2)
project(sirius_moteus_driver)

find_package(
  catkin REQUIRED
  COMPONENTS can_msgs
             rospy
             sensor_msgs
             std_msgs)

catkin_package(INCLUDE_DIRS CATKIN_DEPENDS rospy)

include_directories(${catkin_INCLUDE_DIRS})
