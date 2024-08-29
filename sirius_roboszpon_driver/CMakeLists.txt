cmake_minimum_required(VERSION 3.0.2)
project(sirius_roboszpon_driver)

find_package(
  catkin REQUIRED
  COMPONENTS can_msgs
             rospy
             message_generation
             sensor_msgs
             std_msgs)

add_message_files(FILES RoboszponStatus.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(INCLUDE_DIRS CATKIN_DEPENDS rospy)

include_directories(${catkin_INCLUDE_DIRS})
