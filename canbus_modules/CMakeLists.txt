cmake_minimum_required(VERSION 3.0.2)
project(canbus_modules)

find_package(
  catkin REQUIRED
  COMPONENTS geometry_msgs
             rospy
             sensor_msgs
             message_generation
             dynamic_reconfigure)

add_message_files(FILES PowerStatus.msg)

generate_messages(DEPENDENCIES)

generate_dynamic_reconfigure_options()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
