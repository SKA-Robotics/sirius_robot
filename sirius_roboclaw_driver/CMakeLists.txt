cmake_minimum_required(VERSION 3.0.2)
project(sirius_roboclaw_driver)

find_package(
  catkin REQUIRED
  COMPONENTS combined_robot_hw
             hardware_interface
             realtime_tools
             roscpp
             rospy
             sirius_msgs)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  combined_robot_hw
  hardware_interface
  realtime_tools
  roscpp
  rospy
  sirius_msgs)

include_directories(include/ ${catkin_INCLUDE_DIRS})

add_library(roboclaw_hardware_interfaces src/control.cpp)
target_link_libraries(roboclaw_hardware_interfaces ${catkin_LIBRARIES})

catkin_install_python(
  PROGRAMS
  scripts/driver.py
  scripts/all.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES roboclaw_hw_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
