cmake_minimum_required(VERSION 3.0.2)
project(autonomy)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(FILES Marker.msg MarkerArray.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
