cmake_minimum_required(VERSION 3.8)
project(autonomy)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Marker.msg"
  "msg/MarkerArray.msg"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  scripts/aruco_detector_node.py
  scripts/aruco_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
