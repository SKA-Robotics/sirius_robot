cmake_minimum_required(VERSION 3.8)
project(canbus_modules)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(can_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PowerStatus.msg"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  scripts/canbus_interface.py
  scripts/gripper.py
  scripts/lamps.py
  scripts/power_measurement.py
  scripts/scales_erc.py
  scripts/science.py
  scripts/science_sensor.py
  scripts/science_teleop.py
  scripts/science_urc.py
  scripts/webcam_gui.py
  scripts/webcam_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  scripts/camera_interface.ui
  DESTINATION share/${PROJECT_NAME}/ui
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()