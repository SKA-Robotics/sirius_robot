cmake_minimum_required(VERSION 3.8)
project(sirius_roboszpon_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(can_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoboszponStatus.msg"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  scripts/driver.py
  scripts/roboszpon_interface.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_dependencies(rosidl_default_runtime)
ament_package()