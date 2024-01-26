cmake_minimum_required(VERSION 3.16.3...3.28.1)
project(sirius_depthai_driver)

find_package(
  catkin REQUIRED
  COMPONENTS camera_info_manager
             depthai_bridge
             depthai_ros_msgs
             roscpp
             sensor_msgs)
find_package(depthai CONFIG REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  camera_info_manager
  depthai_bridge
  depthai_ros_msgs
  roscpp
  sensor_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/sirius_depthai_driver.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES} depthai::core)
install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
