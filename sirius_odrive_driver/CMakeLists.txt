cmake_minimum_required(VERSION 3.8)
project(sirius_odrive_driver)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  scripts/driver.py
  scripts/relaxing_middleware.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
