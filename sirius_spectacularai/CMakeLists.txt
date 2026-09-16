cmake_minimum_required(VERSION 3.8)
project(sirius_spectacularai)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  script/slam.py
  script/message_constructors.py
  script/transforms.py
  script/offset.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
