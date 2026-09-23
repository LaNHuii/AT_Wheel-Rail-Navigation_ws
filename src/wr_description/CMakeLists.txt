cmake_minimum_required(VERSION 3.8)
project(wr_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_test(
    NAME test_robot_description
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_robot_description.py
  )
endif()

ament_package()
