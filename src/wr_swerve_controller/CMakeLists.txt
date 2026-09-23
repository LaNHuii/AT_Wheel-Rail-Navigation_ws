cmake_minimum_required(VERSION 3.8)
project(wr_swerve_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wr_interfaces REQUIRED)

add_library(swerve_kinematics src/swerve_kinematics.cpp)
target_compile_features(swerve_kinematics PUBLIC cxx_std_17)
target_include_directories(swerve_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(swerve_controller_node src/swerve_controller_node.cpp)
target_compile_features(swerve_controller_node PUBLIC cxx_std_17)
target_link_libraries(swerve_controller_node swerve_kinematics)
ament_target_dependencies(swerve_controller_node
  geometry_msgs rclcpp sensor_msgs std_msgs wr_interfaces
)

install(TARGETS swerve_kinematics swerve_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_swerve_kinematics test/test_swerve_kinematics.cpp)
  target_link_libraries(test_swerve_kinematics swerve_kinematics)
endif()

ament_export_include_directories(include)
ament_export_libraries(swerve_kinematics)
ament_package()
