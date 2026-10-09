cmake_minimum_required(VERSION 3.8)
project(wr_chassis_gateway)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wr_interfaces REQUIRED)

add_library(can_protocol src/can_protocol.cpp)
target_compile_features(can_protocol PUBLIC cxx_std_17)
target_include_directories(can_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(socketcan_transport src/socketcan_transport.cpp)
target_compile_features(socketcan_transport PUBLIC cxx_std_17)
target_include_directories(socketcan_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(velocity_limiter src/velocity_limiter.cpp)
target_compile_features(velocity_limiter PUBLIC cxx_std_17)
target_include_directories(velocity_limiter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(cmd_vel_guard_node src/cmd_vel_guard_node.cpp)
target_compile_features(cmd_vel_guard_node PUBLIC cxx_std_17)
target_link_libraries(cmd_vel_guard_node velocity_limiter)
ament_target_dependencies(cmd_vel_guard_node geometry_msgs rclcpp std_msgs)

add_executable(cmd_vel_arbiter_node src/cmd_vel_arbiter_node.cpp)
target_compile_features(cmd_vel_arbiter_node PUBLIC cxx_std_17)
ament_target_dependencies(cmd_vel_arbiter_node geometry_msgs rclcpp std_msgs wr_interfaces)

add_executable(real_chassis_adapter_node src/real_chassis_adapter_stub_node.cpp)
target_compile_features(real_chassis_adapter_node PUBLIC cxx_std_17)
target_link_libraries(real_chassis_adapter_node can_protocol socketcan_transport)
ament_target_dependencies(
  real_chassis_adapter_node geometry_msgs rclcpp sensor_msgs wr_interfaces)

install(TARGETS can_protocol socketcan_transport velocity_limiter cmd_vel_guard_node cmd_vel_arbiter_node real_chassis_adapter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_velocity_limiter test/test_velocity_limiter.cpp)
  target_link_libraries(test_velocity_limiter velocity_limiter)
  ament_add_gtest(test_can_protocol test/test_can_protocol.cpp)
  target_link_libraries(test_can_protocol can_protocol)
endif()

ament_export_include_directories(include)
ament_export_libraries(can_protocol socketcan_transport velocity_limiter)
ament_package()
