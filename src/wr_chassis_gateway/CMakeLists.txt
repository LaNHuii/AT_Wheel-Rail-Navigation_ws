cmake_minimum_required(VERSION 3.8)
project(wr_chassis_gateway)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wr_interfaces REQUIRED)

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

add_executable(real_chassis_adapter_stub_node src/real_chassis_adapter_stub_node.cpp)
target_compile_features(real_chassis_adapter_stub_node PUBLIC cxx_std_17)
ament_target_dependencies(real_chassis_adapter_stub_node geometry_msgs rclcpp wr_interfaces)

install(TARGETS velocity_limiter cmd_vel_guard_node real_chassis_adapter_stub_node
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
endif()

ament_export_include_directories(include)
ament_export_libraries(velocity_limiter)
ament_package()
