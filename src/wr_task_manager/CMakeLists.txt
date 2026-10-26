cmake_minimum_required(VERSION 3.8)
project(wr_task_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wr_interfaces REQUIRED)

add_executable(wr_task_manager_node src/task_manager_node.cpp)
target_compile_features(wr_task_manager_node PUBLIC cxx_std_17)
ament_target_dependencies(wr_task_manager_node
  ament_index_cpp behaviortree_cpp_v3 rclcpp rclcpp_action std_msgs wr_interfaces)

install(TARGETS wr_task_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees launch DESTINATION share/${PROJECT_NAME})
ament_package()
