cmake_minimum_required(VERSION 3.8)
project(wr_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SwerveModuleState.msg"
  "msg/ModuleState.msg"
  "msg/ChassisState.msg"
  "msg/ChassisControlCommand.msg"
  "msg/SteeringTaskState.msg"
  "msg/CommandAck.msg"
  "msg/ImuStatus.msg"
  "msg/TrackedPerson.msg"
  "msg/LocalSafetyState.msg"
  "msg/TraversabilityState.msg"
  "msg/SystemHealth.msg"
  "msg/PersonFollowStatus.msg"
  "msg/SwerveState.msg"
  "msg/RailEngagementState.msg"
  "msg/RailEntryEstimate.msg"
  "msg/RailClampCommand.msg"
  "msg/RailClampFeedback.msg"
  "srv/SetRailMode.srv"
  "srv/SetMotionSource.srv"
  "action/ExecuteRailTransition.action"
  "action/FollowPerson.action"
  DEPENDENCIES action_msgs geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
