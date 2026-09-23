cmake_minimum_required(VERSION 3.8)
project(wr_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf worlds DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/gazebo_command_adapter.py
    scripts/verify_flat_ground.py
    scripts/verify_static_rail.py
    scripts/verify_rail_motion.py
    scripts/verify_rail_clamp.py
    scripts/verify_rail_entry.py
    scripts/verify_rail_entry_closed_loop.py
    scripts/verify_rail_entry_state_machine.py
    scripts/verify_rail_entry_phased.py
    scripts/verify_rail_exit.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
