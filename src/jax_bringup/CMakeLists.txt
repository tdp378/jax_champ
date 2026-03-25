cmake_minimum_required(VERSION 3.8)
project(jax_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/jax_display_node.py
  scripts/jax_serial_bridge.py
  scripts/joint_remapper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()