cmake_minimum_required(VERSION 3.8)
project(jax_behaviors)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/jax_behavior_node.py
  scripts/jax_mode_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()