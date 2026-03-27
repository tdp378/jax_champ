cmake_minimum_required(VERSION 3.8)
project(jax_hardware)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/jax_serial_bridge.py
  scripts/jax_display_node.py
  scripts/jax_wifi_status_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
