cmake_minimum_required(VERSION 3.8)
project(jax_behavior)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Install the Python scripts as executables
# This puts them in the install/lib folder so 'ros2 run' can find them
install(PROGRAMS
  scripts/mode_manager.py
  scripts/poses.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()