cmake_minimum_required(VERSION 3.8)
project(jax_locomotion)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/jax_imu_cmdvel_stabilizer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()