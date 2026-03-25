cmake_minimum_required(VERSION 3.8)
project(jax_locomotion)

find_package(ament_cmake REQUIRED)
find_package(champ_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/jax_imu_leg_height_stabilizer.py
  scripts/champ_leg_height_wrapper.py
  scripts/jax_linkage_compensator.py
  scripts/jax_joint_state_to_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
