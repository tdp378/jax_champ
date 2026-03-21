Jax Quadruped (ROS 2 Jazzy + CHAMP)
Jax is a custom quadruped robot built on ROS 2 Jazzy, using the CHAMP locomotion stack, with support for simulation in Gazebo Harmonic and behavior-based control modes.
This repository contains everything needed to simulate, control, and extend Jax, including robot description, locomotion configs, behavior system, and bringup.
🚀 Features
🐕 Quadruped locomotion using CHAMP
🎮 Mode-based control (walk / stand / sit / lay)
🧠 Behavior + locomotion routing system
⚖️ IMU-based stabilization (cmd_vel correction)
🌍 Gazebo Harmonic simulation (GPU-ready)
🦴 Full URDF/Xacro robot description
🎛️ ROS2 control integration (ros2_control)
📺 Optional display node (status / boot UI)
📦 Package Overview
jax_description
Robot model, URDF/Xacro, and simulation assets.
Robot geometry and joints
Gazebo world
ros2_control integration
jax_locomotion
CHAMP configuration and motion tuning.
Gait parameters
Motion limits (velocity, rotation)
Joint and link configs
IMU stabilization node
jax_behaviors
Behavior system and mode control.
Stand / Sit / Lay poses
Mode switching logic
Smooth transitions between states
jax_bringup
Launch system for simulation and full stack startup.
Gazebo launch
Controller startup
Node orchestration
🧠 System Architecture
Jax separates control into two main paths:
1. Locomotion (CHAMP)
Handles walking
Consumes /cmd_vel
Outputs joint trajectories
2. Behaviors
Handles static poses (stand, sit, lay)
Publishes predefined joint trajectories
🔀 Mode Manager
The jax_mode_manager routes control:
Walk → CHAMP
Stand/Sit/Lay → Behavior system
Smooth transition before enabling walking
⚙️ Requirements
Ubuntu 24.04
ROS 2 Jazzy
Gazebo Harmonic (ros_gz_sim)
CHAMP (ROS2-compatible fork)
ros2_control
