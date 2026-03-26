# Jax Champ Workspace

ROS 2 Jazzy workspace for the Jax quadruped, built on CHAMP-style locomotion with a clean split between:

- Hardware interfaces and robot IO
- Locomotion and leg safety logic
- Bringup and launch orchestration
- Teleop and operator-facing controls

This README is a current snapshot of what is in this repo today.

## Current Repository Layout

Top-level workspace:

- src/: all source packages
- build/, install/, log/: colcon outputs

Main packages in src/:

- jax_bringup: launch files, RViz config, high-level orchestration
- jax_description: robot model and description assets
- jax_hardware: serial bridge, display node, hardware calibration
- jax_locomotion: locomotion support nodes, linkage envelope safety, tuning config
- jax_teleop: keyboard/app control and velocity smoothing
- champ_engine: CHAMP engine package used by locomotion pipeline
- camera_ros, sounds: supporting packages/assets

## What Is Running Where

### jax_bringup

Launch entry points:

- robot_launch.py: physical robot bringup
- sim_launch.py: simulation bringup
- rviz_launch.py: visualization and optional compensator path

Notes:

- jax_bringup is now launch/config focused (no core runtime scripts)
- It wires together nodes from jax_hardware and jax_locomotion

### jax_hardware

Scripts:

- jax_serial_bridge.py: talks to hardware/servo layer
- jax_display_node.py: display/status output node

Config:

- config/joint_calibration.yaml: joint offset calibration used by hardware bridge

### jax_locomotion

Scripts:

- jax_linkage_envelope.py: leg linkage envelope enforcement and safe clamping
- jax_imu_leg_height_stabilizer.py: IMU-assisted leg height stabilization
- champ_leg_height_wrapper.py: wrapper around CHAMP leg height behavior
- jax_joint_state_to_trajectory.py: conversion/bridge utility for joint interfaces

Config:

- config/gait/: gait tuning
- config/motion/: motion limits and behavior
- config/joints/, config/links/: robot kinematic configuration
- config/imu/: IMU-related tuning
- config/ros_control/: ros2_control integration parameters

### jax_teleop

Scripts:

- jax_keyboard_node.py: keyboard teleop input
- jax_velocity_smoother.py: command smoothing
- jax_app_controller.py: app/controller-side command bridge

## Control and Safety Flow

At a high level:

1. Teleop/app inputs produce motion commands.
2. Locomotion stack generates joint trajectories.
3. jax_linkage_envelope.py enforces safe thigh-calf envelope constraints.
4. Hardware bridge publishes final commands to the robot.
5. Joint states are republished for TF/RViz and downstream consumers.

The envelope node is the primary protection layer against linkage overtravel and binding in thigh/calf combinations.

## Build

From workspace root:

```bash
colcon build
source install/setup.bash
```

Build only key packages while iterating:

```bash
colcon build --packages-select jax_locomotion jax_hardware jax_bringup
source install/setup.bash
```

## Launch

Simulation:

```bash
ros2 launch jax_bringup sim_launch.py
```

Robot bringup:

```bash
ros2 launch jax_bringup robot_launch.py
```

RViz and GUI tooling:

```bash
ros2 launch jax_bringup rviz_launch.py gui:=true rviz:=true enable_compensator:=true use_sim_time:=false
```

If your shell aliases are configured, these shortcuts may be available:

- rviz
- robot

## Useful Day-to-Day Workflow

1. Edit code in jax_locomotion or jax_hardware.
2. Rebuild only changed packages.
3. Source install/setup.bash.
4. Launch rviz_launch.py (or sim/robot launch) and validate behavior.

## Current Status Snapshot

- Linkage safety logic has been consolidated into jax_linkage_envelope.py.
- Legacy remapper/compensator scripts have been removed.
- Hardware-specific scripts and calibration live in jax_hardware.
- jax_bringup remains the central launch orchestration package.

## Next README Improvements (Optional)

If you want, next pass can add:

- Full topic map (publishers/subscribers per node)
- Parameter reference tables for each script
- Hardware wiring and startup checklist
- Troubleshooting guide for launch/runtime issues
