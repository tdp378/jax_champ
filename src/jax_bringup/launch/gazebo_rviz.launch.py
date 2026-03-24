"""
Connect Rviz with Gazebo to visualize robot movements.
Usage: ros2 launch jax_bringup gazebo_rviz.launch.py

Note: Start Gazebo separately first, then run this to connect Rviz.
Example:
  Terminal 1: gazebo ~/worlds/my_world.sdf
  Terminal 2: ros2 launch jax_bringup gazebo_rviz.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package paths
    jax_description = get_package_share_directory("jax_description")
    jax_bringup = get_package_share_directory("jax_bringup")
    jax_locomotion = get_package_share_directory("jax_locomotion")

    # URDF and config paths
    robot_xacro = os.path.join(jax_description, "urdf", "jax_robot.xacro")
    rviz_config = os.path.join(jax_bringup, "rviz", "rviz.rviz")
    
    gait_config = os.path.join(jax_locomotion, "config", "gait", "gait.yaml")
    links_config = os.path.join(jax_locomotion, "config", "links", "links.yaml")
    joints_config = os.path.join(jax_locomotion, "config", "joints", "joints.yaml")

    # Launch arguments
    use_gui = LaunchConfiguration("gui", default="false")
    show_rviz = LaunchConfiguration("rviz", default="true")

    # Robot description
    robot_description = {
        "robot_description": Command([
            "xacro ",
            robot_xacro,
        ])
    }

    # ==================== NODES ====================

    # 1. Robot State Publisher 
    # This READS /joint_states from Gazebo and PUBLISHES /tf transforms for Rviz
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": False},  # Set to True if Gazebo publishes /clock
        ],
    )

    # 2. Joint State Publisher GUI (optional - for manual testing)
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(use_gui),
    )

    # 3. Rviz2 - VISUALIZES the /tf transforms
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(show_rviz),
    )

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument("gui", default_value="false",
                            description="Enable joint state publisher GUI for manual testing"),
        DeclareLaunchArgument("rviz", default_value="true",
                            description="Enable Rviz2 visualization"),

        # Launch nodes
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])
