import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # 1. Define exact paths based on your jax_locomotion structure
    jax_locomotion = get_package_share_directory("jax_locomotion")
    jax_description = get_package_share_directory("jax_description")

    robot_xacro = os.path.join(jax_description, "urdf", "jax_robot.xacro")
    
    # Matching your 'jacked up' names exactly
    gait_config = os.path.join(jax_locomotion, "config", "gait", "gait.yaml")
    links_config = os.path.join(jax_locomotion, "config", "links", "links.yaml")
    joints_config = os.path.join(jax_locomotion, "config", "joints", "joints.yaml")

    # 2. CHAMP Controller Node (The Math Engine)
    # We set gazebo to False and hardware_connected to True
    quadruped_controller = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        name="quadruped_controller",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"gazebo": False},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"joint_controller_topic": "/jax/walk_joint_trajectory"},
            {"urdf": Command(["xacro ", robot_xacro])},
            joints_config,
            links_config,
            gait_config,
            {"hardware_connected": True},
        ],
        # Maps CHAMP's velocity listener to your manager's output
        remappings=[("/cmd_vel/smooth", "/jax/cmd_vel_walk")],
    )

    # 3. Robot State Publisher (Required for URDF math)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"robot_description": Command(["xacro ", robot_xacro])},
        ],
    )

    return LaunchDescription([
        quadruped_controller,
        robot_state_publisher
    ])