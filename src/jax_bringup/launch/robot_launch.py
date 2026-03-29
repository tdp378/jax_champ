import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node


def load_motion_limits(motion_config_path):
    with open(motion_config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    params = data["/**"]["ros__parameters"]
    return float(params["max_vx"]), float(params["max_vy"]), float(params["max_wz"])


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")

    jax_description = get_package_share_directory("jax_description")
    jax_locomotion = get_package_share_directory("jax_locomotion")
    jax_hardware = get_package_share_directory("jax_hardware")

    robot_xacro = os.path.join(jax_description, "urdf", "jax_robot.xacro")
    joints_config = os.path.join(jax_locomotion, "config", "joints", "joints.yaml")
    links_config = os.path.join(jax_locomotion, "config", "links", "links.yaml")
    gait_config = os.path.join(jax_locomotion, "config", "gait", "gait.yaml")
    motion_config = os.path.join(jax_locomotion, "config", "motion", "motion.yaml")
    simple_calf_follow_config = os.path.join(
        jax_locomotion,
        "config",
        "follow",
        "jax_simple_calf_follow.yaml",
    )
    joint_calibration_config = os.path.join(
        jax_hardware,
        "config",
        "joint_calibration.yaml",
    )
    display_config = os.path.join(jax_hardware, "config", "jax_display.yaml")

    max_vx, max_vy, max_wz = load_motion_limits(motion_config)

    robot_description = {
        "robot_description": Command(["xacro ", robot_xacro])
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    quadruped_controller = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        name="quadruped_controller",
        output="screen",
        parameters=[
            {"gazebo": False},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {"joint_controller_topic": "/jax/walk_joint_trajectory_raw"},
            {"urdf": Command(["xacro ", robot_xacro])},
            joints_config,
            links_config,
            gait_config,
            {"gait.max_linear_velocity_x": max_vx},
            {"gait.max_linear_velocity_y": max_vy},
            {"gait.max_angular_velocity_z": max_wz},
            {"hardware_connected": False},
            {"close_loop_odom": False},
        ],
        remappings=[
            ("/cmd_vel", "/cmd_vel/smooth"),
        ],
    )

    mode_manager = Node(
        package="jax_behavior",
        executable="mode_manager.py",
        name="jax_mode_manager",
        output="screen",
    )

    leg_safety = Node(
        package="jax_locomotion",
        executable="jax_simple_calf_follow.py",
        name="jax_simple_calf_follow_node",
        output="screen",
        parameters=[
            simple_calf_follow_config,
            {"input_trajectory_topic": "/jax/combined_joint_trajectory"},
            {"output_trajectory_topic": "/joint_group_effort_controller/joint_trajectory"},
        ],
        remappings=[
            (
                "/joint_group_effort_controller/joint_trajectory",
                "/jax/joint_commands/linkage_corrected",
            ),
        ],
    )

    serial_bridge = Node(
        package="jax_hardware",
        executable="jax_serial_bridge.py",
        name="jax_serial_bridge",
        output="screen",
        parameters=[joint_calibration_config, {"serial_port": serial_port}],
        remappings=[
            (
                "/joint_group_effort_controller/joint_trajectory",
                "/jax/joint_commands/linkage_corrected",
            ),
        ],
    )

    velocity_smoother = Node(
        package="jax_teleop",
        executable="jax_velocity_smoother.py",
        name="jax_velocity_smoother",
        output="screen",
        parameters=[motion_config],
    )

    display = Node(
        package="jax_hardware",
        executable="jax_display_node.py",
        name="jax_display_node",
        output="screen",
        parameters=[display_config],
    )

    wifi_status = Node(
        package="jax_hardware",
        executable="jax_wifi_status_publisher.py",
        name="jax_wifi_status_publisher",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyAMA0"),
        robot_state_publisher,
        quadruped_controller,
        mode_manager,
        leg_safety,
        serial_bridge,
        velocity_smoother,
        display,
        wifi_status,
    ])