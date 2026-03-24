import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node


def load_motion_limits(motion_config_path):
    with open(motion_config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    params = data["/**"]["ros__parameters"]
    return float(params["max_vx"]), float(params["max_vy"]), float(params["max_wz"])


def generate_launch_description():
    use_rviz = LaunchConfiguration("rviz")
    serial_port = LaunchConfiguration("serial_port")

    jax_description = get_package_share_directory("jax_description")
    jax_locomotion = get_package_share_directory("jax_locomotion")
    jax_bringup = get_package_share_directory("jax_bringup")

    robot_xacro = os.path.join(jax_description, "urdf", "jax_robot.xacro")
    joints_config = os.path.join(jax_locomotion, "config", "joints", "joints.yaml")
    links_config = os.path.join(jax_locomotion, "config", "links", "links.yaml")
    gait_config = os.path.join(jax_locomotion, "config", "gait", "gait.yaml")
    motion_config = os.path.join(jax_locomotion, "config", "motion", "motion.yaml")
    joint_calibration_config = os.path.join(jax_bringup, "config", "joint_calibration.yaml")
    rviz_config = os.path.join(jax_bringup, "rviz", "rviz.rviz")

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
            {"joint_controller_topic": "/joint_group_effort_controller/joint_trajectory"},
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
    )

    serial_bridge = Node(
        package="jax_bringup",
        executable="jax_serial_bridge.py",
        name="jax_serial_bridge",
        output="screen",
        parameters=[joint_calibration_config, {"serial_port": serial_port}],
    )

    velocity_smoother = Node(
        package="jax_teleop",
        executable="jax_velocity_smoother.py",
        name="jax_velocity_smoother",
        output="screen",
        parameters=[motion_config],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyAMA0"),
        robot_state_publisher,
        quadruped_controller,
        serial_bridge,
        velocity_smoother,
        rviz,
    ])
