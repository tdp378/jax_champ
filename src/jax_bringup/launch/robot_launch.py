import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
    gui_control = LaunchConfiguration("gui_control")
    local_gui = LaunchConfiguration("local_gui")

    jax_description = get_package_share_directory("jax_description")
    jax_locomotion = get_package_share_directory("jax_locomotion")
    jax_bringup = get_package_share_directory("jax_bringup")
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
    joint_calibration_config = os.path.join(jax_hardware, "config", "joint_calibration.yaml")
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
        condition=UnlessCondition(gui_control),
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
    )

    leg_safety = Node(
        package="jax_locomotion",
        executable="jax_simple_calf_follow.py",
        name="jax_simple_calf_follow_node",
        output="screen",
        parameters=[simple_calf_follow_config],
        remappings=[
            ('/joint_group_effort_controller/joint_trajectory',
             '/jax/joint_commands/linkage_corrected'),
        ],
    )

    serial_bridge = Node(
        package="jax_hardware",
        executable="jax_serial_bridge.py",
        name="jax_serial_bridge",
        output="screen",
        parameters=[joint_calibration_config, {"serial_port": serial_port}],
        remappings=[
            ('/joint_group_effort_controller/joint_trajectory',
             '/jax/joint_commands/linkage_corrected'),
        ],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(local_gui),
        remappings=[('/joint_states', '/joint_states_raw')],
    )

    joint_state_to_trajectory = Node(
        package="jax_locomotion",
        executable="jax_joint_state_to_trajectory.py",
        name="jax_joint_state_to_trajectory",
        output="screen",
        condition=IfCondition(gui_control),
        parameters=[{"output_topic": "/jax/walk_joint_trajectory_raw"}],
    )

    velocity_smoother = Node(
        package="jax_teleop",
        executable="jax_velocity_smoother.py",
        name="jax_velocity_smoother",
        output="screen",
        condition=UnlessCondition(gui_control),
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
        DeclareLaunchArgument("gui_control", default_value="false"),
        DeclareLaunchArgument("local_gui", default_value="false"),
        robot_state_publisher,
        quadruped_controller,
        leg_safety,
        joint_state_publisher_gui,
        joint_state_to_trajectory,
        serial_bridge,
        velocity_smoother,
        rviz,
    ])
