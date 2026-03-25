import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node


def load_motion_limits(motion_config_path):
    with open(motion_config_path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    params = data["/**"]["ros__parameters"]
    return (
        float(params["max_vx"]),
        float(params["max_vy"]),
        float(params["max_wz"]),
    )


def generate_launch_description():
    use_rviz = LaunchConfiguration("rviz")
    gui_control = LaunchConfiguration("gui_control")

    jax_description = get_package_share_directory("jax_description")
    jax_locomotion = get_package_share_directory("jax_locomotion")
    jax_bringup = get_package_share_directory("jax_bringup")

    robot_xacro = os.path.join(jax_description, "urdf", "jax_robot.xacro")
    ros_control_config = os.path.join(
        jax_locomotion, "config", "ros_control", "ros_control.yaml"
    )
    joints_config = os.path.join(jax_locomotion, "config", "joints", "joints.yaml")
    links_config = os.path.join(jax_locomotion, "config", "links", "links.yaml")
    gait_config = os.path.join(jax_locomotion, "config", "gait", "gait.yaml")
    motion_config = os.path.join(jax_locomotion, "config", "motion", "motion.yaml")
    world = os.path.join(jax_description, "worlds", "default.sdf")
    rviz_config = os.path.join(jax_bringup, "rviz", "rviz.rviz")

    max_vx, max_vy, max_wz = load_motion_limits(motion_config)

    robot_description = {
        "robot_description": Command(
            ["xacro ", robot_xacro, " robot_controllers:=", ros_control_config]
        )
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": world + " -r"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            robot_description,
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_jax",
        output="screen",
        arguments=[
            "-name",
            "jax",
            "-topic",
            "robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.35",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ],
    )

    quadruped_controller = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        name="quadruped_controller",
        output="screen",
        condition=UnlessCondition(gui_control),
        parameters=[
            {"use_sim_time": True},
            {"gazebo": True},
            {"publish_joint_states": False},
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

    joint_remapper = Node(
        package="jax_bringup",
        executable="joint_remapper.py",
        name="joint_remapper",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(gui_control),
        parameters=[{"use_sim_time": True}],
        remappings=[('/joint_states', '/joint_states_raw')],
    )

    joint_state_to_trajectory = Node(
        package="jax_locomotion",
        executable="jax_joint_state_to_trajectory.py",
        name="jax_joint_state_to_trajectory",
        output="screen",
        condition=IfCondition(gui_control),
        parameters=[{"use_sim_time": True}],
        remappings=[
            ('/jax/joint_commands/linkage_corrected',
             '/joint_group_effort_controller/joint_trajectory'),
        ],
    )

    joint_state_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_states_controller"],
                output="screen",
            )
        ],
    )

    trajectory_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_group_effort_controller"],
                output="screen",
            )
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("gui_control", default_value="false"),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            bridge,
            quadruped_controller,
            joint_remapper,
            joint_state_publisher_gui,
            joint_state_to_trajectory,
            joint_state_spawner,
            trajectory_spawner,
            rviz,
        ]
    )