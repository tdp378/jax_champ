import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_pkg = FindPackageShare("jax_description")
    sim_pkg = FindPackageShare("jax_bringup")

    jax_locomotion = get_package_share_directory("jax_locomotion")
    linkage_compensator_config = os.path.join(
        jax_locomotion, "config", "linkage_compensator.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")

    xacro_file = PathJoinSubstitution(
        [description_pkg, "urdf", "jax_robot.xacro"]
    )

    rviz_config = PathJoinSubstitution(
        [sim_pkg, "rviz", "rviz.rviz"]
    )

    robot_description = {
        "robot_description": Command([
            "xacro ",
            xacro_file,
        ])
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[('/joint_states', '/joint_states_raw')],
        condition=IfCondition(gui),
    )

    # Sits between the GUI sliders and robot_state_publisher.
    # Subscribes /joint_states_raw, applies calf compensation, publishes /joint_states
    # so RViz shows the geometrically-correct parallel-linkage calf motion.
    linkage_compensator = Node(
        package="jax_locomotion",
        executable="jax_linkage_compensator.py",
        name="jax_linkage_compensator",
        output="screen",
        parameters=[linkage_compensator_config, {"use_sim_time": use_sim_time}],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),

        robot_state_publisher,
        joint_state_publisher_gui,
        linkage_compensator,
        rviz2,
    ])