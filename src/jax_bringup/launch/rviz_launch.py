from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg = FindPackageShare("jax_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    enable_compensator = LaunchConfiguration("enable_compensator")

    rviz_config = PathJoinSubstitution(
        [sim_pkg, "rviz", "rviz.rviz"]
    )

    # This launch is intentionally GUI-only. The robot/RPi bringup owns
    # robot_state_publisher, linkage enforcement, controllers, and hardware IO.
    # We always publish sliders to /joint_states_raw so the robot-side envelope
    # node can clamp and republish /joint_states.
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[('/joint_states', '/joint_states_raw')],
        condition=IfCondition(gui),
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
        DeclareLaunchArgument("enable_compensator", default_value="true"),

        joint_state_publisher_gui,
        rviz2,
    ])