from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_pkg = FindPackageShare("jax_description")
    sim_pkg = FindPackageShare("jax_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    enable_compensator = LaunchConfiguration("enable_compensator")

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

    # When safety processing is enabled, publish sliders to /joint_states_raw and
    # let jax_linkage_envelope republish /joint_states.
    joint_state_publisher_gui_raw = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui_raw",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[('/joint_states', '/joint_states_raw')],
        condition=IfCondition(PythonExpression([
            "'", gui, "' == 'true' and '", enable_compensator, "' == 'true'"
        ])),
    )

    # When safety processing is disabled, publish directly to /joint_states so
    # robot_state_publisher still receives updates and TF is available.
    joint_state_publisher_gui_direct = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui_direct",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(PythonExpression([
            "'", gui, "' == 'true' and '", enable_compensator, "' != 'true'"
        ])),
    )

    # Sits between GUI sliders and robot_state_publisher when enabled.
    # Subscribes /joint_states_raw, clamps into safety envelope, publishes /joint_states.
    linkage_compensator = Node(
        package="jax_locomotion",
        executable="jax_linkage_envelope.py",
        name="jax_linkage_envelope_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(enable_compensator),
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

        robot_state_publisher,
        joint_state_publisher_gui_raw,
        joint_state_publisher_gui_direct,
        linkage_compensator,
        rviz2,
    ])