import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node


def load_motion_limits(motion_config_path):
    with open(motion_config_path, 'r') as f:
        data = yaml.safe_load(f)

    params = data["/**"]["ros__parameters"]

    max_vx = float(params["max_vx"])
    max_vy = float(params["max_vy"])
    max_wz = float(params["max_wz"])

    return max_vx, max_vy, max_wz


def generate_launch_description():

    jax_locomotion = get_package_share_directory("jax_locomotion")
    jax_description = get_package_share_directory("jax_description")

    robot_xacro = os.path.join(
        jax_description,
        "urdf",
        "jax_robot.xacro"
    )

    ros_control_config = os.path.join(
        jax_locomotion,
        "config",
        "ros_control",
        "ros_control.yaml"
    )

    joints_config = os.path.join(
        jax_locomotion,
        "config",
        "joints",
        "joints.yaml"
    )

    motion_config = os.path.join(
        jax_locomotion,
        "config",
        "motion",
        "motion.yaml"
    )

    gait_config = os.path.join(
        jax_locomotion,
        "config",
        "gait",
        "gait.yaml"
    )

    links_config = os.path.join(
        jax_locomotion,
        "config",
        "links",
        "links.yaml"
    )

    world = os.path.join(
        jax_description,
        "worlds",
        "default.sdf"
    )

    max_vx, max_vy, max_wz = load_motion_limits(motion_config)

    robot_description = {
        "robot_description": Command(
            ["xacro ", robot_xacro, " robot_controllers:=", ros_control_config]
        )
    }

    jax_display =Node(
        package='jax_bringup',
        executable='jax_display_node.py',
        name='jax_display_node',
        output='screen',
        parameters=[{
            'mode_topic': '/jax_mode',
            'imu_topic': '/imu/data',
            'sim': True,
            'robot_name': 'JAX',
            'boot_duration': 2.5,
            'mode_flash_duration': 1.2,
        }]
    )



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
        output="screen",
        parameters=[
            {"use_sim_time": True},
            robot_description,
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "jax",
            "-topic", "robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "0.35",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ],
    )

    jax_behavior_node = Node(
        package="jax_behaviors",
        executable="jax_behavior_node.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    imu_stabilizer = Node(
        package="jax_locomotion",
        executable="jax_imu_cmdvel_stabilizer.py",
        output="screen",
        parameters=[
            {"kp_pitch": 0.7},
            {"kp_roll": 0.6},
            {"deadband": 0.02},
            {"max_correction": 0.20},
            {"alpha": 0.2},
            {"enabled": True},
            {"only_when_moving": True},
            {"motion_threshold": 0.02},
        ],
    )
    
    jax_mode_manager = Node(
        package="jax_behaviors",
        executable="jax_mode_manager.py",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"startup_mode": "stand"},
            {"transition_duration": 1.8},
        ],
    )

    jax_velocity_smoother = Node(
        package="jax_teleop",
        executable="jax_velocity_smoother.py",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            motion_config,
        ],
    )

   

    # Add the Remapper Node
    joint_remapper_node = Node(
        package='jax_bringup',
        executable='joint_remapper.py',  # Ensure this matches the filename in your scripts folder
        name='joint_remapper',
        output='screen',
        parameters=[{'use_sim_time': True}] # Set to False for the physical Pi
    )

    quadruped_controller = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"gazebo": True},
            {"publish_joint_states": False},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            # THIS IS THE RAW TOPIC THE REMAPPER LISTENS TO
            {"joint_controller_topic": "/jax/walk_joint_trajectory"}, 
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
        # Ensure only the cmd_vel is remapped here
        remappings=[("/cmd_vel/smooth", "/jax/cmd_vel_walk")],
    )

    joint_state_spawner = TimerAction(
        period=5.0,
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
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_group_effort_controller"],
                output="screen",
            )
        ],
    )




    return LaunchDescription(
        [
            jax_display,
            gazebo,
            robot_state_publisher,
            spawn_robot,
            bridge,
            jax_behavior_node,
            jax_mode_manager,
            jax_velocity_smoother,
            quadruped_controller,
            imu_stabilizer,
            joint_state_spawner,
            joint_remapper_node,
            trajectory_spawner,
        ]
    )