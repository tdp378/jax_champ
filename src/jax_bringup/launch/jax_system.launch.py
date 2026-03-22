import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # 1. Hardware Bridge
    serial_bridge = Node(
        package='jax_bringup',
        executable='jax_serial_bridge.py',
        name='jax_serial_bridge',
        output='screen'
    )

    # 2. Modern Camera Driver
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='jax_vision',
        parameters=[{'width': 320, 'height': 240}],
        remappings=[
            ('~/image_raw', '/image_raw'),
            ('~/image_raw/compressed', '/image_raw/compressed')
        ],
        output='screen'
    )

    # 3. Web Video Server
    video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{'port': 8080}],
        output='screen'
    )

    # 4. Behavior Logic (Static Poses)
    behavior_node = Node(
        package='jax_behaviors',
        executable='jax_behavior_node.py',
        name='jax_behavior_node',
        output='screen'
    )

    # 5. Velocity Smoother (The "Safety Buffer")
    # This takes raw 'cmd_vel' and outputs 'cmd_vel/smoothed'
    velocity_smoother = Node(
        package='jax_teleop',
        executable='jax_velocity_smoother.py',
        name='jax_velocity_smoother',
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/cmd_vel_smoothed', '/cmd_vel_smoothed') 
        ],
        output='screen'
    )

    # 6. IMU Stabilizer (Body Leveling)
    imu_stabilizer = Node(
        package='jax_locomotion',
        executable='jax_imu_cmdvel_stabilizer.py',
        name='jax_imu_cmdvel_stabilizer',
        output='screen'
    )

    # 7. Joint Remapper (Parallel Linkage Correction)
    joint_remapper = Node(
        package='jax_bringup',
        executable='joint_remapper.py',
        name='joint_remapper',
        output='screen'
    )

    # 8. Mode Manager (DELAYED)
    delayed_mode_manager = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='jax_behaviors',
                executable='jax_mode_manager.py',
                name='jax_mode_manager',
                parameters=[{'startup_mode': 'lay'}],
                output='screen'
            )
        ]
    )

    # 9. Communication & Teleop
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            ])
        ),
        launch_arguments={'port': '9090'}.items()
    )

    app_controller_node = Node(
        package='jax_teleop',
        executable='jax_app_controller.py',
        name='jax_app_controller',
        output='screen'
    )

    return LaunchDescription([
        serial_bridge,
        camera_node,
        video_server,
        behavior_node,
        velocity_smoother,
        imu_stabilizer,
        joint_remapper,
        delayed_mode_manager,
        app_controller_node,
        rosbridge_launch
    ])