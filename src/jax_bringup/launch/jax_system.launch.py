import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Hardware Bridge (Start immediately)
    serial_bridge = Node(
        package='jax_bringup',
        executable='jax_serial_bridge.py',
        name='jax_serial_bridge',
        output='screen'
    )

   # 2. Modern Camera Driver (camera_ros)
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='jax_vision',
        parameters=[{
            'width': 320,
            'height': 240,
        }],
        # This forces the node to publish to the root /image_raw
        remappings=[
            ('~/image_raw', '/image_raw'),
            ('~/image_raw/compressed', '/image_raw/compressed')
        ],
        output='screen'
    )

    # 3. Web Video Server (Start immediately)
    video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{'port': 8080}],
        output='screen'
    )

    # 4. Behavior Logic (Start immediately)
    behavior_node = Node(
        package='jax_behaviors',
        executable='jax_behavior_node.py',
        name='jax_behavior_node',
        output='screen'
    )

    # 5. Mode Manager (DELAYED by 5 seconds)
    # This ensures the bridge is ready to catch the first 'STAND' command
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

    return LaunchDescription([
        serial_bridge,
        camera_node,
        video_server,
        behavior_node,
        delayed_mode_manager
    ])