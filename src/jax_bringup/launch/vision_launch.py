from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='jax_eye',
            parameters=[{
                'width': 320,
                'height': 240,
                'camera_conf': '/usr/local/share/libcamera/ipa/rpi/vc4/imx708_wide.json',
            }]
        )
    ])
