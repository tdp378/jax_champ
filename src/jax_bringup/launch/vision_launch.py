from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the foxglove launch file
    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('foxglove_bridge'), 
                         'launch', 'foxglove_bridge_launch.xml')
        )
    )

    return LaunchDescription([
        # Your existing camera node
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            parameters=[{
                'width': 320,
                'height': 240,
                'fps': 15.0,
                'camera_conf': '/usr/local/share/libcamera/ipa/rpi/vc4/imx708_wide.json',
            }]
        ),
        # Your existing web server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server'
        ),
        # THE NEW FOXGLOVE BRIDGE
        foxglove_bridge_launch
    ])
