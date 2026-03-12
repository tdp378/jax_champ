from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jax_teleop',
            executable='jax_keyboard_node.py',
            name='jax_keyboard',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'cmd_vel_topic': '/cmd_vel',
                'mode_topic': '/jax_mode',
                'walk_mode_name': 'walk',
                'stand_mode_name': 'stand',
                'sit_mode_name': 'sit',
                'lay_mode_name': 'lay',
                'linear_step': 0.06,
                'strafe_step': 0.06,
                'angular_step': 0.20,
                'max_linear_x': 0.50,
                'max_linear_y': 0.35,
                'max_angular_z': 1.20,
            }]
        )
    ])