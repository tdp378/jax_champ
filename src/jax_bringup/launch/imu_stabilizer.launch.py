from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # ============================================================
        # IMU Leg Height Stabilizer
        # ============================================================
        Node(
            package='jax_locomotion',
            executable='jax_imu_leg_height_stabilizer',
            name='imu_leg_height_stabilizer',
            output='screen',
            parameters=[
                {
                    'kp_roll': 0.05,
                    'ki_roll': 0.005,
                    'kd_roll': 0.02,
                    
                    'kp_pitch': 0.05,
                    'ki_pitch': 0.005,
                    'kd_pitch': 0.02,
                    
                    'imu_process_noise': 0.001,
                    'imu_measurement_noise': 0.05,
                    
                    'max_leg_height_adjustment': 0.05,
                    'deadband_angle': 0.005,
                    
                    'enable_speed_adaptation': True,
                    'speed_scale_factor': 1.0,
                    
                    'enabled': True,
                    'only_when_moving': True,
                    'motion_threshold': 0.02,
                    
                    'publish_diagnostics': True,
                }
            ],
            remappings=[
                ('/imu/data', '/imu/data'),
                ('/cmd_vel', '/cmd_vel/smooth'),
            ]
        ),
        
        # ============================================================
        # CHAMP Leg Height Wrapper (bridges IMU offsets to CHAMP)
        # ============================================================
        Node(
            package='jax_locomotion',
            executable='champ_leg_height_wrapper',
            name='champ_leg_height_wrapper',
            output='screen',
            parameters=[
                {
                    'leg_names': ['FL', 'FR', 'BL', 'BR'],
                }
            ],
            remappings=[
                ('/jax/leg_height_offsets', '/jax/leg_height_offsets'),
                ('/champ/foot/links', '/champ/foot/links'),
                ('/champ/foot/links_adjusted', '/champ/foot/links_adjusted'),
            ]
        ),
    ])
