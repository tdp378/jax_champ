#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import os
import yaml
from ament_index_python.packages import get_package_share_directory

class JointRemapper(Node):
    def __init__(self):
        super().__init__('joint_remapper')
        
        # 1. Load the Map from YAML
        try:
            pkg_share = get_package_share_directory('jax_description')
            config_path = os.path.join(pkg_share, 'config', 'geometry.yaml')
            
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
                self.leg_config = data['legs']
                # Store indices mapping for quick access
                self.indices = data.get('indices', {
                    'lf': [0, 1, 2], 'rf': [3, 4, 5],
                    'lh': [6, 7, 8], 'rh': [9, 10, 11]
                })
            self.get_logger().info(f"SUCCESS: Remapper using logic from {config_path}")
        except Exception as e:
            self.get_logger().error(f"CRITICAL: Could not parse YAML: {e}")
            self.leg_config = None

        self.sub = self.create_subscription(JointTrajectory, '/jax/walk_joint_trajectory_raw', self.joint_callback, 10)
        self.pub = self.create_publisher(JointTrajectory, '/joint_group_effort_controller/joint_trajectory', 10)

    def joint_callback(self, msg):
        if not self.leg_config:
            return

        new_msg = JointTrajectory()
        new_msg.header = msg.header
        new_msg.joint_names = msg.joint_names

        for point in msg.points:
            new_point = JointTrajectoryPoint()
            new_point.time_from_start = point.time_from_start
            pos = list(point.positions)

            # Loop through each leg defined in your YAML (lf, rf, lh, rh)
            for leg_name, config in self.leg_config.items():
                idx = self.indices[leg_name] # e.g., [6, 7, 8] for lh
                
                # Apply Hip Flip if 'flip: true' is in YAML
                if config.get('flip', False):
                    pos[idx[0]] = pos[idx[0]] * -1.0
                
                # Apply Cantilever/Coupling if 'coupled: true' is in YAML
                if config.get('coupled', False):
                    # Standard Parallel Linkage: Lower = Lower - Upper
                    pos[idx[2]] = pos[idx[2]] - pos[idx[1]]

            new_point.positions = pos
            new_msg.points.append(new_point)

        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointRemapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()