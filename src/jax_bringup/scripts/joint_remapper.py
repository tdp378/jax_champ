#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState

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
                self.indices = data.get('indices', {
                    'lf': [0, 1, 2], 'rf': [3, 4, 5],
                    'lh': [6, 7, 8], 'rh': [9, 10, 11]
                })
            self.get_logger().info(f"SUCCESS: Remapper loaded from {config_path}")
        except Exception as e:
            self.get_logger().error(f"CRITICAL: Could not parse YAML: {e}")
            self.leg_config = None

        # CHAMP/Trajectory Subscriptions
        self.sub = self.create_subscription(JointTrajectory, '/jax/walk_joint_trajectory_raw', self.joint_callback, 10)
        self.pub = self.create_publisher(JointTrajectory, '/joint_group_effort_controller/joint_trajectory', 10)

        # GUI/RViz Sliders Subscriptions
        self.js_sub = self.create_subscription(JointState, '/joint_states_raw', self.js_callback, 10)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)

        # --- DYNAMIC LIMIT CONFIG ---
        self.NEUTRAL_LIMIT = 1.0  # Range where calf can stay level
        self.MAX_PHYSICAL = 2.0   # Absolute hardware stop

    def apply_linkage_logic(self, thigh_val, calf_target):
        """
        Calculates the motor position based on the Thigh's 
        position and the Calf's goal, with Cooperative Range.
        """
        # 1. Base Remap (Subtracting Thigh because they move opposite to crouch)
        motor_request = calf_target - thigh_val

        # 2. COOPERATIVE LIMIT LOGIC
        # If Thigh pushes past 1.0, the Calf MUST assist the rod
        if thigh_val > self.NEUTRAL_LIMIT:
            # Assist the rod by moving in the same direction as thigh
            assist_offset = thigh_val - self.NEUTRAL_LIMIT
            final_pos = max(motor_request, -self.NEUTRAL_LIMIT + assist_offset)
        elif thigh_val < -self.NEUTRAL_LIMIT:
            assist_offset = thigh_val + self.NEUTRAL_LIMIT
            final_pos = min(motor_request, self.NEUTRAL_LIMIT + assist_offset)
        else:
            # Standard Neutral Zone Clipping
            final_pos = max(min(motor_request, self.NEUTRAL_LIMIT), -self.NEUTRAL_LIMIT)
        
        return final_pos

    def js_callback(self, msg):
        new_js = msg
        pos = list(msg.position)
        
        # Indices: LF(1,2), RF(4,5), LH(7,8), RH(10,11)
        legs = [(1, 2), (4, 5), (7, 8), (10, 11)]

        for t_idx, c_idx in legs:
            if c_idx < len(pos) and t_idx < len(pos):
                pos[c_idx] = self.apply_linkage_logic(pos[t_idx], pos[c_idx])

        new_js.position = tuple(pos)
        self.js_pub.publish(new_js)
    
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

            for leg_name, config in self.leg_config.items():
                idx = self.indices[leg_name] # e.g., [0, 1, 2]
                
                # Apply Hip Flip
                if config.get('flip', False):
                    pos[idx[0]] = pos[idx[0]] * -1.0
                
                # Apply Coupled Logic (Thigh=idx[1], Calf=idx[2])
                if config.get('coupled', False):
                    pos[idx[2]] = self.apply_linkage_logic(pos[idx[1]], pos[idx[2]])

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