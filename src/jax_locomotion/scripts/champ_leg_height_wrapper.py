#!/usr/bin/env python3

"""
CHAMP Leg Height Adjustment Wrapper
====================================

This node wraps CHAMP's locomotion output and applies per-leg height
adjustments from the IMU stabilizer before sending joint commands.

Subscribes to:
  - /champ/body/frame (CHAMP's body state)
  - /jax/leg_height_offsets (from IMU stabilizer)

Publishes to:
  - /champ/foot/links (modified foot target positions with height offsets)
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
# You may need additional CHAMP message types here
# from champ_msgs.msg import BodyState, FootTarget


class ChampLegHeightWrapper(Node):
    
    def __init__(self):
        super().__init__('champ_leg_height_wrapper')
        
        self.declare_parameter('leg_names', ['FL', 'FR', 'BL', 'BR'])
        self.leg_names = self.get_parameter('leg_names').value
        
        # Current leg height offsets
        self.leg_height_offsets = [0.0, 0.0, 0.0, 0.0]
        
        # Subscribe to leg height adjustments
        self.leg_height_sub = self.create_subscription(
            Float32MultiArray,
            '/jax/leg_height_offsets',
            self.leg_height_callback,
            10
        )
        
        # Subscribe to CHAMP's foot targets
        # NOTE: You may need to adjust these topics based on your CHAMP setup
        # Common topics: /champ/foot/links, /champ/body/foot_target, etc.
        self.champ_foot_sub = self.create_subscription(
            Float32MultiArray,  # Placeholder - adjust to actual CHAMP message type
            '/champ/foot/links',
            self.champ_foot_callback,
            10
        )
        
        # Publish adjusted foot targets
        self.adjusted_foot_pub = self.create_publisher(
            Float32MultiArray,  # Placeholder
            '/champ/foot/links_adjusted',
            10
        )
        
        self.get_logger().info("CHAMP Leg Height Wrapper started")
    
    def leg_height_callback(self, msg: Float32MultiArray):
        """Store the latest leg height offsets from IMU stabilizer."""
        if len(msg.data) == 4:
            self.leg_height_offsets = list(msg.data)
    
    def champ_foot_callback(self, msg: Float32MultiArray):
        """
        Receive CHAMP foot targets and apply height adjustments.
        
        Expected format: [FL_x, FL_y, FL_z, FR_x, FR_y, FR_z, BL_x, BL_y, BL_z, BR_x, BR_y, BR_z]
        We modify the Z (height) component for each foot.
        """
        if len(msg.data) != 12:
            self.get_logger().warn(f"Expected 12 values (4 legs × 3 coords), got {len(msg.data)}")
            return
        
        adjusted_data = list(msg.data)
        
        # Modify Z coordinate of each leg's foot position
        # Indices: FL=2, FR=5, BL=8, BR=11
        leg_z_indices = [2, 5, 8, 11]
        
        for i, leg_idx in enumerate(leg_z_indices):
            adjusted_data[leg_idx] += self.leg_height_offsets[i]
        
        # Publish adjusted targets
        adjusted_msg = Float32MultiArray()
        adjusted_msg.data = adjusted_data
        self.adjusted_foot_pub.publish(adjusted_msg)
        
        self.get_logger().debug(
            f"Adjusted foot targets: {[f'{x:.4f}' for x in adjusted_data]}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ChampLegHeightWrapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
