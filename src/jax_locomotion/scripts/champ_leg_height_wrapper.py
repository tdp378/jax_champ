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
from champ_msgs.msg import PointArray


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
            PointArray,  # Use the correct CHAMP message type
            '/champ/foot/links',
            self.champ_foot_callback,
            10
        )
        
        # Publish adjusted foot targets
        self.adjusted_foot_pub = self.create_publisher(
            PointArray,  # Use the correct message type
            '/champ/foot/links_adjusted',
            10
        )
        
        self.get_logger().info("CHAMP Leg Height Wrapper started")
    
    def leg_height_callback(self, msg: Float32MultiArray):
        """Store the latest leg height offsets from IMU stabilizer."""
        if len(msg.data) == 4:
            self.leg_height_offsets = list(msg.data)
    
    def champ_foot_callback(self, msg: PointArray):
        """
        Receive CHAMP foot targets and apply height adjustments.
        
        PointArray has lf, rf, lh, rh points with x,y,z.
        We modify the Z coordinate for each foot.
        """
        adjusted_msg = PointArray()
        
        # Copy the points and adjust Z
        adjusted_msg.lf = msg.lf
        adjusted_msg.lf.z += self.leg_height_offsets[0]  # FL
        
        adjusted_msg.rf = msg.rf
        adjusted_msg.rf.z += self.leg_height_offsets[1]  # FR
        
        adjusted_msg.lh = msg.lh
        adjusted_msg.lh.z += self.leg_height_offsets[2]  # BL? Wait, lh is left hind, which is BL?
        
        # Assuming leg order: FL, FR, BL, BR
        # But PointArray has lf, rf, lh, rh
        # lf = left front = FL
        # rf = right front = FR
        # lh = left hind = BL
        # rh = right hind = BR
        
        adjusted_msg.lh.z += self.leg_height_offsets[2]  # BL
        
        adjusted_msg.rh = msg.rh
        adjusted_msg.rh.z += self.leg_height_offsets[3]  # BR
        
        # Publish adjusted targets
        self.adjusted_foot_pub.publish(adjusted_msg)
        
        self.get_logger().debug(
            f"Adjusted foot targets: LF z={adjusted_msg.lf.z}, RF z={adjusted_msg.rf.z}, LH z={adjusted_msg.lh.z}, RH z={adjusted_msg.rh.z}"
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
