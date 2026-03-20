#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import os
import time
from sensor_msgs.msg import BatteryState, JointState

class JaxSerialBridge(Node):
    def __init__(self):
        super().__init__('jax_serial_bridge')
        
        # =====================================================
        # MASTER JOINT MAPPING (URDF Name -> Arduino Index)
        # =====================================================
        # Indices: 0, 1, 2 = Rear Right | 3, 4, 5 = Rear Left
        # Indices: 6, 7, 8 = Front Left | 9, 10, 11 = Front Right
        self.joint_order = [
            "rh_hip_joint", "rh_lower_leg_joint", "rh_upper_leg_joint", # 0, 1, 2
            "lh_upper_leg_joint", "lh_hip_joint", "lh_lower_leg_joint", # 3, 4, 5
            "lf_upper_leg_joint", "lf_lower_leg_joint", "lf_hip_joint", # 6, 7, 8
            "rf_hip_joint", "rf_lower_leg_joint", "rf_upper_leg_joint"  # 9, 10, 11
        ]

        # 1. HARDWARE INITIALIZATION
        os.system("stty -F /dev/ttyAMA0 115200 raw -echo")
        time.sleep(1)

        try:
            self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.05)
            self.get_logger().info("Jax Serial Bridge: CONNECTED")
            
            # Wake the Arduino up (Red -> Blue LEDs + OE_PIN Low)
            self.ser.write(b"WAKE\n")
        except Exception as e:
            self.get_logger().error(f"FATAL: Serial Connection Failed: {e}")
            return

        # 2. ROS INTERFACE
        self.battery_pub = self.create_publisher(BatteryState, 'jax/battery', 10)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        
        # 10Hz Timer for Battery Telemetry
        self.timer = self.create_timer(0.1, self.read_from_arduino)

    def joint_callback(self, msg):
        """Sorts incoming ROS joints into the Arduino's 0-11 index order"""
        try:
            # Map names to their current values from the ROS message
            positions = dict(zip(msg.name, msg.position))
            
            # Rebuild the list based on our MASTER JOINT MAPPING
            sorted_angles = []
            for name in self.joint_order:
                if name in positions:
                    # Send raw radians with 4 decimal precision
                    sorted_angles.append(f"{positions[name]:.4f}")
                else:
                    # Default to 0.0 if a joint name isn't found in the message
                    sorted_angles.append("0.0000")

            # Send as "J:val1,val2,val3..."
            if len(sorted_angles) == 12:
                payload = "J:" + ",".join(sorted_angles) + "\n"
                self.ser.write(payload.encode('utf-8'))
                
        except Exception as e:
            self.get_logger().error(f"Mapping/Sort Error: {e}")

    def read_from_arduino(self):
        """Processes 'VOLT:xx.xx' strings from Arduino for Foxglove"""
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if "VOLT" in line:
                    voltage = float(line.split(':')[-1])
                    msg = BatteryState()
                    msg.voltage = voltage
                    msg.header.stamp = self.get_clock().now().to_msg()
                    self.battery_pub.publish(msg)
                    
                    if voltage < 13.8:
                        self.get_logger().warn(f"LOW BATTERY: {voltage}V")
        except Exception as e:
            pass 

def main(args=None):
    rclpy.init(args=args)
    node = JaxSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Safety: Go Limp (Red LEDs + OE_PIN High) on shutdown
        node.ser.write(b"SLEEP\n")
        node.get_logger().info("Jax is going to Sleep")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()