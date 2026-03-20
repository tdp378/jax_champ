#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import os
import time
from sensor_msgs.msg import BatteryState

class JaxSerialBridge(Node):
    def __init__(self):
        super().__init__('jax_serial_bridge')
        
        # 1. HARDWARE SELF-HEAL: Force the Pi to listen correctly
        self.get_logger().info("Cleaning serial port /dev/ttyAMA0...")
        os.system("stty -F /dev/ttyAMA0 115200 raw -echo")
        time.sleep(1) # Give the hardware a second to breathe

        # 2. Setup Serial
        try:
            self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.5)
            self.get_logger().info("Jax Serial Bridge: CONNECTED")
        except Exception as e:
            self.get_logger().error(f"FATAL: Could not open serial port: {e}")
            return

        # 3. ROS Publishers
        self.battery_pub = self.create_publisher(BatteryState, 'jax/battery', 10)
        
        # 4. Main Loop Timer (10Hz)
        self.timer = self.create_timer(0.1, self.read_from_arduino)

    def read_from_arduino(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Hear the raw data for debugging
                if line:
                    self.get_logger().info(f"Jax heard: '{line}'")

                # Parse VOLT:15.82 or BATT:15.82
                if "VOLT" in line or "BATT" in line:
                    val_str = line.split(':')[-1].strip()
                    voltage = float(val_str)

                    # Create ROS message
                    msg = BatteryState()
                    msg.voltage = voltage
                    msg.header.stamp = self.get_clock().now().to_msg()
                    
                    self.battery_pub.publish(msg)

                    # Safety Warning
                    if voltage < 13.8:
                        self.get_logger().warn(f"BATTERY LOW: {voltage}V - Charge Jax soon!")

        except Exception as e:
            self.get_logger().error(f"Bridge Processing Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JaxSerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()