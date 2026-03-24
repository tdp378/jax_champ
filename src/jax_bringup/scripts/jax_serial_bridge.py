#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import os
import time
import subprocess  # Added for audio playback
from sensor_msgs.msg import BatteryState
from trajectory_msgs.msg import JointTrajectory

class JaxSerialBridge(Node):
    def __init__(self):
        super().__init__('jax_serial_bridge')
        
        # This matches the 12-channel order your Arduino expects
        self.arduino_order = [
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint"
        ]

        self.current_pos = [0.0] * 12 
        self.target_pos = [0.0] * 12   
        self.step_size = 0.01 # Smooth transition speed
        self.armed = False

        # --- SERIAL SETUP ---
        try:
            # Open port and wait for Arduino to finish its internal setup
            self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.1)
            time.sleep(2.0) 
            
            # Initial Wake attempt
            self.ser.write(b"WAKE\n")
            self.get_logger().info("Jax Bridge: Serial Port Open. Waiting for ROS data...")
        except Exception as e:
            self.get_logger().error(f"SERIAL CONNECTION FAILED: {e}")
            return

        # Subscriptions
        self.traj_sub = self.create_subscription(
            JointTrajectory, 
            '/joint_group_effort_controller/joint_trajectory', 
            self.traj_callback, 
            10)
        
        # Publishers (For your battery logs)
        self.battery_pub = self.create_publisher(BatteryState, 'jax/battery', 10)
        
        # 50Hz Loop
        self.timer = self.create_timer(0.02, self.loop_callback)

    def traj_callback(self, msg):
        if not msg.points: return
        
        # Map incoming ROS joint names to their target values
        incoming_data = dict(zip(msg.joint_names, msg.points[0].positions))
        
        # Reorder them for the Arduino's 0-11 channel logic
        for i, joint_name in enumerate(self.arduino_order):
            if joint_name in incoming_data:
                self.target_pos[i] = incoming_data[joint_name]

    def loop_callback(self):
        # 1. ARMING: If we see data but aren't armed, send WAKE and PLAY SOUND
        if not self.armed:
            try:
                # Send the serial command for the NeoPixels/Servos
                self.ser.write(b"WAKE\n")
                
                # TRIGGER THE AUDIO CLIP (Non-blocking)
                # Ensure the path points to your actual .wav file
                subprocess.Popen(['aplay', '/home/tdp378/jax_champ/src/sounds/bootup_complete.wav']) 
                
                self.armed = True
                self.get_logger().info("!!! JAX ARMED: NeoPixels Active & Playing Sound !!!")
            except Exception as e:
                self.get_logger().error(f"Failed to arm or play sound: {e}")

        # 2. INTERPOLATION: Move current toward target using step_size
        changed = False
        for i in range(12):
            diff = self.target_pos[i] - self.current_pos[i]
            if abs(diff) > 0.001:
                move = self.step_size if abs(diff) > self.step_size else abs(diff)
                self.current_pos[i] += move if diff > 0 else -move
                changed = True

        # 3. SEND TO ARDUINO: "J:val1,val2...val12\n"
        if changed:
            # Radians are expected by your Arduino's applyPoseRadians function
            payload = "J:" + ",".join([f"{v:.4f}" for v in self.current_pos]) + "\n"
            try:
                self.ser.write(payload.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Write failed: {e}")
        
        # 4. READ VOLTAGE: Process "VOLT:XX.XX" from Arduino
        self.read_serial_feedback()

    def read_serial_feedback(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if "VOLT:" in line:
                    msg = BatteryState()
                    # Extract the float from "VOLT:16.45"
                    msg.voltage = float(line.split(':')[-1])
                    self.battery_pub.publish(msg)
        except:
            pass 

def main(args=None):
    rclpy.init(args=args)
    node = JaxSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Put robot in LIMP mode on shutdown
        node.ser.write(b"SLEEP\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()