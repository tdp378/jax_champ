#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Boilerplate to ensure local imports work in CMake/Install space
sys.path.insert(0, os.path.dirname(__file__))
from poses import POSES

class JaxModeManager(Node):
    def __init__(self):
        super().__init__('jax_mode_manager')
        
        self.current_pose = POSES["stand"]
        self.target_pose = POSES["stand"]
        self.transition_steps = 50 
        self.step_count = self.transition_steps # Start "finished"
        
        self.sub = self.create_subscription(String, '/jax/set_mode', self.mode_callback, 10)
        self.pub = self.create_publisher(JointTrajectory, '/jax/joint_commands/linkage_corrected', 10)
        self.timer = self.create_timer(0.02, self.interpolation_loop)
        
        self.get_logger().info("Jax Mode Manager ready. Poses loaded from poses.py")

    def mode_callback(self, msg):
        mode = msg.data.lower()
        if mode in POSES:
            # We grab the angles from the imported POSES dictionary
            self.target_pose = POSES[mode]
            self.step_count = 0 
            self.get_logger().info(f"Transitioning to: {mode}")
        else:
            self.get_logger().warn(f"Pose '{mode}' is not defined in poses.py")

    def interpolation_loop(self):
        if self.step_count < self.transition_steps:
            self.step_count += 1
            alpha = self.step_count / self.transition_steps
            
            new_angles = [
                self.current_pose[i] + (self.target_pose[i] - self.current_pose[i]) * alpha 
                for i in range(12)
            ]
            
            self.publish_angles(new_angles)
            
            if self.step_count == self.transition_steps:
                self.current_pose = list(self.target_pose)

    def publish_angles(self, angles):
        msg = JointTrajectory()
        msg.joint_names = [
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint"
        ]
        point = JointTrajectoryPoint()
        point.positions = angles
        msg.points.append(point)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = JaxModeManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()