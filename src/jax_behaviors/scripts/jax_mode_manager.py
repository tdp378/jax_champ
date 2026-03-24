#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory


class JaxModeManager(Node):
    def __init__(self):
        super().__init__('jax_mode_manager')

        self.declare_parameter('transition_duration', 1.8)
        self.declare_parameter('startup_mode', 'lay')

        # Use a consistent naming convention
        self.current_mode = str(self.get_parameter('startup_mode').value).strip().lower()
        self.walk_enable_time = None
        self.transition_duration = float(self.get_parameter('transition_duration').value)
        
        # Variables for the safe landing sequence
        self.target_mode_after_landing = None
        self.landing_end_time = None

        self.mode_sub = self.create_subscription(
            String,
            '/jax_mode',
            self.mode_callback,
            10,
        )

        self.cmd_vel_in_sub = self.create_subscription(
            Twist,
            '/cmd_vel/smooth_stable',
            self.cmd_vel_callback,
            10,
        )

        self.walk_traj_sub = self.create_subscription(
            JointTrajectory,
            '/jax/walk_joint_trajectory',
            self.walk_traj_callback,
            10,
        )

        self.behavior_traj_sub = self.create_subscription(
            JointTrajectory,
            '/jax/behavior_joint_trajectory',
            self.behavior_traj_callback,
            10,
        )

        self.cmd_vel_walk_pub = self.create_publisher(
            Twist,
            '/jax/cmd_vel_walk',
            10,
        )

        self.controller_traj_pub = self.create_publisher(
            JointTrajectory,
            '/jax/walk_joint_trajectory_raw',
            10,
        )

        self.behavior_mode_pub = self.create_publisher(
            String,
            '/jax_behavior_mode',
            10,
        )

        self.timer = self.create_timer(0.05, self.update)

        # Initial startup
        self.publish_behavior_mode(self.current_mode)

        self.get_logger().info(
            f'jax_mode_manager ready, starting in {self.current_mode.upper()} mode'
        )

    def publish_zero_cmd(self):
        self.cmd_vel_walk_pub.publish(Twist())

    def publish_behavior_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.behavior_mode_pub.publish(msg)

    def mode_callback(self, msg: String):
        new_mode = msg.data.strip().lower()

        if new_mode not in ['walk', 'stand', 'sit', 'lay', 'paw']:
            self.get_logger().warn(f'Ignoring unknown mode: {new_mode}')
            return

        # If we are already landing, ignore new commands until foot is down
        if self.current_mode == 'landing':
            self.get_logger().warn('Currently landing, please wait...')
            return

        # --- THE FIX: DETECT EXITING PAW ---
        if self.current_mode == 'paw' and new_mode != 'paw':
            self.get_logger().info('DETECTED EXIT FROM PAW: Transitioning to SIT first...')
            self.target_mode_after_landing = new_mode
            self.landing_end_time = (self.get_clock().now().nanoseconds / 1e9) + 1.2
            self.current_mode = 'landing'
            
            # Immediately tell behavior node to go to sit (touchdown)
            self.publish_behavior_mode('sit')
            return

        # Normal mode execution
        self._execute_mode_change(new_mode)

    def _execute_mode_change(self, new_mode):
        if new_mode == 'walk':
            self.current_mode = 'transition_to_walk'
            self.walk_enable_time = (self.get_clock().now().nanoseconds / 1e9) + self.transition_duration
            self.publish_zero_cmd()
            self.publish_behavior_mode('stand')
            self.get_logger().info('Transitioning to WALK via STAND')
        else:
            self.current_mode = new_mode
            self.walk_enable_time = None
            self.publish_zero_cmd()
            self.publish_behavior_mode(new_mode)
            self.get_logger().info(f'Mode set to: {self.current_mode}')

    def cmd_vel_callback(self, msg: Twist):
        if self.current_mode == 'walk':
            self.cmd_vel_walk_pub.publish(msg)

    def publish_raw_trajectory(self, msg: JointTrajectory):
        self.controller_traj_pub.publish(msg)

    def walk_traj_callback(self, msg: JointTrajectory):
        if self.current_mode == 'walk':
            self.publish_raw_trajectory(msg)

    def behavior_traj_callback(self, msg: JointTrajectory):
        if self.current_mode != 'walk':
            self.publish_raw_trajectory(msg)

    def update(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        
        if self.current_mode != 'walk':
            self.publish_zero_cmd()

        # Handle the Landing State completion
        if self.current_mode == 'landing' and self.landing_end_time is not None:
            if now_sec >= self.landing_end_time:
                self.get_logger().info('Landing sequence complete.')
                target = self.target_mode_after_landing
                self.landing_end_time = None
                self.target_mode_after_landing = None
                self._execute_mode_change(target)

        # Handle Walk Transition completion
        if self.current_mode == 'transition_to_walk' and self.walk_enable_time is not None:
            if now_sec >= self.walk_enable_time:
                self.current_mode = 'walk'
                self.walk_enable_time = None
                self.get_logger().info('Walk mode enabled')


def main(args=None):
    rclpy.init(args=args)
    node = JaxModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()