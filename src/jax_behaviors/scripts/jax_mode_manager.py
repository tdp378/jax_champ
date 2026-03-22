#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory


class JaxModeManager(Node):
    def __init__(self):
        super().__init__('jax_mode_manager')

       
        self.declare_parameter('transition_duration', 1.8)
        self.declare_parameter('startup_mode', 'lay')

        self.mode = str(self.get_parameter('startup_mode').value).strip().lower()
        self.walk_enable_time = None
        self.transition_duration = float(self.get_parameter('transition_duration').value)

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

        if self.mode in ['stand', 'sit', 'lay']:
            self.publish_behavior_mode(self.mode)

        self.get_logger().info(
            f'jax_mode_manager ready, starting in {self.mode.upper()} mode'
        )

    def publish_zero_cmd(self):
        self.cmd_vel_walk_pub.publish(Twist())

    def publish_behavior_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.behavior_mode_pub.publish(msg)

    def mode_callback(self, msg: String):
        new_mode = msg.data.strip().lower()

        if new_mode not in ['walk', 'stand', 'sit', 'lay']:
            self.get_logger().warn(f'Ignoring unknown mode: {new_mode}')
            return

        if new_mode == 'walk':
            self.mode = 'transition_to_walk'
            self.walk_enable_time = self.get_clock().now().nanoseconds / 1e9 + self.transition_duration
            self.publish_zero_cmd()
            self.publish_behavior_mode('stand')
            self.get_logger().info('Transitioning to WALK via STAND')
        else:
            self.mode = new_mode
            self.walk_enable_time = None
            self.publish_zero_cmd()
            self.publish_behavior_mode(new_mode)
            self.get_logger().info(f'Mode set to: {self.mode}')

    def cmd_vel_callback(self, msg: Twist):
        if self.mode == 'walk':
            self.cmd_vel_walk_pub.publish(msg)

    def walk_traj_callback(self, msg: JointTrajectory):
        if self.mode == 'walk':
            self.controller_traj_pub.publish(msg)

    def behavior_traj_callback(self, msg: JointTrajectory):
        if self.mode in ['stand', 'sit', 'lay', 'transition_to_walk']:
            self.controller_traj_pub.publish(msg)

    def update(self):
        if self.mode != 'walk':
            self.publish_zero_cmd()

        if self.mode == 'transition_to_walk' and self.walk_enable_time is not None:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec >= self.walk_enable_time:
                self.mode = 'walk'
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