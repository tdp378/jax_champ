#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class JaxBehaviorNode(Node):
    def __init__(self):
        super().__init__('jax_behavior_node')

        self.current_mode = 'stand'
        self.latest_pose_cmd = Twist()

        self.mode_sub = self.create_subscription(
            String,
            '/jax_behavior_mode',
            self.mode_callback,
            10,
        )

        self.declare_parameter('pose_cmd_topic', '/cmd_vel/smooth_stable')
        self.declare_parameter('pose_publish_hz', 20.0)
        self.declare_parameter('pose_move_time', 0.08)
        self.declare_parameter('pose_pitch_gain', 2.8)
        self.declare_parameter('pose_roll_gain', 2.8)
        self.declare_parameter('pose_yaw_gain', 1.6)
        self.declare_parameter('pose_pitch_limit', 0.22)
        self.declare_parameter('pose_roll_limit', 0.22)
        self.declare_parameter('pose_yaw_limit', 0.18)
        self.declare_parameter('pose_deadband', 0.02)

        self.pose_cmd_topic = str(self.get_parameter('pose_cmd_topic').value)
        self.pose_publish_hz = float(self.get_parameter('pose_publish_hz').value)
        self.pose_move_time = float(self.get_parameter('pose_move_time').value)
        self.pose_pitch_gain = float(self.get_parameter('pose_pitch_gain').value)
        self.pose_roll_gain = float(self.get_parameter('pose_roll_gain').value)
        self.pose_yaw_gain = float(self.get_parameter('pose_yaw_gain').value)
        self.pose_pitch_limit = float(self.get_parameter('pose_pitch_limit').value)
        self.pose_roll_limit = float(self.get_parameter('pose_roll_limit').value)
        self.pose_yaw_limit = float(self.get_parameter('pose_yaw_limit').value)
        self.pose_deadband = float(self.get_parameter('pose_deadband').value)

        self.pose_cmd_sub = self.create_subscription(
            Twist,
            self.pose_cmd_topic,
            self.pose_cmd_callback,
            10,
        )

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/jax/behavior_joint_trajectory',
            10,
        )

        self.pose_timer = self.create_timer(1.0 / self.pose_publish_hz, self.pose_update)

        self.joint_names = [
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint",
        ]

        self.get_logger().info("jax_behavior_node ready")

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def _apply_deadband(self, value):
        return 0.0 if abs(value) < self.pose_deadband else value

    def pose_cmd_callback(self, msg: Twist):
        self.latest_pose_cmd = msg

    def compute_pose_mode_positions(self, cmd: Twist):
        pitch = self._clamp(
            -self._apply_deadband(cmd.linear.x) * self.pose_pitch_gain,
            self.pose_pitch_limit,
        )
        roll = self._clamp(
            self._apply_deadband(cmd.linear.y) * self.pose_roll_gain,
            self.pose_roll_limit,
        )
        yaw = self._clamp(
            self._apply_deadband(cmd.angular.z) * self.pose_yaw_gain,
            self.pose_yaw_limit,
        )

        leg_signs = [
            (1.0, 1.0),   # LF: front, left
            (1.0, -1.0),  # RF: front, right
            (-1.0, 1.0),  # LH: rear, left
            (-1.0, -1.0), # RH: rear, right
        ]

        positions = []
        for sign_fb, sign_lr in leg_signs:
            # Small-angle approximation: coordinated joint deltas that shift body
            # around while trying to keep foot contacts close to the same points.
            hip = (-sign_lr * roll) + (0.55 * sign_fb * yaw)
            upper = (-sign_fb * pitch) - (0.35 * sign_lr * roll)
            lower = (sign_fb * pitch) - (0.35 * sign_lr * roll)

            positions.extend([
                self._clamp(hip, 0.35),
                self._clamp(upper, 0.45),
                self._clamp(lower, 0.45),
            ])

        return positions

    def pose_update(self):
        if self.current_mode != 'pose':
            return

        self.publish_pose(
            self.compute_pose_mode_positions(self.latest_pose_cmd),
            move_time=self.pose_move_time,
        )

    def publish_pose(self, positions, move_time=1.0):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = positions

        d = Duration()
        d.sec = int(move_time)
        d.nanosec = int((move_time - int(move_time)) * 1e9)
        pt.time_from_start = d

        msg.points = [pt]
        self.traj_pub.publish(msg)

    def mode_callback(self, msg: String):
        mode = msg.data.strip().lower()
        self.current_mode = mode

        if mode == "stand":
            self.get_logger().info("Behavior: stand")
            self.publish_pose([
                0, 0, 0,
                0, 0, 0,
                0, 0, 0,
                0, 0, 0
            ], move_time=1.0)

        elif mode == "sit":
            self.get_logger().info("Behavior: sit")
            self.publish_pose([
                0.0,  0.0, 0.5,
                0.0,  0.0, 0.5,
                0.0,  0.25, 0.0,
                0.0,  0.25, 0.0,
            ], move_time=1.2)

        elif mode == "lay":
            self.get_logger().info("Behavior: lay")
            self.publish_pose([
                0.0,  0.8, 0.0,
                0.0,  0.8, 0.0,
                0.0,  0.8, 0.0,
                0.0,  0.8, 0.0,
            ], move_time=1.5)


        elif mode == "paw":
            self.get_logger().info("Behavior: paw - Sitting")
            # STAGE 1: Shift body to the Left and slightly Forward
            self.publish_pose([
                0.0,  0.0, 0.5,
                0.0,  0.0, 0.5,
                0.0,  0.25, 0.0,
                0.0,  0.25, 0.0,
            ], move_time=1.2)

            # Small pause to let the servos settle the weight
            time.sleep(0.8)

            # STAGE 2: The actual Lift
            self.get_logger().info("Behavior: paw - Shifting Weight")
            self.publish_pose([
                0.15,  0.1,  0.8,   # LF: Stay braced
                0.0,  0.0, 0.5,   # RF: Lift high, Hip further out for clearance
                0.15, 0.25, -0.2,  # LH: Stay tucked
                -0.05, 0.25, 0.0   # RH: Stay tucked
            ], move_time=0.6)

            self.get_logger().info("Behavior: paw - Lifting Paw")
            self.publish_pose([
                0.15,  0.1,  0.6,   # LF: Stay braced
                -0.2, -1.0, -1.4,   # RF: Lift high, Hip further out for clearance
                0.15, 0.25, -0.2,  # LH: Stay tucked
                -0.05, 0.25, 0.0   # RH: Stay tucked
            ], move_time=0.2)

        elif mode == "walk":
            self.get_logger().info("Behavior: walk mode requested")
            # no pose command here; mode manager will hand control back to CHAMP

        elif mode == "pose":
            self.get_logger().info("Behavior: pose (body shift)")
            self.publish_pose(
                self.compute_pose_mode_positions(self.latest_pose_cmd),
                move_time=0.3,
            )

        else:
            self.get_logger().warn(f"Unknown mode: {mode}")


def main(args=None):
    rclpy.init(args=args)
    node = JaxBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()