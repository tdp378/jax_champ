#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class JaxBehaviorNode(Node):
    def __init__(self):
        super().__init__('jax_behavior_node')

        self.mode_sub = self.create_subscription(
            String,
            '/jax_behavior_mode',
            self.mode_callback,
            10,
        )

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/jax/behavior_joint_trajectory',
            10,
        )

        self.joint_names = [
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint",
        ]

        self.get_logger().info("jax_behavior_node ready")

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
                0.0,  0.25, -0.2,
                0.0,  0.25, -0.2,
            ], move_time=1.2)

        elif mode == "lay":
            self.get_logger().info("Behavior: lay")
            self.publish_pose([
                0.0,  0.9, 0.3,
                0.0,  0.9, 0.3,
                0.0,  0.9, 0.2,
                0.0,  0.9, 0.2,
            ], move_time=1.5)

        elif mode == "walk":
            self.get_logger().info("Behavior: walk mode requested")
            # no pose command here; mode manager will hand control back to CHAMP

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