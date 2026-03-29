#!/usr/bin/env python3
import sys
import os
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(__file__))
from poses import POSES


class JaxModeManager(Node):
    def __init__(self):
        super().__init__('jax_mode_manager')

        self.joint_names = [
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint"
        ]

        self.static_modes = {"stand", "sit", "lay"}
        self.valid_modes = self.static_modes | {"walk"}

        self.current_mode = "stand"
        self.current_pose = list(POSES["stand"])
        self.target_pose = list(POSES["stand"])

        self.transition_steps = 50
        self.step_count = self.transition_steps

        self.latest_joint_positions = None

        self.mode_sub = self.create_subscription(
            String,
            '/jax_mode',
            self.mode_callback,
            10
        )

        self.walk_sub = self.create_subscription(
            JointTrajectory,
            '/jax/walk_joint_trajectory_raw',
            self.walk_callback,
            10
        )

        self.js_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            50
        )

        # IMPORTANT: publish directly to the real controller
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/jax/combined_joint_trajectory',
            10
        )

        self.timer = self.create_timer(0.02, self.interpolation_loop)

        self.get_logger().info("Jax Mode Manager ready. Direct-to-controller mode routing enabled.")

    def mode_callback(self, msg):
        mode = msg.data.lower().strip()

        if mode not in self.valid_modes:
            self.get_logger().warn(
                f"Mode '{mode}' is not valid. Valid modes: {sorted(self.valid_modes)}"
            )
            return

        if mode == self.current_mode:
            return

        old_mode = self.current_mode
        self.current_mode = mode
        self.get_logger().info(f"Mode change: {old_mode} -> {mode}")

        if mode == "walk":
            self.step_count = self.transition_steps
            return

        if mode in self.static_modes:
            snap = self.get_latest_pose_in_expected_order()
            if snap is not None:
                self.current_pose = snap
                self.get_logger().info("Using live joint snapshot for smooth transition.")
            else:
                self.get_logger().warn("No valid /joint_states snapshot available; using last known pose.")

            self.target_pose = list(POSES[mode])
            self.step_count = 0
            self.get_logger().info(f"Transitioning smoothly to: {mode}")

    def joint_state_callback(self, msg: JointState):
        if not msg.name or not msg.position:
            return

        name_to_pos = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                name_to_pos[name] = msg.position[i]

        if all(name in name_to_pos for name in self.joint_names):
            self.latest_joint_positions = [name_to_pos[name] for name in self.joint_names]

    def get_latest_pose_in_expected_order(self):
        if self.latest_joint_positions is None:
            return None
        if len(self.latest_joint_positions) != 12:
            return None
        if any(math.isnan(x) or math.isinf(x) for x in self.latest_joint_positions):
            return None
        return list(self.latest_joint_positions)

    def walk_callback(self, msg: JointTrajectory):
        if self.current_mode == "walk":
            self.traj_pub.publish(msg)

    def interpolation_loop(self):
        if self.current_mode not in self.static_modes:
            return

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
        msg.joint_names = list(self.joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(angles)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 20000000

        msg.points.append(point)
        self.traj_pub.publish(msg)


def main():
    rclpy.init()
    node = JaxModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()