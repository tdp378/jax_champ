#!/usr/bin/env python3

import copy
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JaxLegSafetyNode(Node):

    def __init__(self) -> None:
        super().__init__('jax_leg_safety_node')

        # -----------------------------
        # Topics
        # -----------------------------
        self.declare_parameter('traj_input_topic', '/jax/walk_joint_trajectory_raw')
        self.declare_parameter('traj_output_topic', '/joint_group_effort_controller/joint_trajectory')
        self.declare_parameter('js_input_topic', '/joint_states_raw')
        self.declare_parameter('js_output_topic', '/joint_states')

        self.traj_input_topic = self.get_parameter('traj_input_topic').value
        self.traj_output_topic = self.get_parameter('traj_output_topic').value
        self.js_input_topic = self.get_parameter('js_input_topic').value
        self.js_output_topic = self.get_parameter('js_output_topic').value

        # -----------------------------
        # Joint indices [hip, thigh, calf]
        # -----------------------------
        self.declare_parameter('lf_indices', [0, 1, 2])
        self.declare_parameter('rf_indices', [3, 4, 5])
        self.declare_parameter('lh_indices', [6, 7, 8])
        self.declare_parameter('rh_indices', [9, 10, 11])

        self.indices = {
            'lf': list(self.get_parameter('lf_indices').value),
            'rf': list(self.get_parameter('rf_indices').value),
            'lh': list(self.get_parameter('lh_indices').value),
            'rh': list(self.get_parameter('rh_indices').value),
        }

        # -----------------------------
        # Direction signs
        # -----------------------------
        self.declare_parameter('thigh_direction_sign', 1.0)
        self.declare_parameter('calf_direction_sign', 1.0)

        self.thigh_direction_sign = float(self.get_parameter('thigh_direction_sign').value)
        self.calf_direction_sign = float(self.get_parameter('calf_direction_sign').value)

        # -----------------------------
        # Margins (RADIANS)
        # -----------------------------
        self.declare_parameter('min_margin_rad', 0.0)
        self.declare_parameter('max_margin_rad', 0.0)

        self.min_margin_rad = float(self.get_parameter('min_margin_rad').value)
        self.max_margin_rad = float(self.get_parameter('max_margin_rad').value)

        # -----------------------------
        # Passive behavior
        # -----------------------------
        self.declare_parameter('enable_passive_calf', True)
        self.declare_parameter('neutral_thigh_deg', 0.0)

        self.enable_passive_calf = bool(self.get_parameter('enable_passive_calf').value)
        self.neutral_thigh_deg = self.get_parameter('neutral_thigh_deg').value * 3.14159 / 180.0

        # -----------------------------
        # Logging
        # -----------------------------
        self.declare_parameter('warn_on_clamp', True)
        self.warn_on_clamp = bool(self.get_parameter('warn_on_clamp').value)

        # ============================================================
        # 🔥 SAFETY ENVELOPE (RADIANS)
        # ============================================================
        self.table_rad: List[Tuple[float, float, float]] = [
            (-1.40,   0.733,  0.733),
            (-1.00,     0.0,  0.733),
            (-0.70,  -0.700,  0.733),
            (-0.60,  -0.800,  0.733),
            (-0.50,  -0.900,  0.733),
            (-0.40,  -1.000,  0.733),
            (-0.30,  -1.000,  0.733),
            (-0.20,  -1.000,  0.733),
            (-0.10,  -1.000,  0.733),
            ( 0.000, -1.000,  0.733),
            ( 1.047, -1.000, -0.524),
        ]

        # ============================================================
        # 🔥 PASSIVE CURVE (RADIANS)
        # ============================================================
        self.passive_table_rad: List[Tuple[float, float]] = [
            (-1.40,  0.733),       
            ( 0.000,  0.000),
            ( 0.9, -1.000),
        ]

        # -----------------------------
        # ROS I/O
        # -----------------------------
        self.traj_sub = self.create_subscription(
            JointTrajectory,
            self.traj_input_topic,
            self.joint_callback,
            10,
        )

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            self.traj_output_topic,
            10,
        )

        self.js_sub = self.create_subscription(
            JointState,
            self.js_input_topic,
            self.js_callback,
            10,
        )

        self.js_pub = self.create_publisher(
            JointState,
            self.js_output_topic,
            10,
        )

        self.get_logger().info("Jax Leg Safety Node (RAD version) started")

    # ============================================================
    # HELPERS
    # ============================================================

    def clamp(self, val, lo, hi):
        return max(lo, min(hi, val))

    def lerp(self, x, x0, y0, x1, y1):
        if abs(x1 - x0) < 1e-9:
            return y0
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0)

    def interpolate_limits_rad(self, upper):
        table = self.table_rad

        if upper <= table[0][0]:
            return upper, table[0][1], table[0][2]

        if upper >= table[-1][0]:
            return upper, table[-1][1], table[-1][2]

        for i in range(len(table) - 1):
            x0, min0, max0 = table[i]
            x1, min1, max1 = table[i + 1]

            if x0 <= upper <= x1:
                return (
                    upper,
                    self.lerp(upper, x0, min0, x1, min1),
                    self.lerp(upper, x0, max0, x1, max1),
                )

        return upper, table[-1][1], table[-1][2]

    def interpolate_passive_rad(self, upper):
        table = self.passive_table_rad

        if upper <= table[0][0]:
            return table[0][1]

        if upper >= table[-1][0]:
            return table[-1][1]

        for i in range(len(table) - 1):
            x0, y0 = table[i]
            x1, y1 = table[i + 1]

            if x0 <= upper <= x1:
                return self.lerp(upper, x0, y0, x1, y1)

        return table[-1][1]

    def apply_safety_margins_rad(self, lower_min, lower_max):
        safe_min = lower_min + self.min_margin_rad
        safe_max = lower_max - self.max_margin_rad

        if safe_min > safe_max:
            mid = 0.5 * (safe_min + safe_max)
            return mid, mid

        return safe_min, safe_max

    # ============================================================
    # CORE LOGIC
    # ============================================================

    def apply_leg_safety(self, thigh, calf):
        thigh_eff = thigh * self.thigh_direction_sign
        calf_eff = calf * self.calf_direction_sign

        safe_thigh, low_min, low_max = self.interpolate_limits_rad(thigh_eff)
        low_min, low_max = self.apply_safety_margins_rad(low_min, low_max)

        passive_delta = 0.0
        if self.enable_passive_calf:
            passive_here = self.interpolate_passive_rad(safe_thigh)
            passive_neutral = self.interpolate_passive_rad(self.neutral_thigh_deg)
            passive_delta = passive_here - passive_neutral

        requested_calf = calf_eff + passive_delta
        safe_calf = self.clamp(requested_calf, low_min, low_max)

        safe_thigh /= self.thigh_direction_sign
        safe_calf /= self.calf_direction_sign

        return safe_thigh, safe_calf, (low_min, low_max), passive_delta

    # ============================================================
    # CALLBACKS
    # ============================================================

    def js_callback(self, msg: JointState):
        out = copy.deepcopy(msg)
        pos = list(out.position)

        for leg, idx in self.indices.items():
            t_i = idx[1]
            c_i = idx[2]

            if t_i >= len(pos):
                continue

            thigh = pos[t_i]
            calf = pos[c_i]

            safe_t, safe_c, lims, passive = self.apply_leg_safety(thigh, calf)

            pos[t_i] = safe_t
            pos[c_i] = safe_c

            self.get_logger().info(
                f'{leg}: thigh={thigh:.3f}, calf_in={calf:.3f}, '
                f'passive={passive:.3f}, '
                f'allowed=[{lims[0]:.3f},{lims[1]:.3f}], '
                f'out={safe_c:.3f}'
            )

        out.position = pos
        self.js_pub.publish(out)

    def joint_callback(self, msg: JointTrajectory):
        out = JointTrajectory()
        out.header = msg.header
        out.joint_names = msg.joint_names

        for point in msg.points:
            new = JointTrajectoryPoint()
            new.positions = list(point.positions)

            for leg, idx in self.indices.items():
                t_i = idx[1]
                c_i = idx[2]

                thigh = new.positions[t_i]
                calf = new.positions[c_i]

                safe_t, safe_c, _, _ = self.apply_leg_safety(thigh, calf)

                new.positions[t_i] = safe_t
                new.positions[c_i] = safe_c

            out.points.append(new)

        self.traj_pub.publish(out)


def main():
    rclpy.init()
    node = JaxLegSafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()