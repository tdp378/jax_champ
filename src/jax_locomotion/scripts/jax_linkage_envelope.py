#!/usr/bin/env python3

import copy
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JaxLinkageEnvelopeNode(Node):

    def __init__(self) -> None:
        super().__init__('jax_linkage_envelope_node')

        # ============================================================
        # 🔧🔧🔧 TUNABLE PARAMETERS (ALL IN ONE PLACE) 🔧🔧🔧
        # ============================================================

        # ---- Direction (keep calf = 1.0 so RViz matches table) ----
        self.thigh_direction_sign = float(
            self.declare_parameter('thigh_direction_sign', 1.0).value
        )
        self.calf_direction_sign = float(
            self.declare_parameter('calf_direction_sign', 1.0).value
        )

        # ---- Follow direction (flip these if wrong direction) ----
        self.follow_direction = float(
            self.declare_parameter('follow_direction', 1.0).value
        )
        self.open_follow_sign = float(
            self.declare_parameter('open_follow_sign', 1.0).value
        )
        self.closed_follow_sign = float(
            self.declare_parameter('closed_follow_sign', 1.0).value
        )

        # ---- Follow gains ----
        self.open_follow_gain = float(
            self.declare_parameter('open_follow_gain', 1.0).value
        )
        self.closed_follow_gain = float(
            self.declare_parameter('closed_follow_gain', 1.0).value
        )

        # ---- Follow thresholds ----
        self.rear_follow_start_rad = float(
            self.declare_parameter('rear_follow_start_rad', 0.75).value
        )
        self.front_follow_start_rad = float(
            self.declare_parameter('front_follow_start_rad', -0.55).value
        )

        # ---- Edge detection ----
        self.open_follow_trigger_fraction = float(
            self.declare_parameter('open_follow_trigger_fraction', 0.10).value
        )
        self.closed_follow_trigger_fraction = float(
            self.declare_parameter('closed_follow_trigger_fraction', 0.10).value
        )
        self.early_follow_edge_fraction = float(
            self.declare_parameter('early_follow_edge_fraction', 0.02).value
        )

        # ---- Safety margins ----
        self.min_margin_rad = float(
            self.declare_parameter('min_margin_rad', 0.0).value
        )
        self.max_margin_rad = float(
            self.declare_parameter('max_margin_rad', 0.0).value
        )

        # ---- Passive behavior ----
        self.enable_passive_calf = bool(
            self.declare_parameter('enable_passive_calf', True).value
        )
        self.neutral_thigh_rad = float(
            self.declare_parameter('neutral_thigh_rad', 0.0).value
        )

        # ---- Debug ----
        self.debug_logs = bool(
            self.declare_parameter('debug_logs', False).value
        )
        self.warn_on_clamp = bool(
            self.declare_parameter('warn_on_clamp', True).value
        )

        # ============================================================
        # SAFETY ENVELOPE (RADIANS)
        # ============================================================
        self.table_rad: List[Tuple[float, float, float]] = [
            (-1.40,  0.733,  0.733),
            (-1.00,  0.000,  0.733),
            (-0.70, -0.700,  0.733),
            (-0.60, -0.800,  0.733),
            (-0.50, -0.900,  0.733),
            (-0.40, -1.000,  0.733),
            (-0.30, -1.000,  0.733),
            (-0.20, -1.000,  0.733),
            (-0.10, -1.000,  0.733),
            ( 0.00, -1.000,  0.733),
            ( 0.30, -1.000,  0.500),
            ( 0.60, -1.000,  0.150),
            ( 0.80, -1.000, -0.150),
            ( 0.90, -1.000, -0.300),
            ( 1.047, -1.000, -0.524),
        ]

        # Passive curve (for visualization/debug)
        self.passive_table_rad: List[Tuple[float, float]] = [
            (-1.40,  0.733),
            ( 0.00,  0.000),
            ( 0.90, -1.000),
        ]

        # ============================================================
        # Joint indices
        # ============================================================
        self.indices = {
            'lf': [0, 1, 2],
            'rf': [3, 4, 5],
            'lh': [6, 7, 8],
            'rh': [9, 10, 11],
        }

        # State
        self.prev_state: Dict[str, Dict[str, float]] = {}

        # ROS I/O
        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/jax/walk_joint_trajectory_raw',
            self.joint_callback,
            10,
        )

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_group_effort_controller/joint_trajectory',
            10,
        )

        self.js_sub = self.create_subscription(
            JointState,
            '/joint_states_raw',
            self.js_callback,
            10,
        )

        self.js_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10,
        )

        self.get_logger().info(
            'Jax linkage envelope node started '
            f'(follow_direction={self.follow_direction}, '
            f'open_follow_sign={self.open_follow_sign}, '
            f'closed_follow_sign={self.closed_follow_sign})'
        )
        self.get_logger().info(
            'Effective follow signs '
            f'(open={self.follow_direction * self.open_follow_sign}, '
            f'closed={self.follow_direction * self.closed_follow_sign})'
        )

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

    def envelope_norm(self, calf, low_min, low_max):
        width = max(low_max - low_min, 1e-6)
        return self.clamp((calf - low_min) / width, 0.0, 1.0)

    # ============================================================
    # CORE LOGIC
    # ============================================================

    def apply_leg_safety(self, leg, thigh, calf):

        thigh_eff = thigh * self.thigh_direction_sign
        calf_eff = calf * self.calf_direction_sign

        safe_thigh, low_min, low_max = self.interpolate_limits_rad(thigh_eff)

        # ---- CLAMP ----
        # Keep all follow math in joint command space so follow_direction maps
        # directly to observed motor direction, independent of calf sign mapping.
        low_cmd = low_min / self.calf_direction_sign
        high_cmd = low_max / self.calf_direction_sign
        if low_cmd > high_cmd:
            low_cmd, high_cmd = high_cmd, low_cmd

        # Start from previous safe calf (if available) so follow is not reset
        # each tick by a raw slider command outside the envelope.
        base_calf = self.clamp(calf, low_cmd, high_cmd)
        prev = self.prev_state.get(leg)
        if prev and 'calf' in prev:
            requested_calf = self.clamp(prev['calf'], low_cmd, high_cmd)
        else:
            requested_calf = base_calf
        extra_follow = 0.0

        raw_norm = self.envelope_norm(calf_eff, low_min, low_max)

        really_open = raw_norm >= (1.0 - self.early_follow_edge_fraction)
        really_closed = raw_norm <= self.early_follow_edge_fraction

        if prev:
            # Use effective (post-sign) thigh motion for coupling.
            prev_thigh_eff = prev.get('thigh_eff', safe_thigh)
            dthigh_eff = safe_thigh - prev_thigh_eff

            rear_allowed = safe_thigh >= self.rear_follow_start_rad or really_closed
            front_allowed = safe_thigh <= self.front_follow_start_rad or really_open

            open_sign = self.follow_direction * self.open_follow_sign
            closed_sign = self.follow_direction * self.closed_follow_sign

            if dthigh_eff < 0.0 and rear_allowed:
                # Opposite-of-thigh in effective space, then convert to
                # command-space through calf_direction_sign.
                extra_follow = (
                    -closed_sign * dthigh_eff * self.closed_follow_gain
                ) / self.calf_direction_sign

            elif dthigh_eff > 0.0 and front_allowed:
                # Opposite-of-thigh in effective space, then convert to
                # command-space through calf_direction_sign.
                extra_follow = (
                    -open_sign * dthigh_eff * self.open_follow_gain
                ) / self.calf_direction_sign

            requested_calf += extra_follow

        safe_calf = self.clamp(requested_calf, low_cmd, high_cmd)

        self.prev_state[leg] = {
            'thigh_eff': safe_thigh,
            'calf': safe_calf,
        }

        return (
            safe_thigh,
            safe_calf
        )

    # ============================================================
    # CALLBACKS
    # ============================================================

    def js_callback(self, msg):
        out = copy.deepcopy(msg)
        pos = list(out.position)

        for leg, idx in self.indices.items():
            t_i, c_i = idx[1], idx[2]

            if t_i >= len(pos) or c_i >= len(pos):
                continue

            t, c = pos[t_i], pos[c_i]
            safe_t, safe_c = self.apply_leg_safety(leg, t, c)

            pos[t_i] = safe_t
            pos[c_i] = safe_c

        out.position = pos
        self.js_pub.publish(out)

    def joint_callback(self, msg):
        out = JointTrajectory()
        out.header = msg.header
        out.joint_names = msg.joint_names

        for point in msg.points:
            new = JointTrajectoryPoint()
            new.positions = list(point.positions)

            for leg, idx in self.indices.items():
                t_i, c_i = idx[1], idx[2]

                if t_i >= len(new.positions) or c_i >= len(new.positions):
                    continue

                t, c = new.positions[t_i], new.positions[c_i]
                safe_t, safe_c = self.apply_leg_safety(leg, t, c)

                new.positions[t_i] = safe_t
                new.positions[c_i] = safe_c

            out.points.append(new)

        self.traj_pub.publish(out)


def main():
    rclpy.init()
    node = JaxLinkageEnvelopeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()