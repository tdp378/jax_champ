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
        self.declare_parameter('neutral_thigh_rad', 0.0)

        self.enable_passive_calf = bool(self.get_parameter('enable_passive_calf').value)
        self.neutral_thigh_rad = float(self.get_parameter('neutral_thigh_rad').value)

        # -----------------------------
        # Edge-triggered follow
        # -----------------------------
        self.declare_parameter('enable_edge_follow', True)

        # How close to the envelope edge before follow activates
        # 0.10 = within 10% of the edge
        self.declare_parameter('open_follow_trigger_fraction', 0.10)
        self.declare_parameter('closed_follow_trigger_fraction', 0.10)

        # Extra calf follow per radian of thigh motion once triggered
        self.declare_parameter('open_follow_gain', 1.0)
        self.declare_parameter('closed_follow_gain', 1.0)

        # Optional safety: if slider teleports a lot, skip extra follow for that step
        self.declare_parameter('reset_follow_on_large_jump', True)
        self.declare_parameter('large_jump_threshold_rad', 0.35)

        self.enable_edge_follow = bool(self.get_parameter('enable_edge_follow').value)
        self.open_follow_trigger_fraction = float(self.get_parameter('open_follow_trigger_fraction').value)
        self.closed_follow_trigger_fraction = float(self.get_parameter('closed_follow_trigger_fraction').value)
        self.open_follow_gain = float(self.get_parameter('open_follow_gain').value)
        self.closed_follow_gain = float(self.get_parameter('closed_follow_gain').value)
        self.reset_follow_on_large_jump = bool(self.get_parameter('reset_follow_on_large_jump').value)
        self.large_jump_threshold_rad = float(self.get_parameter('large_jump_threshold_rad').value)

        # -----------------------------
        # Logging
        # -----------------------------
        self.declare_parameter('warn_on_clamp', True)
        self.declare_parameter('debug_logs', True)

        self.warn_on_clamp = bool(self.get_parameter('warn_on_clamp').value)
        self.debug_logs = bool(self.get_parameter('debug_logs').value)

        # ============================================================
        # SAFETY ENVELOPE (RADIANS)
        # (upper_rad, lower_min_rad, lower_max_rad)
        #
        # Replace this with your live tuned table if different.
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

        # ============================================================
        # PASSIVE CURVE (RADIANS)
        # (upper_rad, passive_lower_rad)
        #
        # Replace this with your live tuned table if different.
        # ============================================================
        self.passive_table_rad: List[Tuple[float, float]] = [
            (-1.40,  0.733),
            ( 0.000,  0.000),
            ( 0.900, -1.000),
        ]

        # Previous effective thigh per leg, used only for motion direction
        self.prev_state: Dict[str, Dict[str, float]] = {}

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

        self.get_logger().info('Jax linkage envelope node started')

    # ============================================================
    # HELPERS
    # ============================================================

    def clamp(self, val: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, val))

    def lerp(self, x: float, x0: float, y0: float, x1: float, y1: float) -> float:
        if abs(x1 - x0) < 1e-9:
            return y0
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0)

    def interpolate_limits_rad(self, upper: float) -> Tuple[float, float, float]:
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

    def interpolate_passive_rad(self, upper: float) -> float:
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

    def apply_safety_margins_rad(self, lower_min: float, lower_max: float) -> Tuple[float, float]:
        safe_min = lower_min + self.min_margin_rad
        safe_max = lower_max - self.max_margin_rad

        if safe_min > safe_max:
            mid = 0.5 * (safe_min + safe_max)
            return mid, mid

        return safe_min, safe_max

    def envelope_norm(self, calf_eff: float, low_min: float, low_max: float) -> float:
        width = max(low_max - low_min, 1e-6)
        norm = (calf_eff - low_min) / width
        return self.clamp(norm, 0.0, 1.0)

    # ============================================================
    # CORE LOGIC
    # ============================================================

    def apply_leg_safety(self, leg_name: str, thigh: float, calf: float):
        thigh_eff = thigh * self.thigh_direction_sign
        calf_eff = calf * self.calf_direction_sign

        # Envelope for current thigh
        safe_thigh, low_min, low_max = self.interpolate_limits_rad(thigh_eff)
        low_min, low_max = self.apply_safety_margins_rad(low_min, low_max)

        # Passive linkage movement always applies
        passive_delta = 0.0
        if self.enable_passive_calf:
            passive_here = self.interpolate_passive_rad(safe_thigh)
            passive_neutral = self.interpolate_passive_rad(self.neutral_thigh_rad)
            passive_delta = passive_here - passive_neutral

        requested_calf = calf_eff + passive_delta

        # Edge-triggered follow
        extra_follow = 0.0

        norm_pos = self.envelope_norm(requested_calf, low_min, low_max)

        near_open = norm_pos >= (1.0 - self.open_follow_trigger_fraction)
        near_closed = norm_pos <= self.closed_follow_trigger_fraction

        state_reset = False
        dthigh = 0.0

        if self.enable_edge_follow:
            prev = self.prev_state.get(leg_name, None)

            if prev is not None:
                prev_thigh_eff = prev['thigh_eff']
                dthigh = safe_thigh - prev_thigh_eff

                if self.reset_follow_on_large_jump and abs(dthigh) > self.large_jump_threshold_rad:
                    state_reset = True
                else:
                    # Backward / rearward motion: thigh increasing
                    # If already near closed edge, add more closing
                    if dthigh > 0.0 and near_closed:
                        extra_follow = -dthigh * self.closed_follow_gain

                    # Forward motion: thigh decreasing
                    # If already near open edge, add more opening
                    elif dthigh < 0.0 and near_open:
                        extra_follow = -dthigh * self.open_follow_gain

                    requested_calf = requested_calf + extra_follow

        # Final clamp into safe envelope
        safe_calf = self.clamp(requested_calf, low_min, low_max)

        # Save safe effective state for next step
        self.prev_state[leg_name] = {
            'thigh_eff': safe_thigh,
            'safe_calf_eff': safe_calf,
        }

        # Convert back to ROS convention
        safe_thigh_ros = safe_thigh / self.thigh_direction_sign
        safe_calf_ros = safe_calf / self.calf_direction_sign

        return (
            safe_thigh_ros,
            safe_calf_ros,
            (low_min, low_max),
            passive_delta,
            norm_pos,
            near_open,
            near_closed,
            dthigh,
            extra_follow,
            state_reset,
        )

    # ============================================================
    # CALLBACKS
    # ============================================================

    def js_callback(self, msg: JointState):
        out = copy.deepcopy(msg)
        pos = list(out.position)

        for leg, idx in self.indices.items():
            t_i = idx[1]
            c_i = idx[2]

            if t_i >= len(pos) or c_i >= len(pos):
                continue

            thigh = pos[t_i]
            calf = pos[c_i]

            (
                safe_t,
                safe_c,
                lims,
                passive,
                norm_pos,
                near_open,
                near_closed,
                dthigh,
                extra_follow,
                state_reset,
            ) = self.apply_leg_safety(leg, thigh, calf)

            changed = abs(safe_t - thigh) > 1e-6 or abs(safe_c - calf) > 1e-6

            pos[t_i] = safe_t
            pos[c_i] = safe_c

            if self.debug_logs:
                self.get_logger().info(
                    f'{leg}: thigh={thigh:.3f}, calf_in={calf:.3f}, '
                    f'passive={passive:.3f}, '
                    f'norm={norm_pos:.3f}, '
                    f'near_open={near_open}, near_closed={near_closed}, '
                    f'dthigh={dthigh:.3f}, extra_follow={extra_follow:.3f}, '
                    f'reset={state_reset}, '
                    f'allowed=[{lims[0]:.3f},{lims[1]:.3f}], '
                    f'out={safe_c:.3f}'
                )

            if self.warn_on_clamp and changed:
                self.get_logger().warning(
                    f'{leg}: adjusted js thigh={thigh:.3f}->{safe_t:.3f}, '
                    f'calf={calf:.3f}->{safe_c:.3f}'
                )

        out.position = pos
        self.js_pub.publish(out)

    def joint_callback(self, msg: JointTrajectory):
        out = JointTrajectory()
        out.header = msg.header
        out.joint_names = msg.joint_names

        for point in msg.points:
            new = JointTrajectoryPoint()
            new.time_from_start = point.time_from_start
            new.positions = list(point.positions)
            new.velocities = list(point.velocities)
            new.accelerations = list(point.accelerations)
            new.effort = list(point.effort)

            for leg, idx in self.indices.items():
                t_i = idx[1]
                c_i = idx[2]

                if t_i >= len(new.positions) or c_i >= len(new.positions):
                    continue

                thigh = new.positions[t_i]
                calf = new.positions[c_i]

                safe_t, safe_c, _, _, _, _, _, _, _, _ = self.apply_leg_safety(leg, thigh, calf)

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