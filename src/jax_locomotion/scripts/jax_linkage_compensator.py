#!/usr/bin/env python3

"""
Jax Linkage Compensator
=======================

Two-path model:
1) Motor command path (JointTrajectory):
   - Inside passive zone [neutral_min, neutral_max], calf motor command is unchanged.
   - Outside passive zone, apply corrective follow and clamp to thigh-dependent limits.

2) Display path (JointState for RViz):
   - Uses display_neutral_min/max so RViz can show full passive linkage motion
     independent from the motor passive zone.
"""

import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory


# Per-leg: (thigh_idx, calf_idx) within 12-element layout
LEG_JOINT_INDICES = {
    'FL': (1, 2),
    'FR': (4, 5),
    'BL': (7, 8),
    'BR': (10, 11),
}
LEG_ORDER = ['FL', 'FR', 'BL', 'BR']

_LEG_PREFIX = {'FL': 'lf', 'FR': 'rf', 'BL': 'lh', 'BR': 'rh'}
_JOINT_KEYWORD = {
    'thigh': ['upper_leg', 'upper', 'thigh'],
    'calf': ['lower_leg', 'lower', 'calf'],
}


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _lerp(start: float, end: float, t: float) -> float:
    return start + (end - start) * t


def compute_linkage_correction(thigh_angle: float,
                               neutral_min: float,
                               neutral_max: float,
                               linkage_ratio: float,
                               max_correction: float) -> float:
    """
    Piecewise correction:
      - no correction in [neutral_min, neutral_max]
      - linear correction outside that zone

    Sign convention in this codebase:
      - forward thigh overshoot (below neutral_min) -> positive correction
      - backward thigh overshoot (above neutral_max) -> negative correction
    """
    if thigh_angle < neutral_min:
        delta = thigh_angle - neutral_min
    elif thigh_angle > neutral_max:
        delta = thigh_angle - neutral_max
    else:
        delta = 0.0

    correction = -delta * linkage_ratio
    return _clamp(correction, -max_correction, max_correction)


class JaxLinkageCompensator(Node):
    def __init__(self):
        super().__init__('jax_linkage_compensator')

        # Motor passive zone (robot/sim actuation path)
        self.declare_parameter('neutral_min', -0.35)
        self.declare_parameter('neutral_max', 0.90)

        # Display-only neutral zone (rviz path)
        self.declare_parameter('display_neutral_min', 0.0)
        self.declare_parameter('display_neutral_max', 0.0)

        self.declare_parameter('linkage_ratio', 1.0)
        self.declare_parameter('max_correction', 0.90)

        self.declare_parameter('thigh_backward_limit', 0.90)
        self.declare_parameter('thigh_forward_limit', -0.35)
        self.declare_parameter('calf_min_angle', -0.90)
        self.declare_parameter('calf_max_angle', 0.90)
        self.declare_parameter('forward_calf_window', 0.10)

        self.declare_parameter('enabled', True)
        self.declare_parameter('publish_diagnostics', True)

        self.neutral_min = float(self.get_parameter('neutral_min').value)
        self.neutral_max = float(self.get_parameter('neutral_max').value)
        self.display_neutral_min = float(self.get_parameter('display_neutral_min').value)
        self.display_neutral_max = float(self.get_parameter('display_neutral_max').value)

        self.linkage_ratio = float(self.get_parameter('linkage_ratio').value)
        self.max_correction = float(self.get_parameter('max_correction').value)

        self.thigh_backward_limit = float(self.get_parameter('thigh_backward_limit').value)
        self.thigh_forward_limit = float(self.get_parameter('thigh_forward_limit').value)
        self.calf_min_angle = float(self.get_parameter('calf_min_angle').value)
        self.calf_max_angle = float(self.get_parameter('calf_max_angle').value)
        self.forward_calf_window = float(self.get_parameter('forward_calf_window').value)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.publish_diagnostics = bool(self.get_parameter('publish_diagnostics').value)

        self.joint_cmd_sub = self.create_subscription(
            JointTrajectory,
            '/joint_group_position_controller/command',
            self.joint_cmd_callback,
            10,
        )
        self.corrected_pub = self.create_publisher(
            JointTrajectory,
            '/jax/joint_commands/linkage_corrected',
            10,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states_raw',
            self.joint_state_callback,
            10,
        )
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        if self.publish_diagnostics:
            self.diag_pub = self.create_publisher(
                Float32MultiArray,
                '/jax/linkage_compensator/diagnostics',
                10,
            )

        self.get_logger().info(
            f"Jax Linkage Compensator started\n"
            f"  Motor passive zone : [{self.neutral_min:.3f}, {self.neutral_max:.3f}] rad\n"
            f"  Display neutral    : [{self.display_neutral_min:.3f}, {self.display_neutral_max:.3f}] rad\n"
            f"  Linkage ratio      : {self.linkage_ratio:.3f}\n"
            f"  Max correction     : {self.max_correction:.3f} rad\n"
            f"  Thigh limits       : [{self.thigh_forward_limit:.3f}, {self.thigh_backward_limit:.3f}] rad\n"
            f"  Calf limits        : [{self.calf_min_angle:.3f}, {self.calf_max_angle:.3f}] rad\n"
            f"  Forward calf window: {self.forward_calf_window:.3f} rad\n"
            f"  Enabled            : {self.enabled}"
        )

    def _compute_calf_limits(self, thigh_angle: float, linkage_calf: float):
        """
        Thigh-dependent calf envelope:
          - Backward side: progressively remove extra closing room.
          - Forward side: progressively tighten to a small window.
        """
        min_calf = self.calf_min_angle
        max_calf = self.calf_max_angle

        if self.thigh_backward_limit > 0.0 and thigh_angle > 0.0:
            back_progress = _clamp(thigh_angle / self.thigh_backward_limit, 0.0, 1.0)
            min_calf = _lerp(self.calf_min_angle, linkage_calf, back_progress)

        if self.thigh_forward_limit < 0.0 and thigh_angle < 0.0:
            forward_progress = _clamp(thigh_angle / self.thigh_forward_limit, 0.0, 1.0)
            forward_min = max(self.calf_min_angle, linkage_calf - self.forward_calf_window)
            forward_max = min(self.calf_max_angle, linkage_calf + self.forward_calf_window)
            min_calf = _lerp(self.calf_min_angle, forward_min, forward_progress)
            max_calf = _lerp(self.calf_max_angle, forward_max, forward_progress)

        if min_calf > max_calf:
            mid = 0.5 * (min_calf + max_calf)
            min_calf = mid
            max_calf = mid

        return min_calf, max_calf

    def _apply_motor_model(self, raw_calf: float, thigh_angle: float):
        """
        Motor-command path: inside passive zone, leave calf command unchanged.
        """
        correction = compute_linkage_correction(
            thigh_angle,
            self.neutral_min,
            self.neutral_max,
            self.linkage_ratio,
            self.max_correction,
        )

        if abs(correction) < 1e-9:
            # Hard guarantee: no compensator-induced calf motion in passive zone.
            return raw_calf, 0.0, self.calf_min_angle, self.calf_max_angle

        corrected_calf = raw_calf + correction
        min_calf, max_calf = self._compute_calf_limits(thigh_angle, correction)
        corrected_calf = _clamp(corrected_calf, min_calf, max_calf)
        return corrected_calf, correction, min_calf, max_calf

    def _apply_display_model(self, raw_calf: float, thigh_angle: float):
        """
        Display path: shows passive linkage behavior using display neutral zone.
        """
        correction = compute_linkage_correction(
            thigh_angle,
            self.display_neutral_min,
            self.display_neutral_max,
            self.linkage_ratio,
            self.max_correction,
        )
        corrected_calf = raw_calf + correction
        min_calf, max_calf = self._compute_calf_limits(thigh_angle, correction)
        corrected_calf = _clamp(corrected_calf, min_calf, max_calf)
        return corrected_calf, correction, min_calf, max_calf

    def _find_joint_name(self, name_to_idx: dict, leg: str,
                         joint_type: str, fallback_idx: int) -> str:
        prefix = _LEG_PREFIX.get(leg, leg.lower())
        for kw in _JOINT_KEYWORD.get(joint_type, [joint_type]):
            for name in (
                f"{prefix}_{kw}_joint",
                f"{prefix}_{kw}",
                f"{leg}_{kw}_joint",
                f"{leg}_{kw}",
            ):
                if name in name_to_idx:
                    return name

        for name in (
            f"{leg}_{joint_type}",
            f"{leg}_{joint_type}_joint",
            f"{leg.lower()}_{joint_type}",
            f"{leg.lower()}_{joint_type}_joint",
        ):
            if name in name_to_idx:
                return name

        names = list(name_to_idx.keys())
        if fallback_idx < len(names):
            return names[fallback_idx]
        return ""

    def joint_cmd_callback(self, msg: JointTrajectory):
        if not self.enabled or not msg.points:
            self.corrected_pub.publish(msg)
            return

        corrected_msg = JointTrajectory()
        corrected_msg.header = msg.header
        corrected_msg.joint_names = msg.joint_names

        name_to_idx = {name: i for i, name in enumerate(msg.joint_names)}
        diag_data = []

        for point in msg.points:
            corrected_point = copy.deepcopy(point)

            for leg in LEG_ORDER:
                thigh_idx, calf_idx = LEG_JOINT_INDICES[leg]
                thigh_name = self._find_joint_name(name_to_idx, leg, 'thigh', thigh_idx)
                calf_name = self._find_joint_name(name_to_idx, leg, 'calf', calf_idx)

                t_idx = name_to_idx.get(thigh_name, thigh_idx)
                c_idx = name_to_idx.get(calf_name, calf_idx)

                if t_idx >= len(corrected_point.positions) or c_idx >= len(corrected_point.positions):
                    continue

                thigh_angle = corrected_point.positions[t_idx]
                raw_calf = corrected_point.positions[c_idx]
                corrected_calf, correction, min_calf, max_calf = self._apply_motor_model(
                    raw_calf,
                    thigh_angle,
                )
                corrected_point.positions[c_idx] = corrected_calf

                if self.publish_diagnostics:
                    diag_data.extend([
                        float(thigh_angle),
                        float(raw_calf),
                        float(correction),
                        float(corrected_calf),
                        float(min_calf),
                        float(max_calf),
                    ])

            corrected_msg.points.append(corrected_point)

        self.corrected_pub.publish(corrected_msg)

        if self.publish_diagnostics and diag_data and hasattr(self, 'diag_pub'):
            diag_msg = Float32MultiArray()
            diag_msg.data = diag_data
            self.diag_pub.publish(diag_msg)

    def joint_state_callback(self, msg: JointState):
        if not self.enabled or not msg.position:
            self.joint_state_pub.publish(msg)
            return

        corrected = copy.deepcopy(msg)
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        pos = list(corrected.position)

        diag_data = []

        for leg in LEG_ORDER:
            thigh_idx_nom, calf_idx_nom = LEG_JOINT_INDICES[leg]
            thigh_name = self._find_joint_name(name_to_idx, leg, 'thigh', thigh_idx_nom)
            calf_name = self._find_joint_name(name_to_idx, leg, 'calf', calf_idx_nom)

            t_idx = name_to_idx.get(thigh_name)
            c_idx = name_to_idx.get(calf_name)

            if t_idx is None or c_idx is None:
                continue
            if t_idx >= len(pos) or c_idx >= len(pos):
                continue

            thigh_angle = pos[t_idx]
            raw_calf = pos[c_idx]
            corrected_calf, correction, min_calf, max_calf = self._apply_display_model(
                raw_calf,
                thigh_angle,
            )
            pos[c_idx] = corrected_calf

            if self.publish_diagnostics:
                diag_data.extend([
                    float(thigh_angle),
                    float(raw_calf),
                    float(correction),
                    float(corrected_calf),
                    float(min_calf),
                    float(max_calf),
                ])

        corrected.position = pos
        self.joint_state_pub.publish(corrected)

        if self.publish_diagnostics and diag_data and hasattr(self, 'diag_pub'):
            diag_msg = Float32MultiArray()
            diag_msg.data = diag_data
            self.diag_pub.publish(diag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JaxLinkageCompensator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
