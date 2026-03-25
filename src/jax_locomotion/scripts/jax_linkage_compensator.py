#!/usr/bin/env python3

"""
Jax Linkage Compensator
========================

Corrects the calf (knee) joint commands for the mechanical linkage
that connects the hip motor to the calf via a T-horn and long/short
rod assembly.

Mechanical summary:
  - The hip motor drives a SHORT arm on the T-horn pivot.
  - A LONG arm on the T-horn drives a rod to the BACK of the calf.
  - All four knees are reversed (>> orientation).
  - Within a neutral thigh angle range, the linkage geometry is
    slack enough that the calf angle is unaffected.
  - Outside that range, the calf is forced open (thigh forward)
    or closed/crouched (thigh backward) proportionally.

Correction formula (applied per leg):
    delta = thigh_angle - neutral_min   if thigh_angle < neutral_min
    delta = thigh_angle - neutral_max   if thigh_angle > neutral_max
    delta = 0                           otherwise

    calf_correction = delta * linkage_ratio

    corrected_calf = raw_calf + calf_correction

Because the knees are reversed (>>), a positive correction OPENS
the calf (increases calf joint angle away from body).

Pipeline position:
    CHAMP output  -->  [this node]  -->  /jax/joint_commands/linkage_corrected
                                              --> future layers --> hardware

Subscribes:
    /joint_group_position_controller/command  (JointTrajectory)

Publishes:
    /jax/joint_commands/linkage_corrected     (JointTrajectory)
    /jax/linkage_compensator/diagnostics      (Float32MultiArray, optional)

Parameters (same value applied to all 4 legs unless per-leg override added):
    neutral_min       (float, rad)  Lower bound of neutral thigh range. Default: -0.3
    neutral_max       (float, rad)  Upper bound of neutral thigh range. Default:  0.3
    linkage_ratio     (float)       Calf correction per radian of thigh overshoot. Default: 0.5
    max_correction    (float, rad)  Safety clamp on calf correction. Default: 0.8
    enabled           (bool)        Pass-through when False. Default: True
    publish_diagnostics (bool)      Publish per-leg debug data. Default: True

Joint name convention (must match URDF/CHAMP config):
    Joints are expected in order: [hip, thigh, calf] x 4 legs
    Index layout in the 12-element array:
        0  FL_hip    1  FL_thigh    2  FL_calf
        3  FR_hip    4  FR_thigh    5  FR_calf
        6  BL_hip    7  BL_thigh    8  BL_calf
        9  BR_hip   10  BR_thigh   11  BR_calf
"""

import copy
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory


# ---------------------------------------------------------------------------
# Index helpers — matches CHAMP's standard 12-joint layout
# ---------------------------------------------------------------------------

# Per-leg: (thigh_idx, calf_idx) within the 12-element positions array
LEG_JOINT_INDICES = {
    'FL': (1, 2),
    'FR': (4, 5),
    'BL': (7, 8),
    'BR': (10, 11),
}

LEG_ORDER = ['FL', 'FR', 'BL', 'BR']

# Maps CHAMP leg labels (FL/FR/BL/BR) to their URDF prefixes (lf/rf/lh/rh)
_LEG_PREFIX = {'FL': 'lf', 'FR': 'rf', 'BL': 'lh', 'BR': 'rh'}

# Maps CHAMP joint-type labels to URDF keyword synonyms
_JOINT_KEYWORD = {
    'thigh': ['upper_leg', 'upper', 'thigh'],
    'calf':  ['lower_leg', 'lower', 'calf'],
}


def compute_linkage_correction(thigh_angle: float,
                                neutral_min: float,
                                neutral_max: float,
                                linkage_ratio: float,
                                max_correction: float) -> float:
    """
    Compute the calf angle correction due to linkage geometry.

    Args:
        thigh_angle:    Current thigh joint position (rad).
        neutral_min:    Lower bound of neutral zone (rad). No correction inside.
        neutral_max:    Upper bound of neutral zone (rad). No correction inside.
        linkage_ratio:  Calf correction (rad) per radian of thigh overshoot.
        max_correction: Absolute clamp on the output correction (rad).

    Returns:
        Calf correction in radians. Positive = calf opens (>> knee orientation).

    Geometry:
        - Thigh forward (positive overshoot): linkage pulls calf OPEN  -> negative correction
        - Thigh backward (negative overshoot): linkage pushes calf CLOSED -> positive correction
    """
    if thigh_angle < neutral_min:
        delta = thigh_angle - neutral_min          # negative value
    elif thigh_angle > neutral_max:
        delta = thigh_angle - neutral_max          # positive value
    else:
        delta = 0.0

    correction = -delta * linkage_ratio

    # Safety clamp
    correction = max(-max_correction, min(max_correction, correction))
    return correction


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _lerp(start: float, end: float, t: float) -> float:
    return start + (end - start) * t


class JaxLinkageCompensator(Node):

    def __init__(self):
        super().__init__('jax_linkage_compensator')

        # ----------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------
        self.declare_parameter('neutral_min',       -0.3)   # rad
        self.declare_parameter('neutral_max',        0.3)   # rad
        self.declare_parameter('linkage_ratio',      0.5)   # rad/rad
        self.declare_parameter('max_correction',     0.8)   # rad
        # Display neutral zone for the RViz JointState path (default 0/0 = always
        # show full passive calf motion regardless of motor passive zone)
        self.declare_parameter('display_neutral_min', 0.0)
        self.declare_parameter('display_neutral_max', 0.0)
        self.declare_parameter('thigh_backward_limit', 0.9)  # rad
        self.declare_parameter('thigh_forward_limit', -0.9)  # rad
        self.declare_parameter('calf_min_angle',     -0.9)  # rad
        self.declare_parameter('calf_max_angle',      0.9)  # rad
        self.declare_parameter('forward_calf_window', 0.1)  # rad
        self.declare_parameter('enabled',            True)
        self.declare_parameter('publish_diagnostics', True)

        self.neutral_min       = float(self.get_parameter('neutral_min').value)
        self.neutral_max       = float(self.get_parameter('neutral_max').value)
        self.linkage_ratio     = float(self.get_parameter('linkage_ratio').value)
        self.max_correction    = float(self.get_parameter('max_correction').value)
        self.display_neutral_min = float(self.get_parameter('display_neutral_min').value)
        self.display_neutral_max = float(self.get_parameter('display_neutral_max').value)
        self.thigh_backward_limit = float(self.get_parameter('thigh_backward_limit').value)
        self.thigh_forward_limit = float(self.get_parameter('thigh_forward_limit').value)
        self.calf_min_angle    = float(self.get_parameter('calf_min_angle').value)
        self.calf_max_angle    = float(self.get_parameter('calf_max_angle').value)
        self.forward_calf_window = float(self.get_parameter('forward_calf_window').value)
        self.enabled           = bool(self.get_parameter('enabled').value)
        self.publish_diagnostics = bool(self.get_parameter('publish_diagnostics').value)

        # ----------------------------------------------------------------
        # Subscribers & Publishers
        # ----------------------------------------------------------------

        # Raw CHAMP joint commands come in here
        self.joint_cmd_sub = self.create_subscription(
            JointTrajectory,
            '/joint_group_position_controller/command',
            self.joint_cmd_callback,
            10
        )

        # Corrected commands go out here for the next layer
        self.corrected_pub = self.create_publisher(
            JointTrajectory,
            '/jax/joint_commands/linkage_corrected',
            10
        )

        # JointState pipeline — used by rviz_launch / joint_state_publisher_gui
        # so that moving a thigh slider automatically compensates the calf in RViz
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states_raw',
            self.joint_state_callback,
            10
        )
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        if self.publish_diagnostics:
            # Publishes [FL_thigh, FL_correction, FR_thigh, FR_correction,
            #            BL_thigh, BL_correction, BR_thigh, BR_correction]
            self.diag_pub = self.create_publisher(
                Float32MultiArray,
                '/jax/linkage_compensator/diagnostics',
                10
            )

        self.get_logger().info(
            f"Jax Linkage Compensator started\n"
            f"  Motor passive zone : [{self.neutral_min:.3f}, {self.neutral_max:.3f}] rad\n"
            f"  Display neutral    : [{self.display_neutral_min:.3f}, {self.display_neutral_max:.3f}] rad\n"
            f"  Linkage ratio: {self.linkage_ratio:.3f} rad/rad\n"
            f"  Max correction: {self.max_correction:.3f} rad\n"
            f"  Thigh limits: [{self.thigh_forward_limit:.3f}, {self.thigh_backward_limit:.3f}] rad\n"
            f"  Calf limits: [{self.calf_min_angle:.3f}, {self.calf_max_angle:.3f}] rad\n"
            f"  Forward calf window: {self.forward_calf_window:.3f} rad\n"
            f"  Enabled: {self.enabled}"
        )

    # --------------------------------------------------------------------
    # Callback
    # --------------------------------------------------------------------

    def joint_cmd_callback(self, msg: JointTrajectory):
        """
        Receive raw joint trajectory from CHAMP, apply linkage correction
        to each calf joint, and republish.

        If disabled, the message is forwarded unchanged (pass-through).
        """

        # --- Pass-through when disabled ---
        if not self.enabled:
            self.corrected_pub.publish(msg)
            return

        # --- Expect at least one trajectory point ---
        if not msg.points:
            self.corrected_pub.publish(msg)
            return

        # Work on a copy so we never mutate the incoming message
        corrected_msg = JointTrajectory()
        corrected_msg.header    = msg.header
        corrected_msg.joint_names = msg.joint_names

        # Build a name -> index map for robustness
        # (in case joint order ever changes in the config)
        name_to_idx = {name: i for i, name in enumerate(msg.joint_names)}

        diag_data = []

        for point in msg.points:
            corrected_point = copy.deepcopy(point)

            for leg in LEG_ORDER:
                thigh_idx, calf_idx = LEG_JOINT_INDICES[leg]

                # Resolve actual index from joint name map if possible,
                # otherwise fall back to the fixed layout index
                thigh_name = self._find_joint_name(name_to_idx, leg, 'thigh', thigh_idx)
                calf_name  = self._find_joint_name(name_to_idx, leg, 'calf',  calf_idx)

                t_idx = name_to_idx.get(thigh_name, thigh_idx)
                c_idx = name_to_idx.get(calf_name,  calf_idx)

                if t_idx >= len(corrected_point.positions) or \
                   c_idx >= len(corrected_point.positions):
                    continue  # Guard against malformed messages

                thigh_angle = corrected_point.positions[t_idx]

                correction = compute_linkage_correction(
                    thigh_angle,
                    self.neutral_min,
                    self.neutral_max,
                    self.linkage_ratio,
                    self.max_correction
                )

                corrected_calf = corrected_point.positions[c_idx] + correction
                min_calf, max_calf = self._compute_calf_limits(thigh_angle, correction)
                corrected_point.positions[c_idx] = _clamp(
                    corrected_calf,
                    min_calf,
                    max_calf
                )

                if self.publish_diagnostics:
                    diag_data.extend([
                        float(thigh_angle),
                        float(correction),
                        float(min_calf),
                        float(max_calf),
                    ])

            corrected_msg.points.append(corrected_point)

        self.corrected_pub.publish(corrected_msg)

        # Diagnostics: only publish for the first point to avoid flooding
        if self.publish_diagnostics and diag_data:
            diag_msg = Float32MultiArray()
            diag_msg.data = diag_data
            self.diag_pub.publish(diag_msg)

    # --------------------------------------------------------------------
    # Helpers
    # --------------------------------------------------------------------

    def _compute_calf_limits(self, thigh_angle: float, linkage_calf: float):
        """
        Compute the allowed calf range for a given thigh angle.

        Model:
          - As the thigh moves backward toward +thigh_backward_limit, the calf
            loses closing travel. At the hard stop it may not close past the
            passive linkage position, but it may still open up to calf_max_angle.
          - As the thigh moves forward toward thigh_forward_limit, the linkage
            constrains the calf strongly in both directions. At the hard stop the
            calf is limited to a small window around the passive linkage position.
        """
        min_calf = self.calf_min_angle
        max_calf = self.calf_max_angle

        if self.thigh_backward_limit > 0.0 and thigh_angle > 0.0:
            back_progress = _clamp(thigh_angle / self.thigh_backward_limit, 0.0, 1.0)
            min_calf = _lerp(self.calf_min_angle, linkage_calf, back_progress)

        if self.thigh_forward_limit < 0.0 and thigh_angle < 0.0:
            forward_progress = _clamp(
                thigh_angle / self.thigh_forward_limit,
                0.0,
                1.0,
            )
            forward_min = max(self.calf_min_angle, linkage_calf - self.forward_calf_window)
            forward_max = min(self.calf_max_angle, linkage_calf + self.forward_calf_window)
            min_calf = _lerp(self.calf_min_angle, forward_min, forward_progress)
            max_calf = _lerp(self.calf_max_angle, forward_max, forward_progress)

        if min_calf > max_calf:
            midpoint = 0.5 * (min_calf + max_calf)
            min_calf = midpoint
            max_calf = midpoint

        return min_calf, max_calf

    def _find_joint_name(self, name_to_idx: dict, leg: str,
                         joint_type: str, fallback_idx: int) -> str:
        """
        Attempt to find the joint name for a given leg and joint type
        (thigh or calf) by scanning the joint name list.

        Supports:
          - URDF style:  lf_upper_leg_joint, lf_lower_leg_joint  (lf/rf/lh/rh)
          - CHAMP style: FL_thigh, FL_calf  (FL/FR/BL/BR)
        """
        # URDF-style: lf/rf/lh/rh + upper_leg / lower_leg
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

        # CHAMP-style: FL_thigh, FL_calf, fl_thigh, fl_calf
        for name in (
            f"{leg}_{joint_type}",
            f"{leg}_{joint_type}_joint",
            f"{leg.lower()}_{joint_type}",
            f"{leg.lower()}_{joint_type}_joint",
        ):
            if name in name_to_idx:
                return name

        # Fallback: return the key at the numeric index position
        names = list(name_to_idx.keys())
        if fallback_idx < len(names):
            return names[fallback_idx]
        return ""


    def joint_state_callback(self, msg: JointState):
        """
        Apply linkage compensation to a JointState message.

        Used by the RViz / joint_state_publisher_gui pipeline:
            /joint_states_raw  (from gui)  -->  [this]  -->  /joint_states

        Moving a thigh slider automatically drives the calf to its
        geometrically-correct compensated position in RViz.
        """
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
            calf_name  = self._find_joint_name(name_to_idx, leg, 'calf',  calf_idx_nom)

            t_idx = name_to_idx.get(thigh_name)
            c_idx = name_to_idx.get(calf_name)

            if t_idx is None or c_idx is None:
                continue
            if t_idx >= len(pos) or c_idx >= len(pos):
                continue

            thigh_angle = pos[t_idx]
            correction = compute_linkage_correction(
                thigh_angle,
                self.display_neutral_min,
                self.display_neutral_max,
                self.linkage_ratio,
                self.max_correction
            )
            corrected_calf = pos[c_idx] + correction
            min_calf, max_calf = self._compute_calf_limits(thigh_angle, correction)
            pos[c_idx] = _clamp(corrected_calf, min_calf, max_calf)

            if self.publish_diagnostics:
                diag_data.extend([
                    float(thigh_angle),
                    float(correction),
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