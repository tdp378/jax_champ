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

import math
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32MultiArray


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
        - Thigh forward (positive overshoot): linkage pulls calf OPEN -> positive correction
        - Thigh backward (negative overshoot): linkage pushes calf CLOSED -> negative correction
    """
    if thigh_angle < neutral_min:
        delta = thigh_angle - neutral_min          # negative value
    elif thigh_angle > neutral_max:
        delta = thigh_angle - neutral_max          # positive value
    else:
        delta = 0.0

    correction = delta * linkage_ratio

    # Safety clamp
    correction = max(-max_correction, min(max_correction, correction))
    return correction


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
        self.declare_parameter('enabled',            True)
        self.declare_parameter('publish_diagnostics', True)

        self.neutral_min       = float(self.get_parameter('neutral_min').value)
        self.neutral_max       = float(self.get_parameter('neutral_max').value)
        self.linkage_ratio     = float(self.get_parameter('linkage_ratio').value)
        self.max_correction    = float(self.get_parameter('max_correction').value)
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
            f"  Neutral zone : [{self.neutral_min:.3f}, {self.neutral_max:.3f}] rad\n"
            f"  Linkage ratio: {self.linkage_ratio:.3f} rad/rad\n"
            f"  Max correction: {self.max_correction:.3f} rad\n"
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
            import copy
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

                corrected_point.positions[c_idx] += correction

                if self.publish_diagnostics:
                    diag_data.extend([float(thigh_angle), float(correction)])

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

    def _find_joint_name(self, name_to_idx: dict, leg: str,
                         joint_type: str, fallback_idx: int) -> str:
        """
        Attempt to find the joint name for a given leg and joint type
        (thigh or calf) by scanning the joint name list.

        Supports common naming patterns:
            {leg}_{type}   e.g. FL_thigh, FL_calf
            {leg}_{type}_joint
        """
        candidates = [
            f"{leg}_{joint_type}",
            f"{leg}_{joint_type}_joint",
            f"{leg.lower()}_{joint_type}",
            f"{leg.lower()}_{joint_type}_joint",
        ]
        for c in candidates:
            if c in name_to_idx:
                return c

        # Fallback: return the key at the numeric index position
        names = list(name_to_idx.keys())
        if fallback_idx < len(names):
            return names[fallback_idx]
        return ""


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