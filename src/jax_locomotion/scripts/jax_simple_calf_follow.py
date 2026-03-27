#!/usr/bin/env python3

import copy
from typing import Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JaxSimpleCalfFollowNode(Node):

    def __init__(self) -> None:
        super().__init__('jax_simple_calf_follow_node')

        self.thigh_direction_sign = float(
            self.declare_parameter('thigh_direction_sign', 1.0).value
        )
        self.calf_direction_sign = float(
            self.declare_parameter('calf_direction_sign', -1.0).value
        )
        self.forward_gain = float(
            self.declare_parameter('forward_gain', 1.0).value
        )
        self.backward_gain = float(
            self.declare_parameter('backward_gain', 1.0).value
        )
        self.debug_logs = bool(
            self.declare_parameter('debug_logs', False).value
        )
        self.pos_follow_start = float(
            self.declare_parameter('positive_follow_start_rad', 0.0).value
        )
        self.neg_follow_start = float(
            self.declare_parameter('negative_follow_start_rad', 0.0).value
        )
        self.calf_thresh_scale = float(
            self.declare_parameter('calf_position_threshold_scale', 0.0).value
        )
        self.forward_taper = float(
            self.declare_parameter('forward_gain_taper', 0.0).value
        )
        self.backward_taper = float(
            self.declare_parameter('backward_gain_taper', 0.0).value
        )

        self.indices = {
            'lf': [0, 1, 2],
            'rf': [3, 4, 5],
            'lh': [6, 7, 8],
            'rh': [9, 10, 11],
        }

        self.prev_state: Dict[str, Dict[str, float]] = {}

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
            'Jax simple calf follow node started '
            f'(thigh_sign={self.thigh_direction_sign}, '
            f'calf_sign={self.calf_direction_sign}, '
            f'fwd_gain={self.forward_gain}, bwd_gain={self.backward_gain}, '
            f'pos_start={self.pos_follow_start}, '
            f'neg_start={self.neg_follow_start}, '
            f'calf_thresh_scale={self.calf_thresh_scale}, '
            f'fwd_taper={self.forward_taper}, bwd_taper={self.backward_taper})'
        )

    def apply_leg_follow(self, leg, thigh, calf):
        thigh_eff = thigh * self.thigh_direction_sign
        calf_eff = calf * self.calf_direction_sign

        # Record thigh origin on first call per leg
        if leg not in self.prev_state:
            self.prev_state[leg] = {'thigh_eff_origin': thigh_eff}

        origin = self.prev_state[leg]['thigh_eff_origin']
        thigh_travel = thigh_eff - origin

        # Use commanded calf position for threshold scaling and gain taper
        calf_offset = abs(calf)
        eff_pos_start = max(0.0, self.pos_follow_start - self.calf_thresh_scale * calf_offset)
        eff_neg_start = max(0.0, self.neg_follow_start - self.calf_thresh_scale * calf_offset)

        # Dead zone
        if thigh_travel > eff_pos_start:
            active_travel = thigh_travel - eff_pos_start
        elif thigh_travel < -eff_neg_start:
            active_travel = thigh_travel + eff_neg_start
        else:
            active_travel = 0.0

        # Select gain and taper based on thigh direction
        # Use signed calf: backward taper scales with -calf (closed=more, open=less)
        #                   forward taper scales with +calf (open=more, closed=less)
        if active_travel > 0.0:
            eff_gain = self.backward_gain * max(0.0, 1.0 + self.backward_taper * (-calf))
        elif active_travel < 0.0:
            eff_gain = self.forward_gain * max(0.0, 1.0 + self.forward_taper * calf)
        else:
            eff_gain = 0.0

        # Direct correction: output = input + follow offset (no accumulation)
        correction_eff = -active_travel * eff_gain
        out_calf_eff = calf_eff + correction_eff

        if self.debug_logs:
            self.get_logger().info(
                f'follow[{leg}] travel={thigh_travel:.4f} '
                f'eff_pos={eff_pos_start:.3f} eff_neg={eff_neg_start:.3f} '
                f'active={active_travel:.4f} correction={correction_eff:.4f} '
                f'eff_gain={eff_gain:.3f} calf_out={out_calf_eff:.4f}'
            )

        return thigh, out_calf_eff / self.calf_direction_sign

    def js_callback(self, msg):
        out = copy.deepcopy(msg)
        pos = list(out.position)

        for leg, idx in self.indices.items():
            t_i, c_i = idx[1], idx[2]
            if t_i >= len(pos) or c_i >= len(pos):
                continue

            safe_t, safe_c = self.apply_leg_follow(leg, pos[t_i], pos[c_i])
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
            new.velocities = list(point.velocities)
            new.accelerations = list(point.accelerations)
            new.effort = list(point.effort)
            new.time_from_start = point.time_from_start

            for leg, idx in self.indices.items():
                t_i, c_i = idx[1], idx[2]
                if t_i >= len(new.positions) or c_i >= len(new.positions):
                    continue

                safe_t, safe_c = self.apply_leg_follow(
                    leg,
                    new.positions[t_i],
                    new.positions[c_i],
                )
                new.positions[t_i] = safe_t
                new.positions[c_i] = safe_c

            out.points.append(new)

        self.traj_pub.publish(out)


def main():
    rclpy.init()
    node = JaxSimpleCalfFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()