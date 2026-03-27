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
        self.follow_gain = float(
            self.declare_parameter('follow_gain', 1.0).value
        )
        self.debug_logs = bool(
            self.declare_parameter('debug_logs', False).value
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
            f'follow_gain={self.follow_gain})'
        )

    def apply_leg_follow(self, leg, thigh, calf):
        thigh_eff = thigh * self.thigh_direction_sign
        calf_eff = calf * self.calf_direction_sign

        prev = self.prev_state.get(leg)
        if prev is None:
            out_calf_eff = calf_eff
        else:
            dthigh_eff = thigh_eff - prev['thigh_eff']
            out_calf_eff = prev['calf_eff'] - (dthigh_eff * self.follow_gain)
            if self.debug_logs:
                self.get_logger().info(
                    f'follow[{leg}] dthigh_eff={dthigh_eff:.4f} '
                    f'calf_eff={out_calf_eff:.4f}'
                )

        self.prev_state[leg] = {
            'thigh_eff': thigh_eff,
            'calf_eff': out_calf_eff,
        }

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