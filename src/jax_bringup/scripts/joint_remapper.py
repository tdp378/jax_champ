#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointRemapper(Node):
    def __init__(self):
        super().__init__('joint_remapper')

        # Linkage model parameters (legacy behavior)
        self.declare_parameter('neutral_limit', 1.0)
        self.declare_parameter('max_physical', 2.0)

        self.NEUTRAL_LIMIT = float(self.get_parameter('neutral_limit').value)
        self.MAX_PHYSICAL = float(self.get_parameter('max_physical').value)

        # Joint indices per leg: [hip, thigh, calf]
        self.indices = {
            'lf': [0, 1, 2],
            'rf': [3, 4, 5],
            'lh': [6, 7, 8],
            'rh': [9, 10, 11],
        }

        # Legacy default: all legs coupled, no hip flip
        self.leg_config = {
            'lf': {'flip': False, 'coupled': True},
            'rf': {'flip': False, 'coupled': True},
            'lh': {'flip': False, 'coupled': True},
            'rh': {'flip': False, 'coupled': True},
        }

        # CHAMP/trajectory path
        self.sub = self.create_subscription(
            JointTrajectory,
            '/jax/walk_joint_trajectory_raw',
            self.joint_callback,
            10,
        )
        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_group_effort_controller/joint_trajectory',
            10,
        )

        # Slider/RViz JointState path
        self.js_sub = self.create_subscription(
            JointState,
            '/joint_states_raw',
            self.js_callback,
            10,
        )
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.get_logger().info(
            f'JointRemapper active (neutral_limit={self.NEUTRAL_LIMIT:.3f}, '
            f'max_physical={self.MAX_PHYSICAL:.3f})'
        )

    def apply_linkage_logic(self, thigh_val: float, calf_target: float) -> float:
        # Base remap: calf and thigh are mechanically coupled.
        motor_request = calf_target - thigh_val

        # Cooperative limit logic near and beyond neutral limits.
        if thigh_val > self.NEUTRAL_LIMIT:
            assist_offset = thigh_val - self.NEUTRAL_LIMIT
            final_pos = max(motor_request, -self.NEUTRAL_LIMIT + assist_offset)
        elif thigh_val < -self.NEUTRAL_LIMIT:
            assist_offset = thigh_val + self.NEUTRAL_LIMIT
            final_pos = min(motor_request, self.NEUTRAL_LIMIT + assist_offset)
        else:
            final_pos = max(min(motor_request, self.NEUTRAL_LIMIT), -self.NEUTRAL_LIMIT)

        # Final absolute physical clamp
        final_pos = max(min(final_pos, self.MAX_PHYSICAL), -self.MAX_PHYSICAL)
        return final_pos

    def js_callback(self, msg: JointState):
        new_js = copy.deepcopy(msg)
        pos = list(new_js.position)

        legs = [(1, 2), (4, 5), (7, 8), (10, 11)]
        for t_idx, c_idx in legs:
            if c_idx < len(pos) and t_idx < len(pos):
                pos[c_idx] = self.apply_linkage_logic(pos[t_idx], pos[c_idx])

        new_js.position = pos
        self.js_pub.publish(new_js)

    def joint_callback(self, msg: JointTrajectory):
        new_msg = JointTrajectory()
        new_msg.header = msg.header
        new_msg.joint_names = msg.joint_names

        for point in msg.points:
            new_point = JointTrajectoryPoint()
            new_point.time_from_start = point.time_from_start
            pos = list(point.positions)

            for leg_name, config in self.leg_config.items():
                idx = self.indices[leg_name]

                if idx[2] >= len(pos):
                    continue

                if config.get('flip', False):
                    pos[idx[0]] = pos[idx[0]] * -1.0

                if config.get('coupled', False):
                    pos[idx[2]] = self.apply_linkage_logic(pos[idx[1]], pos[idx[2]])

            new_point.positions = pos
            new_msg.points.append(new_point)

        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointRemapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
