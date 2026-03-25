#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointRemapper(Node):
    def __init__(self):
        super().__init__('joint_remapper')

        # Asymmetric passive zone: calf command is passed through unchanged here.
        self.declare_parameter('thigh_forward_bind', -0.35)
        self.declare_parameter('thigh_backward_bind', 0.90)
        # Multiplies sensed thigh angle before applying passive-zone logic.
        # Use -1.0 if thigh sign is reversed on your hardware.
        self.declare_parameter('thigh_direction_sign', -1.0)
        self.declare_parameter('linkage_ratio', 1.0)
        self.declare_parameter('calf_direction_sign', 1.0)
        self.declare_parameter('max_physical', 0.90)
        self.declare_parameter('enable_calf_slew_limit', True)
        self.declare_parameter('calf_max_step_per_update', 0.02)

        self.THIGH_FORWARD_BIND = float(self.get_parameter('thigh_forward_bind').value)
        self.THIGH_BACKWARD_BIND = float(self.get_parameter('thigh_backward_bind').value)
        self.THIGH_DIRECTION_SIGN = float(self.get_parameter('thigh_direction_sign').value)
        self.LINKAGE_RATIO = float(self.get_parameter('linkage_ratio').value)
        self.CALF_DIRECTION_SIGN = float(self.get_parameter('calf_direction_sign').value)
        self.MAX_PHYSICAL = float(self.get_parameter('max_physical').value)
        self.ENABLE_CALF_SLEW_LIMIT = bool(self.get_parameter('enable_calf_slew_limit').value)
        self.CALF_MAX_STEP = float(self.get_parameter('calf_max_step_per_update').value)

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

        # Last commanded calf value per leg for rate limiting.
        self._last_calf_cmd = {}

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
            f'JointRemapper active (passive_zone=['
            f'{self.THIGH_FORWARD_BIND:.3f}, {self.THIGH_BACKWARD_BIND:.3f}], '
            f'thigh_sign={self.THIGH_DIRECTION_SIGN:.1f}, '
            f'ratio={self.LINKAGE_RATIO:.3f}, '
            f'calf_sign={self.CALF_DIRECTION_SIGN:.1f}, '
            f'max_physical={self.MAX_PHYSICAL:.3f}, '
            f'slew_limit={self.ENABLE_CALF_SLEW_LIMIT}, '
            f'max_step={self.CALF_MAX_STEP:.3f})'
        )

    def _limit_calf_step(self, leg: str, target: float) -> float:
        if not self.ENABLE_CALF_SLEW_LIMIT:
            self._last_calf_cmd[leg] = target
            return target

        prev = self._last_calf_cmd.get(leg)
        if prev is None:
            self._last_calf_cmd[leg] = target
            return target

        delta = target - prev
        if delta > self.CALF_MAX_STEP:
            target = prev + self.CALF_MAX_STEP
        elif delta < -self.CALF_MAX_STEP:
            target = prev - self.CALF_MAX_STEP

        self._last_calf_cmd[leg] = target
        return target

    def apply_linkage_logic(self, thigh_val: float, calf_target: float):
        """Returns (final_calf_pos, correction_active).
        correction_active is True only when thigh is outside the passive zone.
        """
        thigh_eff = thigh_val * self.THIGH_DIRECTION_SIGN

        # True passive zone behavior: if thigh is within bind range, do not
        # move calf unless calf is explicitly commanded.
        if self.THIGH_FORWARD_BIND <= thigh_eff <= self.THIGH_BACKWARD_BIND:
            final_pos = calf_target
            correction_active = False
        else:
            if thigh_eff > self.THIGH_BACKWARD_BIND:
                delta = thigh_eff - self.THIGH_BACKWARD_BIND
            else:
                delta = thigh_eff - self.THIGH_FORWARD_BIND

            correction = -delta * self.LINKAGE_RATIO * self.CALF_DIRECTION_SIGN
            final_pos = calf_target + correction
            correction_active = True

        final_pos = max(min(final_pos, self.MAX_PHYSICAL), -self.MAX_PHYSICAL)
        return final_pos, correction_active

    def js_callback(self, msg: JointState):
        new_js = copy.deepcopy(msg)
        pos = list(new_js.position)

        legs = [('lf', 1, 2), ('rf', 4, 5), ('lh', 7, 8), ('rh', 10, 11)]
        for leg_name, t_idx, c_idx in legs:
            if c_idx < len(pos) and t_idx < len(pos):
                target, correction_active = self.apply_linkage_logic(pos[t_idx], pos[c_idx])
                # Only slew-limit when a linkage correction is active (thigh outside
                # passive zone). Direct calf commands in passive zone are unthrottled.
                if correction_active:
                    pos[c_idx] = self._limit_calf_step(leg_name, target)
                else:
                    self._last_calf_cmd[leg_name] = target
                    pos[c_idx] = target

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
                    target, correction_active = self.apply_linkage_logic(pos[idx[1]], pos[idx[2]])
                    if correction_active:
                        pos[idx[2]] = self._limit_calf_step(leg_name, target)
                    else:
                        self._last_calf_cmd[leg_name] = target
                        pos[idx[2]] = target

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
