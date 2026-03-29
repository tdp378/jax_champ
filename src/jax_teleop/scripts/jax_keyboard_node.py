#!/usr/bin/env python3

import os
import select
import sys
import termios
import tty
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


@dataclass
class CommandState:
    vx_dir: float = 0.0
    vy_dir: float = 0.0
    wz_dir: float = 0.0


class JaxKeyboardNode(Node):
    def __init__(self):
        super().__init__('jax_keyboard_node')

        
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('mode_topic', '/jax_mode')
        self.declare_parameter('publish_rate_hz', 20.0)

        self.declare_parameter('base_linear_x', 0.08)
        self.declare_parameter('base_linear_y', 0.08)
        self.declare_parameter('base_angular_z', 0.15)

        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('speed_scale_step', 0.1)
        self.declare_parameter('speed_scale_min', 0.2)
        self.declare_parameter('speed_scale_max', 2.0)

        self.declare_parameter('walk_mode_name', 'walk')
        self.declare_parameter('stand_mode_name', 'stand')
        self.declare_parameter('sit_mode_name', 'sit')
        self.declare_parameter('lay_mode_name', 'lay')
        self.declare_parameter('pose_mode_name', 'pose')

        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        mode_topic = str(self.get_parameter('mode_topic').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.base_linear_x = float(self.get_parameter('base_linear_x').value)
        self.base_linear_y = float(self.get_parameter('base_linear_y').value)
        self.base_angular_z = float(self.get_parameter('base_angular_z').value)

        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.speed_scale_step = float(self.get_parameter('speed_scale_step').value)
        self.speed_scale_min = float(self.get_parameter('speed_scale_min').value)
        self.speed_scale_max = float(self.get_parameter('speed_scale_max').value)

        self.walk_mode_name = str(self.get_parameter('walk_mode_name').value)
        self.stand_mode_name = str(self.get_parameter('stand_mode_name').value)
        self.sit_mode_name = str(self.get_parameter('sit_mode_name').value)
        self.lay_mode_name = str(self.get_parameter('lay_mode_name').value)
        self.pose_mode_name = str(self.get_parameter('pose_mode_name').value)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.mode_pub = self.create_publisher(String, mode_topic, 10)

        self.cmd_state = CommandState()
        self.current_mode = 'unknown'
        self.last_key = '-'
        self.status_msg = 'ready'

        self.tty = open('/dev/tty', 'rb', buffering=0)
        self.tty_fd = self.tty.fileno()
        self.settings = termios.tcgetattr(self.tty_fd)

        self.publish_timer = self.create_timer(1.0 / publish_rate_hz, self.publish_cmd)

        self.draw_dashboard()

    def make_twist(self) -> Twist:
        msg = Twist()
        msg.linear.x = self.cmd_state.vx_dir * self.base_linear_x * self.speed_scale
        msg.linear.y = self.cmd_state.vy_dir * self.base_linear_y * self.speed_scale
        msg.angular.z = self.cmd_state.wz_dir * self.base_angular_z * self.speed_scale
        return msg

    def zero_cmd(self):
        self.cmd_state = CommandState()

    def set_cmd(self, vx_dir=0.0, vy_dir=0.0, wz_dir=0.0):
        self.cmd_state = CommandState(vx_dir=vx_dir, vy_dir=vy_dir, wz_dir=wz_dir)

    def publish_cmd(self):
        self.cmd_pub.publish(self.make_twist())

    def publish_mode(self, mode_name: str):
        msg = String()
        msg.data = mode_name
        self.mode_pub.publish(msg)
        self.current_mode = mode_name

    def motion_label(self) -> str:
        c = self.cmd_state
        parts = []

        if c.vx_dir > 0:
            parts.append('FWD')
        elif c.vx_dir < 0:
            parts.append('BACK')

        if c.vy_dir > 0:
            parts.append('LEFT')
        elif c.vy_dir < 0:
            parts.append('RIGHT')

        if c.wz_dir > 0:
            parts.append('TURN-L')
        elif c.wz_dir < 0:
            parts.append('TURN-R')

        return ' + '.join(parts) if parts else 'STOP'

    def build_dashboard(self) -> str:
        cmd = self.make_twist()

        lines = [
            "JAX TELEOP",
            "============================================================",
            "Move : w/s  a/d  q/e  z/c  x stop  SPACE estop",
            "Mode : m walk  1 stand  2 sit  3 lay  4 pose",
            "Speed: + faster  - slower",
            "",
            f"Mode      : {self.current_mode}",
            f"Motion    : {self.motion_label()}",
            f"Scale     : {self.speed_scale:.2f}",
            f"Last Key  : {self.last_key}",
            f"Status    : {self.status_msg}",
            "",
            f"cmd vx    : {cmd.linear.x:+.3f}",
            f"cmd vy    : {cmd.linear.y:+.3f}",
            f"cmd wz    : {cmd.angular.z:+.3f}",
            "",
            "CTRL+C to quit",
        ]

        return '\033[2J\033[H' + '\n'.join(lines)

    def draw_dashboard(self):
        sys.stdout.write(self.build_dashboard())
        sys.stdout.flush()

    def get_key(self) -> str:
        tty.setraw(self.tty_fd)
        rlist, _, _ = select.select([self.tty_fd], [], [], 0.05)
        key = os.read(self.tty_fd, 1).decode('utf-8') if rlist else ''
        termios.tcsetattr(self.tty_fd, termios.TCSADRAIN, self.settings)
        return key

    def process_key(self, key: str):
        self.last_key = 'SPACE' if key == ' ' else key

        if key == 's':
            self.set_cmd(vx_dir=1.0)
            self.status_msg = 'forward'
        elif key == 'w':
            self.set_cmd(vx_dir=-1.0)
            self.status_msg = 'backward'
        elif key == 'a':
            self.set_cmd(vy_dir=1.0)
            self.status_msg = 'strafe left'
        elif key == 'd':
            self.set_cmd(vy_dir=-1.0)
            self.status_msg = 'strafe right'
        elif key == 'q':
            self.set_cmd(wz_dir=1.0)
            self.status_msg = 'turn left'
        elif key == 'e':
            self.set_cmd(wz_dir=-1.0)
            self.status_msg = 'turn right'
        elif key == 'z':
            self.set_cmd(vx_dir=1.0, wz_dir=1.0)
            self.status_msg = 'forward + turn left'
        elif key == 'c':
            self.set_cmd(vx_dir=1.0, wz_dir=-1.0)
            self.status_msg = 'forward + turn right'
        elif key == 'x':
            self.zero_cmd()
            self.status_msg = 'stop'
        elif key == ' ':
            self.zero_cmd()
            self.status_msg = 'EMERGENCY STOP'
        elif key == 'm':
            self.publish_mode(self.walk_mode_name)
            # SMALL ROTATION TO FORCE GAIT (march in place-ish)
            self.set_cmd(vx_dir=-0.2)
            self.status_msg = 'mode -> walk (auto step)'
        elif key == '1':
            self.zero_cmd()
            self.publish_mode(self.stand_mode_name)
            self.status_msg = 'mode -> stand'
        elif key == '2':
            self.zero_cmd()
            self.publish_mode(self.sit_mode_name)
            self.status_msg = 'mode -> sit'
        elif key == '3':
            self.zero_cmd()
            self.publish_mode(self.lay_mode_name)
            self.status_msg = 'mode -> lay'
        elif key == '4':
            self.zero_cmd()
            self.publish_mode(self.pose_mode_name)
            self.status_msg = 'mode -> pose'
        elif key in ['+', '=']:
            self.speed_scale = min(
                self.speed_scale + self.speed_scale_step,
                self.speed_scale_max
            )
            self.status_msg = f'scale -> {self.speed_scale:.2f}'
        elif key == '-':
            self.speed_scale = max(
                self.speed_scale - self.speed_scale_step,
                self.speed_scale_min
            )
            self.status_msg = f'scale -> {self.speed_scale:.2f}'
        elif key == 'r':
            self.status_msg = 'refresh'
        else:
            self.status_msg = f'unknown key: {repr(key)}'

        self.draw_dashboard()

    def run(self):
        try:
            sys.stdout.write('\033[2J\033[H')
            sys.stdout.flush()

            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                key = self.get_key()

                if key == '\x03':
                    break

                if key:
                    self.process_key(key)

        finally:
            self.zero_cmd()
            self.publish_cmd()
            sys.stdout.write('\033[2J\033[H')
            sys.stdout.flush()
            termios.tcsetattr(self.tty_fd, termios.TCSADRAIN, self.settings)
            self.tty.close()


def main(args=None):
    rclpy.init(args=args)
    node = JaxKeyboardNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()