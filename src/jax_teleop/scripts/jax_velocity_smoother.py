#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class JaxVelocitySmoother(Node):
    def __init__(self):
        super().__init__('jax_velocity_smoother')

       
        # Topics
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel/smooth')

        # Loop / timeout
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('command_timeout', 0.5)

        # Speed limits
        self.declare_parameter('max_vx', 0.10)
        self.declare_parameter('max_vy', 0.10)
        self.declare_parameter('max_wz', 0.20)

        # Accel limits
        self.declare_parameter('accel_vx', 0.25)
        self.declare_parameter('accel_vy', 0.25)
        self.declare_parameter('accel_wz', 0.40)

        # Decel limits
        self.declare_parameter('decel_vx', 0.35)
        self.declare_parameter('decel_vy', 0.35)
        self.declare_parameter('decel_wz', 0.60)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self.rate = float(self.get_parameter('rate').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)

        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)

        self.accel_vx = float(self.get_parameter('accel_vx').value)
        self.accel_vy = float(self.get_parameter('accel_vy').value)
        self.accel_wz = float(self.get_parameter('accel_wz').value)

        self.decel_vx = float(self.get_parameter('decel_vx').value)
        self.decel_vy = float(self.get_parameter('decel_vy').value)
        self.decel_wz = float(self.get_parameter('decel_wz').value)

        self.target = Twist()
        self.current = Twist()

        self.last_input_time = self.get_clock().now()

        self.sub = self.create_subscription(
            Twist,
            input_topic,
            self.cmd_callback,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            output_topic,
            10
        )

        self.timer = self.create_timer(1.0 / self.rate, self.update)

        self.get_logger().info(
            f'jax_velocity_smoother: {input_topic} -> {output_topic} | '
            f'max=({self.max_vx:.2f}, {self.max_vy:.2f}, {self.max_wz:.2f}) | '
            f'accel=({self.accel_vx:.2f}, {self.accel_vy:.2f}, {self.accel_wz:.2f}) | '
            f'decel=({self.decel_vx:.2f}, {self.decel_vy:.2f}, {self.decel_wz:.2f})'
        )

    @staticmethod
    def clamp(value: float, limit: float) -> float:
        return max(min(value, limit), -limit)

    @staticmethod
    def sign(x: float) -> float:
        if x > 0.0:
            return 1.0
        if x < 0.0:
            return -1.0
        return 0.0

    def cmd_callback(self, msg: Twist):
        self.target.linear.x = self.clamp(msg.linear.x, self.max_vx)
        self.target.linear.y = self.clamp(msg.linear.y, self.max_vy)
        self.target.angular.z = self.clamp(msg.angular.z, self.max_wz)
        self.last_input_time = self.get_clock().now()

    def slew(self, current: float, target: float, accel: float, decel: float, dt: float) -> float:
        delta = target - current

        if abs(delta) < 1e-6:
            return target

        same_direction = (current == 0.0) or (self.sign(current) == self.sign(target))

        if same_direction and abs(target) > abs(current):
            limit = accel
        else:
            limit = decel

        max_step = limit * dt

        if delta > max_step:
            return current + max_step
        if delta < -max_step:
            return current - max_step
        return target

    def update(self):
        now = self.get_clock().now()
        dt = 1.0 / self.rate

        age = (now - self.last_input_time).nanoseconds / 1e9
        if age > self.command_timeout:
            self.target = Twist()

        self.current.linear.x = self.slew(
            self.current.linear.x,
            self.target.linear.x,
            self.accel_vx,
            self.decel_vx,
            dt
        )

        self.current.linear.y = self.slew(
            self.current.linear.y,
            self.target.linear.y,
            self.accel_vy,
            self.decel_vy,
            dt
        )

        self.current.angular.z = self.slew(
            self.current.angular.z,
            self.target.angular.z,
            self.accel_wz,
            self.decel_wz,
            dt
        )

        self.pub.publish(self.current)


def main(args=None):
    rclpy.init(args=args)
    node = JaxVelocitySmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()