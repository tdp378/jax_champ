#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


def quat_to_rpy(x, y, z, w):

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


class JaxCmdVelStabilizer(Node):

    def __init__(self):
        super().__init__('jax_imu_cmdvel_stabilizer')

        #
        # ------------------------------
        # Parameters you can tune
        # ------------------------------
        #

        self.declare_parameter("kp_pitch", 0.7)
        self.declare_parameter("kp_roll", 0.6)

        self.declare_parameter("deadband", 0.02)

        self.declare_parameter("max_correction", 0.20)

        self.declare_parameter("alpha", 0.2)

        self.declare_parameter("enabled", True)

        self.declare_parameter("only_when_moving", True)
        self.declare_parameter("motion_threshold", 0.02)

        #
        # Load parameters
        #

        self.kp_pitch = float(self.get_parameter("kp_pitch").value)
        self.kp_roll = float(self.get_parameter("kp_roll").value)

        self.deadband = float(self.get_parameter("deadband").value)

        self.max_correction = float(self.get_parameter("max_correction").value)

        self.alpha = float(self.get_parameter("alpha").value)

        self.enabled = bool(self.get_parameter("enabled").value)

        self.only_when_moving = bool(self.get_parameter("only_when_moving").value)
        self.motion_threshold = float(self.get_parameter("motion_threshold").value)

        #
        # Internal state
        #

        self.roll = 0.0
        self.pitch = 0.0

        self.roll_f = 0.0
        self.pitch_f = 0.0

        self.last_cmd = Twist()

        #
        # Subscribers
        #

        self.imu_sub = self.create_subscription(
            Imu,
            "/imu/data",
            self.imu_callback,
            50
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel/smooth",
            self.cmd_callback,
            10
        )

        #
        # Publisher
        #

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel/smooth_stable",
            10
        )

        #
        # Timer loop
        #

        self.timer = self.create_timer(
            0.01,
            self.update
        )

        self.get_logger().info("IMU cmd_vel stabilizer started")

    #
    # IMU callback
    #

    def imu_callback(self, msg: Imu):

        q = msg.orientation

        roll, pitch, _ = quat_to_rpy(
            q.x,
            q.y,
            q.z,
            q.w
        )

        self.roll = roll
        self.pitch = pitch

        #
        # Low pass filter
        #

        self.roll_f = self.alpha * self.roll + (1.0 - self.alpha) * self.roll_f
        self.pitch_f = self.alpha * self.pitch + (1.0 - self.alpha) * self.pitch_f

    #
    # Velocity callback
    #

    def cmd_callback(self, msg: Twist):

        self.last_cmd = msg

    #
    # Stabilization loop
    #

    def update(self):

        cmd = Twist()

        moving = (
            abs(self.last_cmd.linear.x) > self.motion_threshold or
            abs(self.last_cmd.linear.y) > self.motion_threshold or
            abs(self.last_cmd.angular.z) > self.motion_threshold
        )

        if not self.enabled:
            self.cmd_pub.publish(self.last_cmd)
            return

        if self.only_when_moving and not moving:
            self.cmd_pub.publish(self.last_cmd)
            return

        #
        # Deadband
        #

        roll_used = 0.0 if abs(self.roll_f) < self.deadband else self.roll_f
        pitch_used = 0.0 if abs(self.pitch_f) < self.deadband else self.pitch_f

        #
        # Stabilization corrections
        #

        pitch_correction = -self.kp_pitch * pitch_used
        roll_correction = -self.kp_roll * roll_used

        #
        # Clamp corrections
        #

        pitch_correction = clamp(
            pitch_correction,
            -self.max_correction,
            self.max_correction
        )

        roll_correction = clamp(
            roll_correction,
            -self.max_correction,
            self.max_correction
        )

        #
        # Apply correction
        #

        cmd.linear.x = self.last_cmd.linear.x + pitch_correction
        cmd.linear.y = self.last_cmd.linear.y + roll_correction
        cmd.angular.z = self.last_cmd.angular.z

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = JaxCmdVelStabilizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()