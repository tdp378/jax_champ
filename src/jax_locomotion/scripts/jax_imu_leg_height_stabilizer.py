#!/usr/bin/env python3

"""
IMU-Based Leg Height Stabilizer for Jax Quadruped
================================================

This node measures body tilt from the IMU and computes per-leg height 
adjustments to maintain level body posture during locomotion.

Instead of modifying cmd_vel, this stabilizer:
1. Subscribes to raw IMU data
2. Extracts roll/pitch with sensor fusion
3. Computes stabilization forces for each leg
4. Publishes leg height offsets that are applied to the foot placement

Integration with CHAMP:
- Publishes to /jax/leg_height_offsets (new topic)
- Can be subscribed by a modified CHAMP wrapper that applies offsets
  to foot trajectory planning

Key improvements:
- Full PID control (P + I + D) for smooth damped response
- Per-leg height modulation for true body stabilization
- Speed-adaptive gains (adjusts based on forward velocity)
- Kalman filter for sensor fusion (gyro + accel)
- Diagnostic output for tuning
"""

import math
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray


class SimpleKalmanFilter1D:
    """
    1D Kalman filter for fusing gyro (fast, drifts) and accel (slow, stable).
    Handles angular position (roll/pitch).
    """
    
    def __init__(self, initial_value=0.0, process_noise=0.001, measurement_noise=0.1):
        """
        Args:
            initial_value: Starting angle (rad)
            process_noise: How much we expect the angle to change between steps
            measurement_noise: How much we trust the accelerometer measurement
        """
        self.x = initial_value  # State: estimated angle
        self.p = 1.0  # Uncertainty
        self.q = process_noise  # Process noise (gyro drift)
        self.r = measurement_noise  # Measurement noise (accel noise)
        
    def update(self, gyro_rate, accel_angle, dt):
        """
        Update filter with gyro rate and accelerometer angle.
        
        Args:
            gyro_rate: Angular velocity from gyro (rad/s)
            accel_angle: Angle estimated from accelerometer (rad)
            dt: Time step (seconds)
            
        Returns:
            Filtered angle estimate (rad)
        """
        # Predict step: use gyro to predict new angle
        x_pred = self.x + gyro_rate * dt
        p_pred = self.p + self.q
        
        # Update step: correct with accelerometer
        innovation = accel_angle - x_pred
        s = p_pred + self.r  # Innovation covariance
        k = p_pred / s  # Kalman gain
        
        self.x = x_pred + k * innovation
        self.p = (1.0 - k) * p_pred
        
        return self.x


class JaxIMULegHeightStabilizer(Node):
    
    def __init__(self):
        super().__init__('jax_imu_leg_height_stabilizer')
        
        # ============================================================
        # Parameters - Tunable via ROS param server
        # ============================================================
        
        # PID gains for roll stabilization
        self.declare_parameter('kp_roll', 0.05)
        self.declare_parameter('ki_roll', 0.005)
        self.declare_parameter('kd_roll', 0.02)
        
        # PID gains for pitch stabilization
        self.declare_parameter('kp_pitch', 0.05)
        self.declare_parameter('ki_pitch', 0.005)
        self.declare_parameter('kd_pitch', 0.02)
        
        # Sensor fusion (Kalman filter tuning)
        self.declare_parameter('imu_process_noise', 0.001)
        self.declare_parameter('imu_measurement_noise', 0.05)
        
        # Stabilization limits
        self.declare_parameter('max_leg_height_adjustment', 0.05)  # meters
        self.declare_parameter('deadband_angle', 0.005)  # rad (~0.3 degrees)
        
        # Motion-dependent tuning
        self.declare_parameter('speed_scale_factor', 1.0)
        self.declare_parameter('enable_speed_adaptation', True)
        
        # Enable/disable
        self.declare_parameter('enabled', True)
        self.declare_parameter('only_when_moving', True)
        self.declare_parameter('motion_threshold', 0.02)
        
        # Diagnostics
        self.declare_parameter('publish_diagnostics', True)
        
        # Load parameters
        self.kp_roll = float(self.get_parameter('kp_roll').value)
        self.ki_roll = float(self.get_parameter('ki_roll').value)
        self.kd_roll = float(self.get_parameter('kd_roll').value)
        
        self.kp_pitch = float(self.get_parameter('kp_pitch').value)
        self.ki_pitch = float(self.get_parameter('ki_pitch').value)
        self.kd_pitch = float(self.get_parameter('kd_pitch').value)
        
        self.imu_process_noise = float(self.get_parameter('imu_process_noise').value)
        self.imu_measurement_noise = float(self.get_parameter('imu_measurement_noise').value)
        
        self.max_leg_height_adjustment = float(self.get_parameter('max_leg_height_adjustment').value)
        self.deadband_angle = float(self.get_parameter('deadband_angle').value)
        
        self.speed_scale_factor = float(self.get_parameter('speed_scale_factor').value)
        self.enable_speed_adaptation = bool(self.get_parameter('enable_speed_adaptation').value)
        
        self.enabled = bool(self.get_parameter('enabled').value)
        self.only_when_moving = bool(self.get_parameter('only_when_moving').value)
        self.motion_threshold = float(self.get_parameter('motion_threshold').value)
        
        self.publish_diagnostics = bool(self.get_parameter('publish_diagnostics').value)
        
        # ============================================================
        # Internal state
        # ============================================================
        
        # IMU data (raw)
        self.roll_raw = 0.0
        self.pitch_raw = 0.0
        self.roll_rate = 0.0  # rad/s from gyro
        self.pitch_rate = 0.0  # rad/s from gyro
        
        # Kalman filters for sensor fusion
        self.roll_filter = SimpleKalmanFilter1D(
            initial_value=0.0,
            process_noise=self.imu_process_noise,
            measurement_noise=self.imu_measurement_noise
        )
        self.pitch_filter = SimpleKalmanFilter1D(
            initial_value=0.0,
            process_noise=self.imu_process_noise,
            measurement_noise=self.imu_measurement_noise
        )
        
        # Filtered IMU data (post-Kalman)
        self.roll_filtered = 0.0
        self.pitch_filtered = 0.0
        
        # PID state
        self.roll_error_integral = 0.0
        self.pitch_error_integral = 0.0
        self.roll_error_prev = 0.0
        self.pitch_error_prev = 0.0
        
        # Command velocity (for speed adaptation)
        self.last_cmd_vel = Twist()
        self.last_update_time = self.get_clock().now()
        
        # Leg names in order: [FL, FR, BL, BR]
        self.leg_names = ['FL', 'FR', 'BL', 'BR']
        self.num_legs = 4
        
        # ============================================================
        # ROS Publishers/Subscribers
        # ============================================================
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            50
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish leg height offsets: [FL, FR, BL, BR]
        # Positive = raise leg, Negative = lower leg
        self.leg_height_pub = self.create_publisher(
            Float32MultiArray,
            '/jax/leg_height_offsets',
            10
        )
        
        # Optional: publish diagnostics for visualization/debugging
        if self.publish_diagnostics:
            self.diag_pub = self.create_publisher(
                Float32MultiArray,
                '/jax/imu_stabilizer/diagnostics',
                5
            )
        
        # Update loop at 100 Hz
        self.timer = self.create_timer(0.01, self.update_loop)
        
        self.get_logger().info(
            f"IMU Leg Height Stabilizer started\n"
            f"  Roll gains (P/I/D): {self.kp_roll}/{self.ki_roll}/{self.kd_roll}\n"
            f"  Pitch gains (P/I/D): {self.kp_pitch}/{self.ki_pitch}/{self.kd_pitch}\n"
            f"  Max adjustment: {self.max_leg_height_adjustment}m\n"
            f"  Speed adaptation: {self.enable_speed_adaptation}"
        )
    
    # ================================================================
    # Callbacks
    # ================================================================
    
    def imu_callback(self, msg: Imu):
        """
        Process IMU data: extract roll/pitch and angular rates.
        """
        q = msg.orientation
        
        # Convert quaternion to roll/pitch
        self.roll_raw, self.pitch_raw = self._quat_to_rp(
            q.x, q.y, q.z, q.w
        )
        
        # Extract angular rates from gyro
        self.roll_rate = msg.angular_velocity.x
        self.pitch_rate = msg.angular_velocity.y
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Track command velocity for speed-adaptive gain scaling.
        """
        self.last_cmd_vel = msg
    
    # ================================================================
    # Core stabilization logic
    # ================================================================
    
    def update_loop(self):
        """
        Main control loop (100 Hz):
        1. Fuse IMU data with Kalman filter
        2. Compute PID corrections for roll/pitch
        3. Convert to per-leg height adjustments
        4. Publish offsets
        """
        current_time = self.get_clock().now()
        dt_seconds = (current_time - self.last_update_time).nanoseconds / 1e9
        if dt_seconds <= 0:
            dt_seconds = 0.01
        self.last_update_time = current_time
        
        # Clamp dt to prevent huge jumps (e.g., during startup)
        dt_seconds = max(0.001, min(0.1, dt_seconds))
        
        # Check if enabled and moving
        if not self.enabled:
            return
        
        moving = (
            abs(self.last_cmd_vel.linear.x) > self.motion_threshold or
            abs(self.last_cmd_vel.linear.y) > self.motion_threshold or
            abs(self.last_cmd_vel.angular.z) > self.motion_threshold
        )
        
        if self.only_when_moving and not moving:
            return
        
        # ========== Sensor Fusion: Kalman Filtering ==========
        self.roll_filtered = self.roll_filter.update(
            self.roll_rate,
            self.roll_raw,
            dt_seconds
        )
        self.pitch_filtered = self.pitch_filter.update(
            self.pitch_rate,
            self.pitch_raw,
            dt_seconds
        )
        
        # ========== Deadband ==========
        roll_error = self.roll_filtered if abs(self.roll_filtered) > self.deadband_angle else 0.0
        pitch_error = self.pitch_filtered if abs(self.pitch_filtered) > self.deadband_angle else 0.0
        
        # ========== Speed-Adaptive Gain Scaling ==========
        forward_speed = abs(self.last_cmd_vel.linear.x)
        speed_scale = self._compute_speed_scale(forward_speed)
        
        kp_r = self.kp_roll * speed_scale
        ki_r = self.ki_roll * speed_scale
        kd_r = self.kd_roll * speed_scale
        
        kp_p = self.kp_pitch * speed_scale
        ki_p = self.ki_pitch * speed_scale
        kd_p = self.kd_pitch * speed_scale
        
        # ========== PID Control: Roll ==========
        self.roll_error_integral += roll_error * dt_seconds
        roll_deriv = (roll_error - self.roll_error_prev) / dt_seconds
        
        roll_correction = (
            kp_r * roll_error +
            ki_r * self.roll_error_integral +
            kd_r * roll_deriv
        )
        
        self.roll_error_prev = roll_error
        
        # ========== PID Control: Pitch ==========
        self.pitch_error_integral += pitch_error * dt_seconds
        pitch_deriv = (pitch_error - self.pitch_error_prev) / dt_seconds
        
        pitch_correction = (
            kp_p * pitch_error +
            ki_p * self.pitch_error_integral +
            kd_p * pitch_deriv
        )
        
        self.pitch_error_prev = pitch_error
        
        # ========== Convert to Leg Height Adjustments ==========
        leg_heights = self._compute_leg_heights(roll_correction, pitch_correction)
        
        # ========== Publish ==========
        self._publish_leg_heights(leg_heights)
        
        # ========== Diagnostics ==========
        if self.publish_diagnostics:
            self._publish_diagnostics(
                roll_error, pitch_error, roll_correction, pitch_correction, 
                forward_speed, speed_scale
            )
    
    def _compute_leg_heights(self, roll_corr, pitch_corr):
        """
        Convert roll and pitch corrections into per-leg height adjustments.
        
        Geometry:
        - Roll correction tilts left/right -> adjust diagonal pairs
        - Pitch correction tilts forward/back -> adjust front/rear pairs
        
        Layout: FL, FR, BL, BR
        
        For positive roll (tilting right):
          - Raise left legs: FL, BL
          - Lower right legs: FR, BR
        
        For positive pitch (tilting back):
          - Raise back legs: BL, BR
          - Lower front legs: FL, FR
        """
        
        # Clamp corrections to max adjustment
        roll_corr = np.clip(roll_corr, 
                           -self.max_leg_height_adjustment,
                           self.max_leg_height_adjustment)
        pitch_corr = np.clip(pitch_corr,
                            -self.max_leg_height_adjustment,
                            self.max_leg_height_adjustment)
        
        # Initialize adjustments
        adjustments = [0.0] * self.num_legs  # [FL, FR, BL, BR]
        
        # Roll: tilting left/right
        # Positive roll = tilting right = lower right, raise left
        adjustments[0] += roll_corr  # FL: left front (positive = raise)
        adjustments[1] -= roll_corr  # FR: right front (negative = lower)
        adjustments[2] += roll_corr  # BL: left back (positive = raise)
        adjustments[3] -= roll_corr  # BR: right back (negative = lower)
        
        # Pitch: tilting forward/back
        # Positive pitch = tilting back = lower back, raise front
        adjustments[0] -= pitch_corr  # FL: front (negative = raise for positive pitch)
        adjustments[1] -= pitch_corr  # FR: front
        adjustments[2] += pitch_corr  # BL: back (positive = lower for positive pitch)
        adjustments[3] += pitch_corr  # BR: back
        
        return adjustments
    
    def _compute_speed_scale(self, forward_speed):
        """
        Adapt PID gains based on walking speed.
        
        At very low speeds: use gentle gains (avoid overcorrection)
        At high speeds: use aggressive gains (respond quickly)
        """
        if not self.enable_speed_adaptation:
            return 1.0
        
        # Sigmoid-like scaling: gradual increase from 0.5 to 2.0
        # At speed=0: scale=0.5 (conservative)
        # At speed=0.5 m/s: scale=1.0 (nominal)
        # At speed=1.0 m/s: scale=1.5 (aggressive)
        
        nominal_speed = 0.5
        scale = 0.5 + 1.5 * (forward_speed / (forward_speed + nominal_speed))
        return np.clip(scale, 0.3, 2.0)
    
    def _publish_leg_heights(self, adjustments):
        """Publish leg height offsets."""
        msg = Float32MultiArray()
        msg.data = adjustments
        self.leg_height_pub.publish(msg)
    
    def _publish_diagnostics(self, roll_err, pitch_err, roll_corr, pitch_corr, 
                            fwd_speed, speed_scale):
        """Publish diagnostic data for visualization."""
        diag_data = [
            float(roll_err),
            float(pitch_err),
            float(roll_corr),
            float(pitch_corr),
            float(fwd_speed),
            float(speed_scale),
            float(self.roll_filtered),
            float(self.pitch_filtered),
        ]
        
        msg = Float32MultiArray()
        msg.data = diag_data
        self.diag_pub.publish(msg)
    
    # ================================================================
    # Utilities
    # ================================================================
    
    @staticmethod
    def _quat_to_rp(x, y, z, w):
        """
        Convert quaternion to roll and pitch (ignoring yaw).
        """
        # Roll (rotation around X)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (rotation around Y)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)
        
        return roll, pitch


def main(args=None):
    rclpy.init(args=args)
    node = JaxIMULegHeightStabilizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()