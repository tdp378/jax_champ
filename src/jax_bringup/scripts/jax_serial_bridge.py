#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import BatteryState

# Arduino PCA9685 channel → ROS joint name mapping
# Hardware wiring: RH=ch0-2, LH=ch3-5, LF=ch6-8, RF=ch9-11
ARDUINO_ORDER = [
    "rh_hip_joint",  "rh_upper_leg_joint",  "rh_lower_leg_joint",  # ch 0-2
    "lh_hip_joint",  "lh_upper_leg_joint",  "lh_lower_leg_joint",  # ch 3-5
    "lf_hip_joint",  "lf_upper_leg_joint",  "lf_lower_leg_joint",  # ch 6-8
    "rf_hip_joint",  "rf_upper_leg_joint",  "rf_lower_leg_joint",  # ch 9-11
]


class JaxSerialBridge(Node):
    def __init__(self):
        super().__init__('jax_serial_bridge')

        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.armed = False
        self.ser = None

        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            time.sleep(2.0)
            self.get_logger().info(f'Serial bridge open: {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Serial open failed: {e}')
            return

        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/joint_group_effort_controller/joint_trajectory',
            self.traj_callback,
            10)

        self.battery_pub = self.create_publisher(BatteryState, 'jax/battery', 10)

        # Low-rate timer to drain serial feedback (voltage telemetry)
        self.create_timer(0.1, self.read_feedback)

    def traj_callback(self, msg):
        if not msg.points or self.ser is None:
            return

        # Send WAKE on first command to activate servos/NeoPixels
        if not self.armed:
            try:
                self.ser.write(b'WAKE\n')
                self.armed = True
                self.get_logger().info('JAX ARMED — servos active')
            except Exception as e:
                self.get_logger().error(f'WAKE failed: {e}')
                return

        # Build joint value map from the message
        joint_map = dict(zip(msg.joint_names, msg.points[0].positions))

        # Reorder to Arduino channel order; default 0.0 if joint missing
        values = [joint_map.get(name, 0.0) for name in ARDUINO_ORDER]

        payload = 'J:' + ','.join(f'{v:.4f}' for v in values) + '\n'
        try:
            self.ser.write(payload.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    def read_feedback(self):
        if self.ser is None:
            return
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('VOLT:'):
                    batt = BatteryState()
                    batt.voltage = float(line[5:])
                    self.battery_pub.publish(batt)
        except Exception:
            pass

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'SLEEP\n')
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JaxSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
