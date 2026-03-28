#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import signal
import serial
import time
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import BatteryState
from rcl_interfaces.msg import SetParametersResult

# Updated Hardware wiring: 
# Pins 0-9 are standard. Pin 10 is skipped (Bad Pin). 
# The last two joints now map to Pins 11 and 12 on your PCA9685.
ARDUINO_ORDER = [
    "rh_hip_joint",  "rh_upper_leg_joint",  "rh_lower_leg_joint",  # ch 0-2
    "lh_hip_joint",  "lh_upper_leg_joint",  "lh_lower_leg_joint",  # ch 3-5
    "lf_hip_joint",  "lf_upper_leg_joint",  "lf_lower_leg_joint",  # ch 6-8
    "rf_hip_joint",  "rf_upper_leg_joint",  "rf_lower_leg_joint",  # NOW MAPS TO ch 9, 11, 12
]

class JaxSerialBridge(Node):
    def __init__(self):
        super().__init__('jax_serial_bridge')

        # Declare parameters (Removed RF-specific thigh throttles)
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('command_alpha', 0.35)
        self.declare_parameter('joint_deadband', 0.01)
        self.declare_parameter('max_step', 0.01)
        self.declare_parameter('joint_offsets', [0.0] * len(ARDUINO_ORDER))

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.command_alpha = float(self.get_parameter('command_alpha').value)
        self.joint_deadband = float(self.get_parameter('joint_deadband').value)
        self.max_step = float(self.get_parameter('max_step').value)
        
        offsets_param = self.get_parameter('joint_offsets').value
        self.joint_offsets = [float(v) for v in offsets_param]
        
        self.add_on_set_parameters_callback(self.on_set_parameters)

        self.armed = False
        self.shutting_down = False
        self.ser = None
        self.target_values = [0.0] * len(ARDUINO_ORDER)
        self.filtered_values = [0.0] * len(ARDUINO_ORDER)
        self.last_sent_values = [0.0] * len(ARDUINO_ORDER)
        self.have_filter_state = False
        self.force_send = False

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

        self.create_timer(0.02, self.output_loop)
        self.create_timer(0.1, self.read_feedback)

    def traj_callback(self, msg):
        if self.shutting_down or not msg.points or self.ser is None:
            return

        if not self.armed:
            try:
                self.ser.write(b'WAKE\n')
                self.armed = True
                self.get_logger().info('JAX ARMED — All 12 servos synchronized')
            except Exception as e:
                self.get_logger().error(f'WAKE failed: {e}')
                return

        joint_map = dict(zip(msg.joint_names, msg.points[0].positions))
        self.target_values = [joint_map.get(name, 0.0) for name in ARDUINO_ORDER]

    def output_loop(self):
        if self.shutting_down or self.ser is None:
            return

        values = list(self.target_values)

        if not self.have_filter_state:
            self.filtered_values = list(values)
            self.last_sent_values = list(values)
            self.have_filter_state = True
            changed = True
        else:
            changed = False
            for i, target in enumerate(values):
                # All joints now use the same step and deadband
                step = self.max_step
                deadband = self.joint_deadband

                prev = self.filtered_values[i]
                blended = (self.command_alpha * target) + ((1.0 - self.command_alpha) * prev)
                diff = blended - prev
                
                if abs(diff) > step:
                    self.filtered_values[i] = prev + (step if diff > 0.0 else -step)
                else:
                    self.filtered_values[i] = blended

                if abs(self.filtered_values[i] - self.last_sent_values[i]) > deadband:
                    self.last_sent_values[i] = self.filtered_values[i]
                    changed = True

        if self.force_send:
            changed = True
            self.force_send = False

        if changed:
            adjusted = [self.last_sent_values[i] + self.joint_offsets[i] for i in range(len(ARDUINO_ORDER))]
            payload = 'J:' + ','.join(f'{v:.4f}' for v in adjusted) + '\n'
            try:
                self.ser.write(payload.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'Serial write error: {e}')

    def on_set_parameters(self, params):
        for p in params:
            if p.name == 'joint_offsets':
                self.joint_offsets = [float(v) for v in p.value]
                self.force_send = True
            elif p.name == 'command_alpha':
                self.command_alpha = float(p.value)
            elif p.name == 'joint_deadband':
                self.joint_deadband = float(p.value)
            elif p.name == 'max_step':
                self.max_step = float(p.value)
        return SetParametersResult(successful=True, reason='')

    def read_feedback(self):
        if self.ser is None: return
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('VOLT:'):
                    batt = BatteryState()
                    batt.voltage = float(line[5:])
                    self.battery_pub.publish(batt)
        except Exception: pass

    def begin_shutdown(self):
        if self.shutting_down: return
        self.shutting_down = True
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'SLEEP\n')
                self.ser.flush()
            except Exception: pass

    def destroy_node(self):
        self.begin_shutdown()
        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except Exception: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JaxSerialBridge()
    def _sigint_handler(sig, frame):
        node.begin_shutdown()
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _sigint_handler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()