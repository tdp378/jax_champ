import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class JaxAppController(Node):
    def __init__(self):
        super().__init__('jax_app_controller')

        # Initialize safely: Track the existing relaxed state from the bring-up node.
        # Jax will not move upon power-up until a specific command is given.
        self.current_state = "lay"

        # Listen to the joystick (velocity)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.vel_callback,
            10
        )

        # Listen to the UI buttons (sit, stand, walk, lay)
        self.mode_sub = self.create_subscription(
            String,
            '/jax_mode',
            self.mode_callback,
            10
        )

        self.get_logger().info("Jax App Controller active. Status: Laying down (motors relaxed). Waiting for command.")

    def mode_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f"Received UI Command: {command}")

        if command == "sit":
            self.current_state = "sitting"
            self.execute_sit()
        elif command == "stand":
            self.current_state = "standing"
            self.execute_stand()
        elif command == "walk":
            if self.current_state in ["sitting", "standing"]:
                self.current_state = "walking"
                self.get_logger().info("Transitioning to walking gait.")
            else:
                self.get_logger().warning("Cannot walk from current state. Stand up first.")
        elif command == "lay":
            self.current_state = "lay"
            self.execute_lay()

    def vel_callback(self, msg):
        # Ignore joystick input unless Jax is in an active stance
        if self.current_state not in ["walking", "standing"]:
            return

        linear_x = msg.linear.x  # Forward/Back from left stick
        linear_y = msg.linear.y  # Strafe from left stick
        angular_z = msg.angular.z # Turn from right stick

        self.update_gait_kinematics(linear_x, linear_y, angular_z)

    def update_gait_kinematics(self, x, y, z):
        # Calculate base angles for the front legs based on joystick input
        front_left_hip_angle = self.calculate_hip_angle(x, y, z, is_left=True)
        front_right_hip_angle = self.calculate_hip_angle(x, y, z, is_left=False)

        # The rear hips are mounted opposite to the front, 
        # so they require matching signs for correct movement.
        rear_left_hip_angle = front_left_hip_angle 
        rear_right_hip_angle = front_right_hip_angle 

        # TODO: Send front_left_hip_angle, rear_left_hip_angle, etc. to hardware PWM/motor controllers
        pass

    def execute_sit(self):
        self.get_logger().info("Engaging motors to sitting position.")
        # TODO: Send specific joint angles to transition from limp to crouched/sitting

    def execute_stand(self):
        self.get_logger().info("Standing up.")
        # TODO: Send specific joint angles to stand up

    def execute_lay(self):
        self.get_logger().info("Disengaging to limp noodle.")
        # TODO: Set motor torques to 0 / limp

    def calculate_hip_angle(self, x, y, z, is_left):
        # Placeholder for your actual inverse kinematics math
        return (x + y + z) * (1.0 if is_left else -1.0) 

def main(args=None):
    rclpy.init(args=args)
    node = JaxAppController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()