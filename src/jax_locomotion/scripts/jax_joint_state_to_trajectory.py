#!/usr/bin/env python3

from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JaxJointStateToTrajectory(Node):
    def __init__(self):
        super().__init__('jax_joint_state_to_trajectory')

        self.declare_parameter('input_topic', '/joint_states')
        self.declare_parameter('output_topic', '/jax/joint_commands/linkage_corrected')
        self.declare_parameter('time_from_start_sec', 0.10)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.time_from_start_sec = float(self.get_parameter('time_from_start_sec').value)

        self.sub = self.create_subscription(
            JointState,
            input_topic,
            self.joint_state_callback,
            10,
        )
        self.pub = self.create_publisher(JointTrajectory, output_topic, 10)

        self.get_logger().info(
            f'JointState->Trajectory bridge active: {input_topic} -> {output_topic}'
        )

    def joint_state_callback(self, msg: JointState):
        if not msg.name or not msg.position:
            return

        traj = JointTrajectory()
        traj.header = msg.header
        traj.joint_names = list(msg.name)

        point = JointTrajectoryPoint()
        point.positions = list(msg.position)
        point.time_from_start = Duration(sec=int(self.time_from_start_sec),
                                         nanosec=int((self.time_from_start_sec % 1.0) * 1e9))
        traj.points = [point]

        self.pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = JaxJointStateToTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
