#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

PUBLISH_FREQUENCY = 330  # Hz
N_JOINTS = 9


class FakeJointStatePublisher(Node):
    def __init__(self):
        super().__init__('fake_joint_state_publisher')

        self.position_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_positions',
            10
        )

        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_velocities',
            10
        )

        self.timer = self.create_timer(
            1.0 / PUBLISH_FREQUENCY,
            self.publish_fake_values
        )

        self.get_logger().info(
            f'Publishing zero joint positions and velocities at {PUBLISH_FREQUENCY} Hz'
        )

    def publish_fake_values(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * N_JOINTS

        self.position_pub.publish(msg)
        self.velocity_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeJointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()