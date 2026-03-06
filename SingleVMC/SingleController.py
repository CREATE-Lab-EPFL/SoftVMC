import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Control frequency [Hz]
CONTROL_FREQUENCY = 500

class SingleController(Node):
    """ROS2 node interface for single segment. Reads joint states, publishes torques."""

    def __init__(self):
        super().__init__('single_controller')

        self.torque_pub   = self.create_publisher(Float64MultiArray, '/goal_torque', 10)
        self.position_sub = self.create_subscription(Float64MultiArray, '/joint_positions',
                                                     self._pos_cb, 10)
        self.velocity_sub = self.create_subscription(Float64MultiArray, '/joint_velocities',
                                                     self._vel_cb, 10)
        self.joint_positions  = np.zeros(3)   # [deg]
        self.joint_velocities = np.zeros(3)   # [deg/s]

    def _pos_cb(self, msg):   self.joint_positions  = np.array(msg.data)
    def _vel_cb(self, msg):   self.joint_velocities = np.array(msg.data)

    def get_joint_positions(self):  return self.joint_positions
    def get_joint_velocities(self): return self.joint_velocities

    def publish_torques(self, torques):
        msg = Float64MultiArray()
        msg.data = list(torques)
        self.torque_pub.publish(msg)
