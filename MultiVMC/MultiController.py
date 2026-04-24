import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from collections import deque
import time

# Control frequency [Hz]
CONTROL_FREQUENCY = 330

class MultiController(Node):
    """
    ROS2 node interface for multi segment.
    Reads joint states, publishes torques.
    Expected ordering:
      [q11,q12,q13, q21,q22,q23, q31,q32,q33]
    """

    def __init__(self):
        super().__init__('multi_controller')

        self.torque_pub   = self.create_publisher(Float64MultiArray, '/goal_torque', 10)
        self.position_sub = self.create_subscription(Float64MultiArray, '/joint_positions', self._pos_cb, 10)
        self.velocity_sub = self.create_subscription(Float64MultiArray, '/joint_velocities', self._vel_cb, 10)

        self.joint_positions  = np.zeros(9, dtype=float)  # [deg] or [rad] based on your sim config
        self.joint_velocities = np.zeros(9, dtype=float)  # [deg/s] or [rad/s]

        # # ---- Frequency monitor for /joint_positions ----
        # self._jp_last_t = None
        # self._jp_periods = deque(maxlen=50)   # moving window size
        # self._jp_log_every_sec = 1.0          # print once per second
        # self._jp_last_log_t = time.perf_counter()

    def _pos_cb(self, msg):
        self.joint_positions = np.array(msg.data)

        # now = time.perf_counter()
        # if self._jp_last_t is not None:
        #     dt = now - self._jp_last_t
        #     if dt > 0:
        #         self._jp_periods.append(dt)

        # self._jp_last_t = now

        # # periodic info log
        # if len(self._jp_periods) > 0 and (now - self._jp_last_log_t) >= self._jp_log_every_sec:
        #     avg_dt = sum(self._jp_periods) / len(self._jp_periods)
        #     hz = 1.0 / avg_dt if avg_dt > 0 else float('inf')
        #     self.get_logger().info(f"/joint_positions frequency: {hz:.2f} Hz")
        #     self._jp_last_log_t = now

    def _vel_cb(self, msg): self.joint_velocities = np.array(msg.data)

    def get_joint_positions(self):  return self.joint_positions
    def get_joint_velocities(self): return self.joint_velocities

    def publish_torques(self, torques):
        msg = Float64MultiArray()
        msg.data = list(torques)
        self.torque_pub.publish(msg)