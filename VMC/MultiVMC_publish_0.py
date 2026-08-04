"""
MutliVMC_publish_0.py
─────────────────────
Continuously publishes DEFAULT_START to /VC_ref_pos so the robot holds its
home pose while you test external perturbances (e.g. via `ros2 topic pub`
on /goal_torque).

TERMINAL 2 (torque backend must already be running):
    cd SoftVMC/MultiVMC
    export PYTHONPATH="${PYTHONPATH}:$(pwd)"
    python3 MutliVMC_publish_0.py
"""

import sys, os
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from GUI_MultiVMC_Config import DEFAULT_START

PUB_HZ = 10.0   # publish rate [Hz]


class HomeRefPublisher(Node):

    def __init__(self):
        super().__init__("home_ref_publisher")
        self._pub = self.create_publisher(Float64MultiArray, "/VC_ref_pos", 10)
        self._msg = Float64MultiArray()
        self._msg.data = DEFAULT_START.flatten().tolist()
        self.create_timer(1.0 / PUB_HZ, self._tick)
        self.get_logger().info(
            f"Publishing DEFAULT_START → /VC_ref_pos at {PUB_HZ:.0f} Hz: {self._msg.data}"
        )

    def _tick(self):
        self._pub.publish(self._msg)


def main():
    rclpy.init()
    node = HomeRefPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
