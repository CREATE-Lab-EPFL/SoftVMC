"""
button_pressing.py
──────────────────
Sequence driver for three button-pressing scenarios:
    avoid   — Gaussian repulsive field (button not contacted)
    exploit — linear spring drives end-effector into button
    minimize — tanh spring with bounded force

Usage:
    python3 button_pressing.py [--scenario avoid|exploit|minimize]

The script publishes VC_Param and VC_ref_pos on ROS 2 topics consumed by
GUI_MultiVMC_Torque.py.  Run from inside the VMC/ conda env ("finger").
"""

import argparse
import sys, os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

SCENARIOS = ("avoid", "exploit", "minimize")


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--scenario", choices=SCENARIOS, default="avoid")
    return p.parse_args()


class ButtonPressPublisher(Node):
    def __init__(self, scenario: str):
        super().__init__("button_press_publisher")
        self.scenario = scenario
        self._pub_param = self.create_publisher(Float64MultiArray, "/VC_Param", 10)
        self._pub_ref   = self.create_publisher(Float64MultiArray, "/VC_ref_pos", 10)
        self.create_timer(0.02, self._tick)  # 50 Hz
        self.get_logger().info(f"Button pressing — scenario: {scenario}")

    def _tick(self):
        # TODO: implement scenario-specific param publishing
        pass


def main():
    args = parse_args()
    rclpy.init()
    node = ButtonPressPublisher(args.scenario)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
