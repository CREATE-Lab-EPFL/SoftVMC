"""
MultiVMC_publish_0.py
─────────────────────
Continuously publishes DEFAULT_START to /VC_ref_pos so the robot holds its
home pose while you test external perturbances (e.g. via `ros2 topic pub`
on /goal_torque).

TERMINAL 2 (torque backend must already be running, run from the experiment directory):
    export PYTHONPATH="${PYTHONPATH}:$(pwd)/.."
    python3 ../VMC/MultiVMC_publish_0.py
"""

import sys, os
# The caller's cwd first, so an experiment folder's config wins over VMC/'s.
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
sys.path.insert(0, os.getcwd())
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path: sys.path.insert(0, _ROOT)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from VMC_utils.ConfigHelpers import config_from_argv

# --config NAME picks GUI_MultiVMC_Config_NAME.py from the launch directory;
# without it the plain GUI_MultiVMC_Config.py is used, as before.
_cfg = config_from_argv()

DEFAULT_START = _cfg.DEFAULT_START

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
