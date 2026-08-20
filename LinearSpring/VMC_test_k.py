"""
VMC_test_k.py
─────────────
Automated k-sweep test: for each k in [K_START, K_MAX]:
  1. Move ref from x=0 → X_TARGET incrementally (no hold)
  2. Move ref back to x=0 incrementally (no hold, same low k)
  3. Set k=K_RETURN, publish x=0 for SNAP_TICKS ticks (robot snaps home)
  4. k += K_STEP, repeat

TERMINAL 2 (torque backend must already be running):
    export PYTHONPATH="${PYTHONPATH}:$(pwd)"
    python3 VMC_test_k.py
"""

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

from GUI_MultiVMC_Config import DEFAULT_START, N_VC, DAMPER_PARAM_INIT, SPRING_PARAM_INIT

# ── Test parameters ────────────────────────────────────────────────────────────
K_START    = 1.7     # first stiffness value [N/m]
K_MAX      = 18.3    # last stiffness value  [N/m]
K_STEP     = 0.7     # stiffness increment per cycle
K_RETURN   = 66.7    # stiffness used to snap back to x=0
C_VALUES   = list(DAMPER_PARAM_INIT)

CTRL_HZ    = 10.0    # timer rate [Hz]
X_TARGET   = -0.25   # target x [m]
STEP_OUT   = -0.001  # x increment going out  [m/tick]
STEP_BACK  = +0.001  # x increment coming back [m/tick]
SNAP_TICKS = 50       # how many ticks to publish x=0 with K_RETURN
X_TOL      = 0.0005  # arrival tolerance [m]
# ──────────────────────────────────────────────────────────────────────────────


class VMCTestK(Node):

    def __init__(self):
        super().__init__("vmc_test_k")

        self._pub_param  = self.create_publisher(Float64MultiArray, "/VC_Param",   10)
        self._pub_refpos = self.create_publisher(Float64MultiArray, "/VC_ref_pos", 10)

        self._ref  = DEFAULT_START.copy()   # (N_VC, 3)
        self._k    = float(K_START)
        self._done = False
        self._snap_count = 0

        self._publish_param(self._k)
        self._publish_refpos()

        self._state = "MOVING_OUT"

        self.create_timer(1.0 / CTRL_HZ, self._tick)
        self.get_logger().info(
            f"VMCTestK ready — k={K_START}..{K_MAX} step={K_STEP}, "
            f"X_TARGET={X_TARGET} m, {CTRL_HZ:.0f} Hz"
        )

    # ── Publishing helpers ────────────────────────────────────────────────────

    def _publish_param(self, k_vc0: float):
        k_vals = [k_vc0] + list(SPRING_PARAM_INIT[1:])
        msg = Float64MultiArray()
        msg.data = k_vals + C_VALUES
        self._pub_param.publish(msg)

    def _publish_refpos(self):
        msg = Float64MultiArray()
        msg.data = self._ref.flatten().tolist()
        self._pub_refpos.publish(msg)

    # ── State machine ─────────────────────────────────────────────────────────

    def _tick(self):
        if self._done:
            return

        x = self._ref[0, 0]

        if self._state == "MOVING_OUT":
            if x > X_TARGET + X_TOL:
                self._ref[0, 0] = max(x + STEP_OUT, X_TARGET)
            else:
                self._ref[0, 0] = X_TARGET
                self._state = "MOVING_BACK"
                self.get_logger().info(f"[k={self._k:.0f}] x={X_TARGET} reached, moving back")
            self._publish_refpos()

        elif self._state == "MOVING_BACK":
            if x < -X_TOL:
                self._ref[0, 0] = min(x + STEP_BACK, 0.0)
                self._publish_refpos()
            else:
                self._ref[0, 0] = 0.0
                self._publish_refpos()
                self._publish_param(K_RETURN)
                self._snap_count = 0
                self._state = "SNAP_AT_ZERO"
                self.get_logger().info(f"[k={self._k:.0f}] x=0 reached, k→{K_RETURN:.0f}")

        elif self._state == "SNAP_AT_ZERO":
            self._publish_refpos()
            self._snap_count += 1
            if self._snap_count >= SNAP_TICKS:
                self._k += K_STEP
                if self._k > K_MAX:
                    self.get_logger().info("Sweep complete.")
                    self._done = True
                    return
                self._publish_param(self._k)
                self._state = "MOVING_OUT"
                self.get_logger().info(f"[k={self._k:.0f}] starting sweep")


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = VMCTestK()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.01)
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
