"""
VMC_test_k.py
─────────────
Automated k-sweep test: for each k in [K_START, K_MAX], move the VC0 reference
from x=0 to X_TARGET and back, logging force vs position at each stiffness.

Sequence per k:
  1. Publish k, move ref from x=0 → X_TARGET at SPEED_OUT cm/s (-x direction)
  2. Hold at X_TARGET for HOLD_TICKS ticks (~1 s)
  3. Raise stiffness to K_RETURN, move ref back to x=0 at SPEED_BACK cm/s
  4. Hold at x=0 for HOLD_TICKS ticks (~1 s)
  5. k++ -> repeat

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

from GUI_MultiVMC_Config import DEFAULT_START, N_VC, DAMPER_PARAM_INIT

# ── Test parameters ────────────────────────────────────────────────────────────
K_START    = 1       # first stiffness value [N/m]
K_MAX      = 100     # last stiffness value  [N/m]
K_RETURN   = 250.0   # stiffness used to snap back to x=0
C_VALUES   = list(DAMPER_PARAM_INIT)   # damper values stay constant throughout

CTRL_HZ    = 100.0   # timer rate [Hz]
X_TARGET   = -0.30   # target x [m]   (-30 cm)
STEP_OUT   = -0.001  # x increment going out  [m/tick]  → 10 cm/s at 100 Hz
STEP_BACK  = +0.002  # x increment coming back [m/tick]  → 20 cm/s at 100 Hz
HOLD_TICKS = 100     # ticks to hold at each end (~1 s at 100 Hz)
X_TOL      = 0.0005  # arrival tolerance [m]
# ──────────────────────────────────────────────────────────────────────────────


class VMCTestK(Node):

    def __init__(self):
        super().__init__("vmc_test_k")

        self._pub_param  = self.create_publisher(Float64MultiArray, "/VC_Param",   10)
        self._pub_refpos = self.create_publisher(Float64MultiArray, "/VC_ref_pos", 10)

        # Reference positions for all components — only VC0's x is modified
        self._ref = DEFAULT_START.copy()   # (N_VC, 3)

        self._k    = float(K_START)
        self._done = False

        # Publish initial stiffness and reference position before the loop starts
        self._publish_param(self._k)
        self._publish_refpos()

        self._state      = "MOVING_OUT"
        self._hold_count = 0

        self.create_timer(1.0 / CTRL_HZ, self._tick)

        self.get_logger().info(
            f"VMCTestK ready — k={K_START}..{K_MAX}, "
            f"X_TARGET={X_TARGET} m, rate={CTRL_HZ:.0f} Hz"
        )

    # ── Publishing helpers ────────────────────────────────────────────────────

    def _publish_param(self, k_vc0: float):
        """[k0, k1, ..., c0, c1, ...]  — only VC0 stiffness varies."""
        k_vals = [k_vc0] + [0.0] * (N_VC - 1)   # VC1+ keep spring param = 0
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
                self._state      = "HOLD_AT_END"
                self._hold_count = 0
                self.get_logger().info(
                    f"[k={self._k:.0f}] reached x={X_TARGET:.3f} m — holding"
                )
            self._publish_refpos()

        elif self._state == "HOLD_AT_END":
            self._publish_refpos()
            self._hold_count += 1
            if self._hold_count >= HOLD_TICKS:
                self._publish_param(K_RETURN)
                self._state = "MOVING_BACK"
                self.get_logger().info(
                    f"[k={self._k:.0f}] stiffness → {K_RETURN:.0f}, moving back to x=0"
                )

        elif self._state == "MOVING_BACK":
            if x < -X_TOL:
                self._ref[0, 0] = min(x + STEP_BACK, 0.0)
            else:
                self._ref[0, 0] = 0.0
                self._state      = "HOLD_AT_START"
                self._hold_count = 0
                self.get_logger().info(
                    f"[k={self._k:.0f}] back at x=0 — holding"
                )
            self._publish_refpos()

        elif self._state == "HOLD_AT_START":
            self._publish_refpos()
            self._hold_count += 1
            if self._hold_count >= HOLD_TICKS:
                self._k += 1.0
                if self._k > K_MAX:
                    self.get_logger().info("Sweep complete.")
                    self._done = True
                    return
                self._publish_param(self._k)
                self._state = "MOVING_OUT"
                self.get_logger().info(
                    f"[k={self._k:.0f}] starting sweep"
                )


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
