"""
test_disturbance_diff_k.py
──────────────────────────
Automated k-sweep test: for each k in [K_START, K_MAX]:
  1. Move ref from x=0 → X_TARGET incrementally at current k
  2. Move ref back from X_TARGET → x=0 at the same k
  3. Snap to x=0 at K_RETURN for SNAP_TICKS ticks (robot snaps home)
  4. k += K_STEP, brief settle at new k, repeat

TERMINAL 2 (torque backend must already be running):
    export PYTHONPATH="${PYTHONPATH}:$(pwd)"
    python3 plots/perturbance_rejection/test_disturbance_diff_k.py
"""

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

from GUI_MultiVMC_Config import DEFAULT_START, N_VC, DAMPER_PARAM_INIT, SPRING_PARAM_INIT

# ── Test parameters ────────────────────────────────────────────────────────────
K_START    = 15      # first stiffness value [N/m]
K_MAX      = 55     # last stiffness value  [N/m]
K_STEP     = 10      # stiffness increment per cycle
K_HOME     = 200.0  # stiffness used only during the initial home-snap
K_RETURN   = 200.0  # stiffness used to snap the robot back to x=0 between cycles
C_VALUES   = list(DAMPER_PARAM_INIT)

CTRL_HZ    = 10.0    # timer rate [Hz]
X_TARGET   = -0.25   # target x [m]
STEP_OUT   = -0.001  # x increment going out  [m/tick]
STEP_BACK  = +0.001  # x increment coming back [m/tick]
X_TOL      = 0.0005  # arrival tolerance [m]
HOME_TICKS = 80      # ticks to hold x=0 at K_HOME before sweep starts (~8 s)
SNAP_TICKS = 50      # ticks to hold x=0 at K_RETURN between cycles (~5 s)
HOLD_TICKS = 20      # ticks to hold x=0 at the new (low) k before MOVING_OUT (~2 s)
# ──────────────────────────────────────────────────────────────────────────────


class VMCTestK(Node):

    def __init__(self):
        super().__init__("vmc_test_k")

        self._pub_param  = self.create_publisher(Float64MultiArray, "/VC_Param",   10)
        self._pub_refpos = self.create_publisher(Float64MultiArray, "/VC_ref_pos", 10)

        self._ref  = DEFAULT_START.copy()   # (N_VC, 3)
        self._k    = float(K_START)
        self._done = False
        self._home_count = 0
        self._snap_count = 0
        self._hold_count = 0

        self._publish_param(K_HOME)   # stiff snap to origin before sweep
        self._publish_refpos()

        self._state = "HOMING"

        self.create_timer(1.0 / CTRL_HZ, self._tick)
        self.get_logger().info(
            f"VMCTestK ready — homing for {HOME_TICKS} ticks, "
            f"then sweep k={K_START} to {K_MAX} with step={K_STEP}, "
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

        # Hold x=0 with a stiff spring until the robot settles, then begin sweep
        if self._state == "HOMING":
            self._publish_refpos()
            self._home_count += 1
            if self._home_count >= HOME_TICKS:
                self._publish_param(self._k)
                self._state = "MOVING_OUT"
                self.get_logger().info(
                    f"Home settled — starting sweep k={self._k:.0f}..{K_MAX:.0f}"
                )
            return

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
                self.get_logger().info(
                    f"[k={self._k:.0f}] x=0 reached, snapping with k={K_RETURN:.0f}"
                )

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
                self._hold_count = 0
                self._state = "HOLD_AT_ZERO"
                self.get_logger().info(f"[k={self._k:.0f}] holding at x=0")

        elif self._state == "HOLD_AT_ZERO":
            self._publish_refpos()
            self._hold_count += 1
            if self._hold_count >= HOLD_TICKS:
                self._state = "MOVING_OUT"
                self.get_logger().info(f"[k={self._k:.0f}] starting next cycle")


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
