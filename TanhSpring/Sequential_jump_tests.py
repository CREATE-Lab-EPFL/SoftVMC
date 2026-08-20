"""
Sequential_jump_tests.py
─────────────────────────
Test tanh spring saturation with a rectangular reference profile.
For k=10, instantaneously jump x to progressively larger displacements
(-5cm, -10cm, ..., -30cm), HOLD at each, then jump back to x=0 (with high
k=600) and HOLD, before jumping to the next deeper target.

Reference profile (everything is an instantaneous jump, no ramping):

      0 ─┐        ┌──┐        ┌──┐
         │        │  │        │  │
  -5cm   └────────┘  │        │  │      (hold HOLD_TARGET, k=K_TEST)
                     │        │  │
 -10cm               └────────┘  │      ...
                                 ...

Pattern per target:
  1. Jump to x_target, hold HOLD_TARGET ticks  (k = K_TEST)
  2. Jump to x=0,       hold HOLD_ZERO   ticks  (k = K_RETURN)
  3. Repeat for next (deeper) target until all done

TERMINAL 2 (torque backend must already be running):
    export PYTHONPATH="${PYTHONPATH}:$(pwd)"
    python3 Sequential_jump_tests.py
"""

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

from GUI_MultiVMC_Config import DEFAULT_START, DAMPER_PARAM_INIT, SPRING_PARAM_INIT

# ── Test parameters ────────────────────────────────────────────────────────────
K_TEST     = 10.0    # stiffness for main jumps [N/m]
K_RETURN   = 65.0    # stiffness to snap back to x=0 [N/m]
C_VALUES   = list(DAMPER_PARAM_INIT)

CTRL_HZ     = 10.0    # timer rate [Hz]
TARGETS     = [-0.05, -0.10, -0.15, -0.20, -0.25, -0.30, -0.35, -0.40, -0.45, -0.50]  # x targets [m]
HOLD_TARGET = 30      # ticks to HOLD at x_target (k=K_TEST)
HOLD_ZERO   = 20      # ticks to HOLD at x=0     (k=K_RETURN)
# ──────────────────────────────────────────────────────────────────────────────


class SequentialJumpTest(Node):

    def __init__(self):
        super().__init__("sequential_jump_test")

        self._pub_param  = self.create_publisher(Float64MultiArray, "/VC_Param",   10)
        self._pub_refpos = self.create_publisher(Float64MultiArray, "/VC_ref_pos", 10)

        self._ref  = DEFAULT_START.copy()   # (N_VC, 3)
        self._done = False
        self._target_idx = 0
        self._step_count = 0

        # Start: jump to first target, hold there with K_TEST
        self._ref[0, 0] = TARGETS[0]
        self._publish_param(K_TEST)
        self._publish_refpos()

        self._state = "HOLD_TARGET"

        self.create_timer(1.0 / CTRL_HZ, self._tick)
        self.get_logger().info(
            f"SequentialJumpTest ready — k_test={K_TEST} N/m, k_return={K_RETURN} N/m, "
            f"targets={[f'{t:.2f}' for t in TARGETS]}m, {CTRL_HZ:.0f} Hz"
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

        if self._target_idx >= len(TARGETS):
            self.get_logger().info("All targets reached. Test complete.")
            self._done = True
            return

        x_target = TARGETS[self._target_idx]

        if self._state == "HOLD_TARGET":
            # Hold at x_target for HOLD_TARGET ticks (reference stays put)
            self._ref[0, 0] = x_target
            self._publish_refpos()
            self._step_count += 1
            if self._step_count >= HOLD_TARGET:
                # Jump back to x=0 and switch to high return stiffness
                self._ref[0, 0] = 0.0
                self._publish_param(K_RETURN)
                self._publish_refpos()
                self._state = "HOLD_ZERO"
                self._step_count = 0
                self.get_logger().info(
                    f"Target {self._target_idx + 1}/{len(TARGETS)} "
                    f"(x={x_target*1000:.0f}mm) held, jumped to x=0 [k={K_RETURN}]"
                )

        elif self._state == "HOLD_ZERO":
            # Hold at x=0 for HOLD_ZERO ticks (snap back / settle)
            self._ref[0, 0] = 0.0
            self._publish_refpos()
            self._step_count += 1
            if self._step_count >= HOLD_ZERO:
                self._target_idx += 1
                if self._target_idx < len(TARGETS):
                    # Jump to next deeper target with K_TEST
                    self._ref[0, 0] = TARGETS[self._target_idx]
                    self._publish_param(K_TEST)
                    self._publish_refpos()
                    self._state = "HOLD_TARGET"
                    self._step_count = 0
                    self.get_logger().info(
                        f"Jumped to target {self._target_idx + 1}/{len(TARGETS)} "
                        f"→ {TARGETS[self._target_idx]*1000:.0f}mm [k={K_TEST}]"
                    )
                else:
                    self.get_logger().info("All targets complete. Sequence finished.")
                    self._done = True


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = SequentialJumpTest()
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
