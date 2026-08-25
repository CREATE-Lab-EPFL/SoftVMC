"""
Shared headless driver for button_pressing.py and person_avoiding.py.
Publishes /VC_Param and /VC_ref_pos for GUI_MultiVMC_Torque.py. Not run directly.

Reads whichever GUI_MultiVMC_Config.py is currently copied into place, so the
target, obstacle and stiffness always match the torque node's own config.

Phases, all time-based and identical across conditions:
  RAMP_GAUSS  strength 0 -> gauss_strength over GAUSS_RAMP_S, tip goal at rest
  APPROACH    tip goal from rest to PRESS_TARGET at APPROACH_V
  HOLD        steady state, hold_time seconds
  CONFIRM     hold the press, wait for ENTER  (CONFIRM_BEFORE_RETURN)
  RETRACT     tip goal back to rest -- home
  RELEASE     strength back to 0

Only the strength differs between runs. Ctrl+C the torque node afterwards to
trigger its exit-time save, then `python3 collect_run.py <folder> <name>`.
"""

import select
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from GUI_MultiVMC_Config import (
    DEFAULT_START, N_VC, DAMPER_PARAM_INIT, K_REACH, OBSTACLE_POS, PRESS_TARGET,
    CTRL_HZ, APPROACH_V, GAUSS_RAMP_S, HOLD_TIME, CONFIRM_BEFORE_RETURN,
)

STEP        = APPROACH_V / CTRL_HZ   # [m/tick]
POS_TOL     = 1e-4   # [m] convergence epsilon of the reference walk
FLUSH_TICKS = 10     # ticks of zero-strength publishing before exit


class ReachAvoidDriver(Node):

    def __init__(self, gauss_strength, hold_time=HOLD_TIME, label="reach_avoid"):
        super().__init__(f"{label}_driver")
        if N_VC != 3:
            raise RuntimeError(
                f"expected 3 components (lin_tip, gauss_seg1, gauss_seg2), got N_VC={N_VC}"
            )

        self._k_gauss_target = float(gauss_strength)
        self._hold_time      = float(hold_time)
        self._rest_tip       = DEFAULT_START[0].copy()

        self._ref    = DEFAULT_START.copy()
        self._ref[1] = OBSTACLE_POS
        self._ref[2] = OBSTACLE_POS
        self._k      = [float(K_REACH), 0.0, 0.0]

        self._pub_param  = self.create_publisher(Float64MultiArray, "/VC_Param",   10)
        self._pub_refpos = self.create_publisher(Float64MultiArray, "/VC_ref_pos", 10)

        self._ramp_ticks = max(1, int(round(GAUSS_RAMP_S * CTRL_HZ)))
        self._tick_n     = 0
        self._hold_start = None
        self._state      = "RAMP_GAUSS"
        self.done        = False

        self._timer = None
        self._publish()

        self.get_logger().info(
            f"[{label}] strength -> {self._k_gauss_target:.1f}, k_reach={K_REACH:.1f}, "
            f"target={np.round(PRESS_TARGET, 3).tolist()}, obstacle={np.round(OBSTACLE_POS, 3).tolist()}, "
            f"hold={self._hold_time:.0f}s"
        )

    def start(self):
        """Begin the sequence. Kept out of __init__ so that waiting for the
        torque node cannot advance the state machine while it spins."""
        self._timer = self.create_timer(1.0 / CTRL_HZ, self._tick)

    def _publish(self):
        p = Float64MultiArray()
        p.data = list(self._k) + list(DAMPER_PARAM_INIT)
        self._pub_param.publish(p)

        r = Float64MultiArray()
        r.data = self._ref.flatten().tolist()
        self._pub_refpos.publish(r)

    def _step_tip_toward(self, goal):
        """Move the tip reference one STEP toward goal. True once it is there."""
        delta = goal - self._ref[0]
        dist  = float(np.linalg.norm(delta))
        if dist <= POS_TOL:
            self._ref[0] = goal.copy()
            return True
        self._ref[0] = self._ref[0] + delta * min(1.0, STEP / dist)
        return False

    def _tick(self):
        if self.done:
            return

        if self._state == "RAMP_GAUSS":
            self._tick_n += 1
            frac = min(1.0, self._tick_n / self._ramp_ticks)
            self._k[1] = self._k[2] = frac * self._k_gauss_target
            if frac >= 1.0:
                self._state, self._tick_n = "APPROACH", 0
                self.get_logger().info("gauss at target strength -- approaching")

        elif self._state == "APPROACH":
            if self._step_tip_toward(PRESS_TARGET):
                self._state      = "HOLD"
                self._hold_start = time.time()
                self.get_logger().info(f"tip reference at target -- holding {self._hold_time:.0f}s")

        elif self._state == "HOLD":
            if time.time() - self._hold_start >= self._hold_time:
                if CONFIRM_BEFORE_RETURN:
                    self._state = "CONFIRM"
                    print("\n  hold complete -- did it touch? "
                          "press ENTER to return home ", end="", flush=True)
                else:
                    self._state = "RETRACT"
                    self.get_logger().info("hold complete -- retracting")

        elif self._state == "CONFIRM":
            # Polled, not input(): the reference keeps publishing at CTRL_HZ so
            # the arm holds its press steadily while waiting for the answer.
            if select.select([sys.stdin], [], [], 0.0)[0]:
                sys.stdin.readline()
                self._state = "RETRACT"
                self.get_logger().info("confirmed -- returning home")

        elif self._state == "RETRACT":
            if self._step_tip_toward(self._rest_tip):
                self._state, self._tick_n = "RELEASE", 0

        elif self._state == "RELEASE":
            self._tick_n += 1
            frac = min(1.0, self._tick_n / self._ramp_ticks)
            self._k[1] = self._k[2] = (1.0 - frac) * self._k_gauss_target
            if frac >= 1.0:
                self._k[1] = self._k[2] = 0.0
                self._state, self._tick_n = "FLUSH", 0

        elif self._state == "FLUSH":
            # keep publishing zeros briefly, otherwise shutdown can race the
            # last message and leave the torque node holding a residual field
            self._tick_n += 1
            if self._tick_n >= FLUSH_TICKS:
                self.get_logger().info(
                    "run complete -- Ctrl+C the torque node to save, then run collect_run.py"
                )
                self.done = True

        self._publish()


def _wait_for_torque_node(node, timeout=10.0):
    """Nothing consumes /VC_ref_pos except the torque node, so no subscriber
    means it is not running and the whole run would be published into the void."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if node._pub_refpos.get_subscription_count() > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return False


def run_reach_avoid(gauss_strength, hold_time=HOLD_TIME, label="reach_avoid"):
    rclpy.init()
    node = ReachAvoidDriver(gauss_strength, hold_time, label)
    if not _wait_for_torque_node(node):
        node.get_logger().error(
            "no subscriber on /VC_ref_pos -- is GUI_MultiVMC_Torque.py running? aborting."
        )
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)
    node.start()
    try:
        while rclpy.ok() and not node.done:
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
