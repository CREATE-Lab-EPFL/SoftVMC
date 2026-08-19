"""
_press_repel_driver.py
───────────────────────
Shared headless driver for the "press + gauss_repel" two-component VMC
pattern used by both button_pressing.py (Fig 9) and person_avoiding.py
(Fig 10) -- same underlying experiment, different target position and
force-safety framing. Not meant to be run directly.

Publishes /VC_Param and /VC_ref_pos for GUI_MultiVMC_Torque.py, exactly like
GUI_MultiVMC.py or GaussianElement/moving_gauss_test.py would, but scripted
(hardcoded target + hardcoded gauss strength) rather than dragged live.

Sequence (mirrors GaussianElement/moving_gauss_test.py's safety pattern of
ramping position first, force second):
  1. APPROACH   -- step the press goal in a straight line from DEFAULT_START
                   to target_pos. gauss_repel's goal is set to target_pos
                   from the start, but its strength stays 0 throughout this
                   phase, so it is fully inert (F = strength*decay*delta = 0
                   whenever strength = 0, regardless of distance).
  2. RAMP_GAUSS -- once press has arrived at target_pos (current pos ~=
                   gauss_repel's obstacle pos, so delta ~= 0 and F ~= 0 at
                   any strength -- see VirtualModels.GaussianSpring), ramp
                   gauss_repel's strength from 0 up to gauss_strength over
                   GAUSS_RAMP_STEPS ticks.
  3. HOLD       -- stay at final strength for hold_time seconds so
                   GUI_MultiVMC_Torque.py logs a clean steady-state window.
                   Ctrl+C the torque node afterward to trigger its
                   exit-time auto-save (it only saves once, on exit).

press.stiffness is published once at start and held constant; press.max_force
and gauss_repel.sigma are NOT settable here -- only the first constructor
argument of each spring (stiffness, strength) is live/VC_Param-tunable (see
GUI_MultiVMC_Config.py's header note and GUI_MultiVMC_Torque.py:_cb_param).
Edit GUI_MultiVMC_Config.py directly and restart the torque node to change
those.

Assumes the arm is physically at (or very near) DEFAULT_START when this
script starts -- same assumption GaussianElement/moving_gauss_test.py makes.
Verify that assumption, or the APPROACH ramp starts from the wrong pose.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from GUI_MultiVMC_Config import DEFAULT_START, N_VC, SPRING_PARAM_INIT, DAMPER_PARAM_INIT

CTRL_HZ          = 10.0    # [Hz] timer rate
STEP             = 0.005   # [m/tick] approach speed (~5 cm/s)
POS_TOL          = 0.0005  # [m] arrival tolerance
GAUSS_RAMP_STEPS = 50       # ticks to ramp strength 0 -> target (5 s @ 10 Hz)


class PressRepelDriver(Node):
    """
    components[0] = press        (TanhSpring,    drives to target_pos)
    components[1] = gauss_repel  (GaussianSpring, obstacle = target_pos)
    Requires GUI_MultiVMC_Config.COMPONENT_SPECS to define exactly these two,
    in this order (checked in __init__).
    """

    def __init__(self, target_pos, gauss_strength, press_stiffness=None,
                 hold_time=15.0, label="press_repel"):
        super().__init__(f"{label}_driver")
        if N_VC != 2:
            raise RuntimeError(
                f"PressRepelDriver expects exactly 2 components (press, gauss_repel), "
                f"got N_VC={N_VC} -- check GUI_MultiVMC_Config.COMPONENT_SPECS."
            )

        self._target          = np.asarray(target_pos, dtype=float)
        self._k_gauss_target  = float(gauss_strength)
        self._k_press         = (float(press_stiffness) if press_stiffness is not None
                                  else float(SPRING_PARAM_INIT[0]))
        self._c_vals          = list(DAMPER_PARAM_INIT)
        self._hold_time       = float(hold_time)

        self._pub_param  = self.create_publisher(Float64MultiArray, "/VC_Param",   10)
        self._pub_refpos = self.create_publisher(Float64MultiArray, "/VC_ref_pos", 10)

        self._ref = DEFAULT_START.copy()   # (2, 3): [press, gauss_repel]
        self._ref[1] = self._target        # gauss_repel goal fixed at target from t=0

        self._k_gauss     = 0.0
        self._gauss_step  = 0
        self._hold_start  = None
        self.done         = False

        self._publish_params()
        self._publish_refpos()

        self._state = "APPROACH"
        self.create_timer(1.0 / CTRL_HZ, self._tick)

        dist = float(np.linalg.norm(self._target - self._ref[0]))
        self.get_logger().info(
            f"[{label}] target={self._target.tolist()}  "
            f"press.stiffness={self._k_press:.1f}  gauss.strength -> {self._k_gauss_target:.1f}  "
            f"approach dist={dist:.3f} m, hold={self._hold_time:.1f} s"
        )

    # ── Publish helpers ───────────────────────────────────────────────────────

    def _publish_params(self):
        msg = Float64MultiArray()
        msg.data = [self._k_press, self._k_gauss] + self._c_vals
        self._pub_param.publish(msg)

    def _publish_refpos(self):
        msg = Float64MultiArray()
        msg.data = self._ref.flatten().tolist()
        self._pub_refpos.publish(msg)

    # ── State machine ─────────────────────────────────────────────────────────

    def _tick(self):
        if self.done:
            return

        if self._state == "APPROACH":
            delta = self._target - self._ref[0]
            dist  = float(np.linalg.norm(delta))
            if dist > POS_TOL:
                self._ref[0] = self._ref[0] + delta * min(1.0, STEP / dist)
                self._publish_refpos()
            else:
                self._ref[0] = self._target.copy()
                self._publish_refpos()
                self._state = "RAMP_GAUSS"
                self.get_logger().info("Press arrived at target -- ramping gauss_repel strength")

        elif self._state == "RAMP_GAUSS":
            self._gauss_step += 1
            frac = min(1.0, self._gauss_step / GAUSS_RAMP_STEPS)
            self._k_gauss = frac * self._k_gauss_target
            self._publish_params()
            if frac >= 1.0:
                self._state = "HOLD"
                self._hold_start = time.time()
                self.get_logger().info(
                    f"gauss_repel strength -> {self._k_gauss_target:.1f} -- holding {self._hold_time:.1f} s"
                )

        elif self._state == "HOLD":
            if time.time() - self._hold_start >= self._hold_time:
                self.get_logger().info(
                    "Hold complete. Ctrl+C the torque node to save this run, then stop this script."
                )
                self.done = True


def run_press_repel(target_pos, gauss_strength, press_stiffness=None,
                     hold_time=15.0, label="press_repel"):
    rclpy.init()
    node = PressRepelDriver(target_pos, gauss_strength, press_stiffness, hold_time, label)
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
