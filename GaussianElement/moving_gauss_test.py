"""
moving_gauss_test.py
────────────────────
Single-circle trajectory: Gaussian spring moves from far (x=-40cm) to the
circle entry (x=-10cm), traces one full circle of radius 10cm around the
origin, then stays.  Linear spring holds at k=K_LIN throughout.

Sequence:
  1. INIT_MOVE_FAR : move gauss goal (0,0) → GAUSS_FAR_X, strength=0
                     → on arrival: enable strength=K_GAUSS
  2. APPROACH      : step gauss x from GAUSS_FAR_X → CIRCLE_X
  3. CIRCLING      : full circle, log every 90°, then stay (done)

TERMINAL 2 (torque backend must already be running):
    cd GaussianElement
    export PYTHONPATH="${PYTHONPATH}:$(pwd)/.."
    python3 moving_gauss_test.py
"""

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

from GUI_MultiVMC_Config import DEFAULT_START, N_VC, DAMPER_PARAM_INIT

# ── Test parameters ────────────────────────────────────────────────────────────
CTRL_HZ        = 10.0    # [Hz]  timer rate

K_LIN          = 15.0    # linear spring stiffness
K_GAUSS        = 1200.0  # gaussian strength once enabled

GAUSS_FAR_X    = -0.40   # [m] far start position (strength disabled here)
CIRCLE_X       = -0.10   # [m] x entry point for circle
CIRCLE_R       = 0.10    # [m] circle radius, centred at origin
N_CIRCLE_STEPS = 200     # steps per full revolution

STEP           = 0.005   # [m/tick] step size for all linear moves
X_TOL          = 0.0005  # [m] arrival tolerance

C_VALUES = list(DAMPER_PARAM_INIT)   # [c_lin, c_gauss]
# ──────────────────────────────────────────────────────────────────────────────

# Circle waypoints: entry at angle=π → (-CIRCLE_R, 0), one full clockwise lap.
_angles    = np.pi + 2.0 * np.pi * np.arange(N_CIRCLE_STEPS) / N_CIRCLE_STEPS
CIRCLE_WPS = np.stack([CIRCLE_R * np.cos(_angles),
                        CIRCLE_R * np.sin(_angles)], axis=1)  # (N, 2)

# Steps at which to log: 0°, 90°, 180°, 270° into the circle
_LOG_STEPS = {int(round(N_CIRCLE_STEPS * q / 4)): f"{q*90}°"
              for q in range(4)}


class MovingGaussTest(Node):

    def __init__(self):
        super().__init__("moving_gauss_test")

        self._pub_param  = self.create_publisher(Float64MultiArray, "/VC_Param",   10)
        self._pub_refpos = self.create_publisher(Float64MultiArray, "/VC_ref_pos", 10)

        # VC0 = lin_seg3, VC1 = gauss_seg3
        self._ref     = DEFAULT_START.copy()          # (N_VC=2, 3)
        self._gauss_z = float(DEFAULT_START[1, 2])    # z stays fixed

        self._k_lin   = K_LIN
        self._k_gauss = 0.0    # disabled until gauss reaches far position

        self._circle_step = 0
        self._done        = False

        self._publish_params()
        self._publish_refpos()

        self._state = "INIT_MOVE_FAR"
        self.create_timer(1.0 / CTRL_HZ, self._tick)

        self.get_logger().info(
            f"MovingGaussTest — k_lin={K_LIN}, k_gauss→{K_GAUSS} after far, "
            f"circle R={CIRCLE_R} m × {N_CIRCLE_STEPS} steps"
        )

    # ── Publish helpers ───────────────────────────────────────────────────────

    def _publish_params(self):
        msg = Float64MultiArray()
        msg.data = [self._k_lin, self._k_gauss] + C_VALUES
        self._pub_param.publish(msg)

    def _publish_refpos(self):
        msg = Float64MultiArray()
        msg.data = self._ref.flatten().tolist()
        self._pub_refpos.publish(msg)

    def _set_gauss(self, x: float, y: float):
        self._ref[1, 0] = x
        self._ref[1, 1] = y
        self._ref[1, 2] = self._gauss_z

    # ── State machine ─────────────────────────────────────────────────────────

    def _tick(self):
        if self._done:
            return

        # INIT_MOVE_FAR — slide gauss goal to far position with strength=0
        if self._state == "INIT_MOVE_FAR":
            x = self._ref[1, 0]
            if x > GAUSS_FAR_X + X_TOL:
                self._set_gauss(max(x - STEP, GAUSS_FAR_X), 0.0)
                self._publish_refpos()
            else:
                self._set_gauss(GAUSS_FAR_X, 0.0)
                self._publish_refpos()
                self._k_gauss = K_GAUSS
                self._publish_params()
                self._state = "APPROACH"
                self.get_logger().info(
                    f"Gauss at far ({GAUSS_FAR_X} m), strength→{K_GAUSS:.0f}, approaching"
                )

        # APPROACH — step gauss toward circle entry
        elif self._state == "APPROACH":
            x = self._ref[1, 0]
            if x < CIRCLE_X - X_TOL:
                self._set_gauss(min(x + STEP, CIRCLE_X), 0.0)
                self._publish_refpos()
            else:
                self._set_gauss(CIRCLE_X, 0.0)
                self._publish_refpos()
                self._circle_step = 0
                self._state = "CIRCLING"
                self.get_logger().info(
                    f"Circle entry ({CIRCLE_X} m), starting orbit"
                )

        # CIRCLING — publish each waypoint; log every 90°; stay when done
        elif self._state == "CIRCLING":
            if self._circle_step in _LOG_STEPS:
                wp = CIRCLE_WPS[self._circle_step]
                deg = _LOG_STEPS[self._circle_step]
                self.get_logger().info(
                    f"Circle {deg}: gauss goal = ({wp[0]:.3f}, {wp[1]:.3f}) m"
                )
            wp = CIRCLE_WPS[self._circle_step]
            self._set_gauss(float(wp[0]), float(wp[1]))
            self._publish_refpos()
            self._circle_step += 1
            if self._circle_step >= N_CIRCLE_STEPS:
                self._set_gauss(CIRCLE_X, 0.0)
                self._publish_refpos()
                self.get_logger().info("Circle complete — gaussian stays. Experiment done.")
                self._done = True


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = MovingGaussTest()
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
