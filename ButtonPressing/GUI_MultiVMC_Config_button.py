"""
Button pressing against the load cell. Select it with the `_button` variant
argument -- `python3 button_pressing.py _button`, and
`python3 ../VMC/GUI_MultiVMC_Torque.py --config _button` for the torque node.

lin_tip (s=1.0) reaches for PRESS_TARGET with a moving reference; gauss_seg1
(s=1/3) and gauss_seg2 (s=2/3) are repelled by OBSTACLE_POS. Everything the run
depends on is in this file -- _reach_avoid_driver.py, button_pressing.py and
_geometry_check.py all read it, so nothing can disagree.

Headless: no GUI, hence no GUI slider metadata here.
"""

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
from VMC.MultiVMCTaskSpace import VirtualComponent
from VMC_utils.VirtualModels import LinearSpring, LinearDamper, GaussianSpring
from VMC_utils.ConfigHelpers import rest_pose, default_start, param_info

CONFIG_NAME = "button"   # guards against running a script against the wrong config

# ── Geometry ─────────────────────────────────────────────────────────────────
BUTTON_POS   = np.array([-0.20, 0.0, 0.38], dtype=float)   # [m]
OBSTACLE_POS = np.array([-0.05, 0.0, 0.20], dtype=float)   # [m]

# Aim past the contact surface: at the surface the steady-state spring error,
# and the contact force with it, would go to zero once the tip arrives.
PRESS_DEPTH = 0.02   # [m] -> nominal press force = K_REACH * PRESS_DEPTH = 0.50 N

_APPROACH_U  = BUTTON_POS - rest_pose(1.0)
PRESS_TARGET = BUTTON_POS + PRESS_DEPTH * _APPROACH_U / np.linalg.norm(_APPROACH_U)

# ── Virtual elements ─────────────────────────────────────────────────────────
K_REACH = 25.0   # [N/m]
SIGMA   = 0.06   # [m]
C_DAMP  = 1.0    # [N.s/m]

COMPONENT_SPECS = [
    dict(component=VirtualComponent(spring=LinearSpring(K_REACH),
                                    damper=LinearDamper(C_DAMP),
                                    s_param=1.0, name="lin_tip"),
         goal=PRESS_TARGET),
    # Both Gaussians launch at 0.0 -- the driver ramps them. Enabled at full
    # strength one would peak immediately, with ||delta|| already near sigma.
    dict(component=VirtualComponent(spring=GaussianSpring(strength=0.0, sigma=SIGMA),
                                    damper=LinearDamper(C_DAMP),
                                    s_param=1.0 / 3.0, name="gauss_seg1"),
         goal=OBSTACLE_POS),
    dict(component=VirtualComponent(spring=GaussianSpring(strength=0.0, sigma=SIGMA),
                                    damper=LinearDamper(C_DAMP),
                                    s_param=2.0 / 3.0, name="gauss_seg2"),
         goal=OBSTACLE_POS),
]

# ── Conditions ───────────────────────────────────────────────────────────────
# Both Gaussians share one strength. |tau_gauss| matches |tau_reach| at ~70 N/m
# for the geometry above, so these straddle the crossover (ratios 0 / 0.87 /
# 1.74 / 2.90). Re-check with `button_pressing.py _button --dry-run` after any edit.
GAUSS_CONDITIONS = {
    "no_gauss": 0.0,
    "mid_1":    60.0,
    "mid_2":    120.0,
    "strong":   200.0,
}

# ── Sequence -- read by _reach_avoid_driver.py ────────────────────────────────
CTRL_HZ      = 20.0   # [Hz]  driver publish rate
APPROACH_V   = 0.02   # [m/s] tip reference speed during APPROACH and RETRACT
GAUSS_RAMP_S = 5.0    # [s]   strength 0 -> condition, and back on release
HOLD_TIME    = 15.0   # [s]   steady state -- the measurement window

# After HOLD, hold the press and wait for ENTER before returning home, so the
# contact can be confirmed by eye first. False = retract as soon as HOLD ends.
CONFIRM_BEFORE_RETURN = True

# ── Controller ───────────────────────────────────────────────────────────────
TAU_LIMIT   = 0.25   # [N·m] per motor
GRAVITY_DIR = "-y"   # arm rotated onto its side: gravity pulls along -y, not -z

# tau_pretension[i] = PRETENSION_WEIGHTS[i] * q_motor[i], only above the threshold
PRETENSION_ENABLED   = True
PRETENSION_WEIGHTS   = np.full(9, 0.03, dtype=float)  # [N·m/rad] x 9
PRETENSION_THRESHOLD = np.deg2rad(30.0)               # [rad]

# ── Output ───────────────────────────────────────────────────────────────────
OUTPUT_FOLDER  = "button_pressing"   # outputs/<folder>/<condition>.npz
SAVE_PLOT      = True
SAVE_DATA      = True    # raw arrays to .npz alongside the plot
COLLECTED_DATA = False   # keep False until the runs are recorded
PLOT_Z         = True
PLOT_FORCE     = True    # load cell is used here

# ── Derived ──────────────────────────────────────────────────────────────────
VIRTUAL_COMPONENTS = [spec["component"] for spec in COMPONENT_SPECS]
DEFAULT_GOALS      = np.array([spec["goal"] for spec in COMPONENT_SPECS], dtype=float)
DEFAULT_START      = default_start(VIRTUAL_COMPONENTS)
N_VC               = len(VIRTUAL_COMPONENTS)

(SPRING_PARAM_NAMES, DAMPER_PARAM_NAMES,
 SPRING_PARAM_INIT,  DAMPER_PARAM_INIT) = param_info(VIRTUAL_COMPONENTS)
