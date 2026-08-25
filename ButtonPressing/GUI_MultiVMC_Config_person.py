"""
Reaching for the button with a person in the way. Select it with the `_person`
variant argument -- `python3 button_pressing.py _person`, and
`python3 ../VMC/GUI_MultiVMC_Torque.py --config _person` for the torque node.

Same structure and the same target as GUI_MultiVMC_Config_button.py: lin_tip
(s=1.0) reaches for the button, gauss_seg1 (s=1/3) and gauss_seg2 (s=2/3) are
repelled by OBSTACLE_POS. What changes is what the obstacle IS -- here it is a
person's arm, not a fixture -- so the reach is softer, the torque limit lower,
and there is no load cell. Driven headlessly by button_pressing.py.
"""

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
from VMC.MultiVMCTaskSpace import VirtualComponent
from VMC_utils.VirtualModels import LinearSpring, LinearDamper, GaussianSpring
from VMC_utils.ConfigHelpers import rest_pose, default_start, param_info

CONFIG_NAME = "person"   # guards against running a script against the wrong config

# ── Geometry ─────────────────────────────────────────────────────────────────
# Same values as GUI_MultiVMC_Config_button.py -- the target is always the
# button; only what sits in the way changes. Keep the two configs in step.
BUTTON_POS = np.array([-0.20, 0.0, 0.38], dtype=float)   # [m] target

# The PERSON is the obstacle: their arm is what the body has to avoid while the
# tip still reaches the button. Defaults to the button config's obstacle
# position -- a PLACEHOLDER. Measure where the person's arm actually is and set
# this before running near anyone.
PERSON_POS   = np.array([-0.05, 0.0, 0.20], dtype=float)   # [m] obstacle
OBSTACLE_POS = PERSON_POS

# Aim past the button: at its surface the steady-state spring error, and the
# contact force with it, would go to zero once the tip arrives.
PRESS_DEPTH = 0.02   # [m] -> nominal press force = K_REACH * PRESS_DEPTH = 0.20 N

_APPROACH_U  = BUTTON_POS - rest_pose(1.0)
PRESS_TARGET = BUTTON_POS + PRESS_DEPTH * _APPROACH_U / np.linalg.norm(_APPROACH_U)

# ── Virtual elements ─────────────────────────────────────────────────────────
# Matches the button config: it may never be stiffer than that one, since this
# is the variant that runs near a person. TAU_LIMIT below stays lower.
K_REACH = 10.0   # [N/m]
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
# Strength of the repulsion from the person. The reach/repulsion crossover is
# now 19.3 N/m, so both of these sit past it (2.1x, 4.1x); 0 / 25 / 50 would
# straddle it. Re-check with `button_pressing.py _person --dry-run`.
GAUSS_CONDITIONS = {
    "no_gauss": 0.0,
    "mid":      40.0,
    "strong":   80.0,
}

# Record this one first and confirm the arm stops clear before going lower.
SAFE_FIRST_CONDITION = "strong"

# ── Sequence -- read by _reach_avoid_driver.py ────────────────────────────────
CTRL_HZ      = 20.0   # [Hz]  driver publish rate
APPROACH_V   = 0.02   # [m/s] tip reference speed during APPROACH and RETRACT
GAUSS_RAMP_S = 5.0    # [s]   strength 0 -> condition, and back on release
HOLD_TIME    = 15.0   # [s]   steady state -- the measurement window

# After HOLD, hold the press and wait for ENTER before returning home, so the
# contact can be confirmed by eye first. False = retract as soon as HOLD ends.
CONFIRM_BEFORE_RETURN = True

# ── Controller ───────────────────────────────────────────────────────────────
TAU_LIMIT   = 0.15   # [N·m] per motor -- lower than the button config
GRAVITY_DIR = "-y"   # arm rotated onto its side: gravity pulls along -y, not -z

# tau_pretension[i] = PRETENSION_WEIGHTS[i] * q_motor[i], only above the threshold
PRETENSION_ENABLED   = True
PRETENSION_WEIGHTS   = np.full(9, 0.03, dtype=float)  # [N·m/rad] x 9
PRETENSION_THRESHOLD = np.deg2rad(30.0)               # [rad]

# ── Output ───────────────────────────────────────────────────────────────────
OUTPUT_FOLDER  = "person_avoiding"   # outputs/<folder>/<condition>.npz
SAVE_PLOT      = True
SAVE_DATA      = True    # raw arrays to .npz alongside the plot
COLLECTED_DATA = False   # keep False until the runs are recorded
PLOT_Z         = True
PLOT_FORCE     = False   # no load cell in this variant

# ── Derived ──────────────────────────────────────────────────────────────────
VIRTUAL_COMPONENTS = [spec["component"] for spec in COMPONENT_SPECS]
DEFAULT_GOALS      = np.array([spec["goal"] for spec in COMPONENT_SPECS], dtype=float)
DEFAULT_START      = default_start(VIRTUAL_COMPONENTS)
N_VC               = len(VIRTUAL_COMPONENTS)

(SPRING_PARAM_NAMES, DAMPER_PARAM_NAMES,
 SPRING_PARAM_INIT,  DAMPER_PARAM_INIT) = param_info(VIRTUAL_COMPONENTS)
