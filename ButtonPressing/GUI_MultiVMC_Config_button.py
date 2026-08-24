import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
from VMC.MultiVMCTaskSpace import VirtualComponent
from VMC_utils.VirtualModels import (
    ConstrainedLinearSpring, LinearDamper, LinearSpring,
    TanhSpring, TanhDamper, GaussianSpring, DeadzoneLimitSpring
)

"""
Button pressing against the load cell. Copy into place before launching:
    cp GUI_MultiVMC_Config_button.py GUI_MultiVMC_Config.py

lin_tip (s=1.0) reaches for PRESS_TARGET with a moving reference; gauss_seg1
(s=1/3) and gauss_seg2 (s=2/3) are repelled by OBSTACLE_POS. Driven headlessly
by button_pressing.py -- positions and strengths are all hardcoded here.
"""

CONFIG_NAME = "button"   # guards against running a script against the wrong config

# ─────────────────────────────────────────────────────────────────────────────
# Reference positions
# ─────────────────────────────────────────────────────────────────────────────
BUTTON_POS   = np.array([0.05, 0.0, 0.38], dtype=float)
OBSTACLE_POS = np.array([0.07, 0.0, 0.22], dtype=float)

# Aim past the contact surface: at the surface the steady-state spring error,
# and the contact force with it, would go to zero once the tip arrives.
PRESS_DEPTH = 0.05   # [m] -> nominal press force = K_REACH * PRESS_DEPTH = 1.25 N

K_REACH = 25.0   # [N/m]
SIGMA   = 0.06   # [m]
C_DAMP  = 1.0    # [N.s/m]

_REST_TIP    = np.array([0.0, 0.0, 3 * 0.145], dtype=float)
_APPROACH_U  = (BUTTON_POS - _REST_TIP) / np.linalg.norm(BUTTON_POS - _REST_TIP)
PRESS_TARGET = BUTTON_POS + PRESS_DEPTH * _APPROACH_U

# ─────────────────────────────────────────────────────────────────────────────
# Virtual Components + Goals
# ─────────────────────────────────────────────────────────────────────────────
COMPONENT_SPECS = [
    dict(
        component=VirtualComponent(
            spring=LinearSpring(K_REACH),
            damper=LinearDamper(C_DAMP),
            s_param=1.0,
            name="lin_tip",
        ),
        goal=PRESS_TARGET,
    ),
    dict(
        component=VirtualComponent(
            # Always launch at 0.0 -- the driver ramps it. A Gaussian enabled at
            # full strength peaks immediately when ||delta|| is near sigma.
            spring=GaussianSpring(strength=0.0, sigma=SIGMA),
            damper=LinearDamper(C_DAMP),
            s_param=1.0 / 3.0,
            name="gauss_seg1",
        ),
        goal=OBSTACLE_POS,
    ),
    dict(
        component=VirtualComponent(
            spring=GaussianSpring(strength=0.0, sigma=SIGMA),
            damper=LinearDamper(C_DAMP),
            s_param=2.0 / 3.0,
            name="gauss_seg2",
        ),
        goal=OBSTACLE_POS,
    ),
]

# Both Gaussians share one strength. |tau_gauss| matches |tau_reach| at ~70 N/m
# for the geometry above, so these straddle the crossover (ratios 0 / 0.87 /
# 1.74 / 2.90). Re-check with `button_pressing.py --dry-run` after any edit.
GAUSS_CONDITIONS = {
    "no_gauss": 0.0,
    "mid_1":    60.0,
    "mid_2":    120.0,
    "strong":   200.0,
}

# ─────────────────────────────────────────────────────────────────────────────
# Derived: VIRTUAL_COMPONENTS / DEFAULT_GOALS / DEFAULT_START
# ─────────────────────────────────────────────────────────────────────────────
VIRTUAL_COMPONENTS = [spec["component"] for spec in COMPONENT_SPECS]
DEFAULT_GOALS      = np.array([spec["goal"] for spec in COMPONENT_SPECS], dtype=float)  # (N,3)


def _start_from_s_param(s: float) -> np.ndarray:
    """Straight-arm rest position for a given s_param in [0,1]."""
    if s <= 1.0 / 3.0:
        z = 3 * 0.145 * s
    elif s <= 2.0 / 3.0:
        z = 0.145 + 0.145 * (s - 1 / 3) * 3
    else:
        z = 2 * 0.145 + 0.145 * (s - 2 / 3) * 3
    return np.array([0.0, 0.0, z], dtype=float)


DEFAULT_START = np.array(
    [_start_from_s_param(comp.s_param) for comp in VIRTUAL_COMPONENTS],
    dtype=float,
)  # (N,3)

# ─────────────────────────────────────────────────────────────────────────────
# Subscriber / Controller Config
# ─────────────────────────────────────────────────────────────────────────────
TAU_LIMIT           = 0.25   # [N·m] per motor
SAVE_PLOT           = True
SAVE_DATA           = True   # save raw arrays to .npz alongside the plot
COLLECTED_DATA      = False  # keep False until the runs are recorded (see GUI_MultiVMC_Torque.py)
PLOT_Z              = True
PLOT_FORCE          = True   # load cell is used here
PLOT_VIRTUAL_FORCES = True

# ─────────────────────────────────────────────────────────────────────────────
# Pretension config
# tau_pretension[i] = PRETENSION_WEIGHTS[i] * q_motor[i]
#   applied only when q_motor[i] > PRETENSION_THRESHOLD (per motor).
# ─────────────────────────────────────────────────────────────────────────────
PRETENSION_WEIGHTS   = np.full(9, 0.03, dtype=float)  # [N·m/rad] x 9
PRETENSION_THRESHOLD = np.deg2rad(30.0)               # [rad]
PRETENSION_ENABLED   = True

# ─────────────────────────────────────────────────────────────────────────────
# ── GUI metadata ─────────────────────────────────────────────────────────────
# ─────────────────────────────────────────────────────────────────────────────
N_VC = len(VIRTUAL_COMPONENTS)

GUI_COLORS = ["red", "blue", "green", "darkorange", "purple", "cyan", "magenta"]


def _first_attr(obj, fallback_name: str, fallback_val: float):
    d = getattr(obj, "__dict__", {})
    if d:
        k = next(iter(d))
        return k, float(list(d.values())[0])
    return fallback_name, fallback_val


_spring_info = [_first_attr(c.spring, "k", 1.0) for c in VIRTUAL_COMPONENTS]
_damper_info = [_first_attr(c.damper, "c", 1.0) for c in VIRTUAL_COMPONENTS]

SPRING_PARAM_NAMES = [t[0] for t in _spring_info]
DAMPER_PARAM_NAMES = [t[0] for t in _damper_info]
SPRING_PARAM_INIT  = [t[1] for t in _spring_info]
DAMPER_PARAM_INIT  = [t[1] for t in _damper_info]

XY_RANGE        = (-0.15, 0.15)   # [m]
Z_RANGE         = (0.2, 0.55)     # [m]
TAU_PLOT_WINDOW = 10.0            # [s]
FK_S_SAMPLES    = 4
