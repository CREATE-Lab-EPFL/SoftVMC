import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
from VMC.MultiVMCTaskSpace import VirtualComponent
from VMC_utils.VirtualModels import (
    ConstrainedLinearSpring, LinearDamper, LinearSpring,
    TanhSpring, TanhDamper, GaussianSpring, DeadzoneLimitSpring
)

"""
Config for button-pressing VMC experiments — shared by GUI_MultiVMC_Torque.py
and GUI_MultiVMC.py.

Two components summed at the same body point (s_param=1.0), both targeting
button_pos, tau = J1^T@F1 + J2^T@F2:
    press        — TanhSpring: drives the tip into the button; saturates at
                   max_force, so that value IS the calibrated full-contact
                   force target (bounded/safe regardless of the Gaussian below).
    gauss_repel  — GaussianSpring: repels the tip away from button_pos. Sweeping
                   its strength from 0 (full contact, press's max_force is felt
                   directly) up to a value that fully cancels the approach
                   (no contact) produces the force family for Fig 9 -- e.g. 4
                   runs: no-contact / middle / middle / contact.

NOTE on what is actually live-tunable: GUI_MultiVMC_Torque.py's /VC_Param
handler only setattr()s each spring's FIRST constructor argument (see
_first_attr below). That means press.stiffness and gauss_repel.strength
ARE live/GUI-slider-tunable at runtime, but press.max_force and
gauss_repel.sigma are NOT -- those two only change by editing the
constructor calls below and restarting the torque node.

Both components' goals must stay coincident at button_pos in the GUI (each has
its own draggable marker) -- only the gauss_repel strength slider should move
between runs, not either goal marker.

REQUIREMENT: len(COMPONENT_SPECS) == number of virtual components == number of goals.
"""

# ─────────────────────────────────────────────────────────────────────────────
# Reference positions
# ─────────────────────────────────────────────────────────────────────────────
button_pos = np.array([0.05, 0.0, 0.38], dtype=float)  # measured on the real robot

# ─────────────────────────────────────────────────────────────────────────────
# Virtual Components + Goals
# ─────────────────────────────────────────────────────────────────────────────
COMPONENT_SPECS = [
    dict(
        component=VirtualComponent(
            # UNTESTED placeholders. stiffness is live/GUI-tunable -- start low
            # and raise while watching /force_shear. max_force is NOT live --
            # edit it here and restart once you know the target contact force.
            spring=TanhSpring(stiffness=20.0, max_force=1.0),
            damper=LinearDamper(0.5),
            s_param=1.0,
            name="press",
        ),
        goal=button_pos,  # GUI OVERRIDES THIS GOAL --- IGNORE ---
    ),
    dict(
        component=VirtualComponent(
            # GaussianSpring produces max force when sigma == delta_x, so it can
            # produce very large forces at initialization -- start with strength=0.0
            # and raise it only once the reference is moved away from the robot.
            spring=GaussianSpring(strength=0.0, sigma=0.03),
            damper=LinearDamper(0.5),
            s_param=1.0,
            name="gauss_repel",
        ),
        goal=button_pos,  # GUI OVERRIDES THIS GOAL --- IGNORE ---
    ),
]

# ─────────────────────────────────────────────────────────────────────────────
# Derived: VIRTUAL_COMPONENTS / DEFAULT_START
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
TAU_LIMIT           = 1.5   # [N·m] per motor
SAVE_PLOT           = True
SAVE_DATA           = True   # save raw arrays to .npz alongside the plot
COLLECTED_DATA      = False  # no real data collected yet -- keep False so runs actually save (see GUI_MultiVMC_Torque.py)
PLOT_Z              = True   # include z-axis subplot in saved plot
PLOT_FORCE          = True   # include /force_shear subplot in saved plot
PLOT_VIRTUAL_FORCES = True   # if True, Cell 4 in the GUI shows virtual forces instead of motor torques

# ─────────────────────────────────────────────────────────────────────────────
# Pretension config
# tau_pretension[i] = PRETENSION_WEIGHTS[i] * q_motor[i]
#   applied only when q_motor[i] > PRETENSION_THRESHOLD (per motor).
# Set weights to 0.0 to disable pretension for a motor.
# ─────────────────────────────────────────────────────────────────────────────
PRETENSION_WEIGHTS   = np.full(9, 0.03, dtype=float)  # [N·m/rad] x 9
PRETENSION_THRESHOLD = np.deg2rad(30.0)               # [rad] same threshold for all motors
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

# ── Position-control graph ranges ─────────────────────────────────────────────
XY_RANGE = (-0.15, 0.15)   # [m] both x and y axes
Z_RANGE  = (0.2, 0.55)     # [m] z slider range

# ── Torque-plot rolling window ─────────────────────────────────────────────────
TAU_PLOT_WINDOW = 10.0     # [s] how many seconds are visible in the live torque chart

# ── 3D visualization s-sample density ─────────────────────────────────────────
FK_S_SAMPLES = 4           # number of FK evaluations along the robot for the 3D line
