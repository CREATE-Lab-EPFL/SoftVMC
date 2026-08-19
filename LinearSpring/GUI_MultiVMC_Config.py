import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
from VMC.MultiVMCTaskSpace import VirtualComponent
from VMC_utils.VirtualModels import (
    ConstrainedLinearSpring, LinearDamper, LinearSpring,
    TanhSpring, TanhDamper, GaussianSpring, DeadzoneLimitSpring
)

"""
Config for Modular VMC — shared by both vmc_torque_node.py and robot_gui.py.

REQUIREMENT: len(COMPONENT_SPECS) == number of virtual components == number of goals.
"""

# ─────────────────────────────────────────────────────────────────────────────
# Reference positions (keep existing)
# ─────────────────────────────────────────────────────────────────────────────
rest_pos_seg3 = np.array([0.0, 0.0, 3 * 0.145], dtype=float)
go_y          = np.array([0.0, 0.3,  0.3],        dtype=float)
go_x          = np.array([0.3, 0.0,  0.35],        dtype=float)
gauss_y       = np.array([0.0, 0.15, 0.2],         dtype=float)

# ─────────────────────────────────────────────────────────────────────────────
# Virtual Components + Goals
# ─────────────────────────────────────────────────────────────────────────────
COMPONENT_SPECS = [
    dict(
        component=VirtualComponent(
            spring=LinearSpring(0.0),
            damper=LinearDamper(0.0),
            s_param=1.0,
            name="lin_seg3",
        ),
        goal=rest_pos_seg3,# GUI OVERRIDES THIS GOAL --- IGNORE ---
    ),
    # dict(
    #     component=VirtualComponent(
    #         spring=LinearSpring(100.0),
    #         damper=LinearDamper(5.0),
    #         s_param=2.0/3.0,
    #         name="lin_seg2",
    #     ),
    #     goal=gauss_y,# GUI OVERRIDES THIS GOAL --- IGNORE ---
    # ),
    # dict(
    #     component=VirtualComponent(
    #         spring=LinearSpring(100.0),
    #         damper=LinearDamper(5.0),
    #         s_param=1.0/3.0,
    #         name="lin_seg1",
    #     ),
    #     goal=gauss_y,# GUI OVERRIDES THIS GOAL --- IGNORE ---
    # ),
    # dict(
    #     component=VirtualComponent(
    #         spring=LinearSpring(230.0),
    #         damper=LinearDamper(5.0),
    #         s_param=1.0,
    #         name="lin_x",
    #     ),
    #     goal=go_x, # GUI OVERRIDES THIS GOAL --- IGNORE ---
    # ),
    # dict(
    #     component=VirtualComponent(
    #         # Gaussian spring produces max force when sigma = delta_x so when initializing on the robot it could produce very large forces
    #         # set stregnth to 0.0 to effectively disable the spring until we can move the reference in the GUI far from robot 
    #         # once the reference is moved we can increase the strength to a nonzero force Typically stregnth = 1200 is quite good
    #         # the max force produced by the gauss spring is when delta_x = sigma 
    #         spring=GaussianSpring(strength=0.0, sigma=0.06), 
    #         damper=LinearDamper(5.0),
    #         s_param=1.0 / 3.0,
    #         name="gauss_seg1",
    #     ),
    #     goal=gauss_y,# GUI OVERRIDES THIS GOAL --- IGNORE ---
    # ),
    # dict(
    #     component=VirtualComponent(
    #         # Gaussian spring produces max force when sigma = delta_x so when initializing on the robot it could produce very large forces
    #         # set stregnth to 0.0 to effectively disable the spring until we can move the reference in the GUI far from robot 
    #         # once the reference is moved we can increase the strength to a nonzero force
    #         spring=GaussianSpring(strength=0.0, sigma=0.06), 
    #         damper=LinearDamper(5.0),
    #         s_param=2.0 / 3.0,
    #         name="gauss_seg2",
    #     ),
    #     goal=gauss_y,# GUI OVERRIDES THIS GOAL --- IGNORE ---
    # ),
    # dict(
    #     component=VirtualComponent(
    #         # Gaussian spring produces max force when sigma = delta_x so when initializing on the robot it could produce very large forces
    #         # set stregnth to 0.0 to effectively disable the spring until we can move the reference in the GUI far from robot 
    #         # once the reference is moved we can increase the strength to a nonzero force
    #         spring=GaussianSpring(strength=0.0, sigma=0.06), 
    #         damper=LinearDamper(5.0),
    #         s_param=1.0,
    #         name="gauss_seg3",
    #     ),
    #     goal=gauss_y,# GUI OVERRIDES THIS GOAL --- IGNORE ---
    # ),
]

# ─────────────────────────────────────────────────────────────────────────────
# Derived: VIRTUAL_COMPONENTS / DEFAULT_GOALS  (same as before)
# ─────────────────────────────────────────────────────────────────────────────
VIRTUAL_COMPONENTS = [spec["component"] for spec in COMPONENT_SPECS]
DEFAULT_GOALS      = np.array([spec["goal"] for spec in COMPONENT_SPECS], dtype=float)  # (N,3)

def _start_from_s_param(s: float) -> np.ndarray:
    """Straight-arm rest position for a given s_param ∈ [0,1]."""
    if s <= 1.0 / 3.0:
        z = 3 * 0.145 * s
    elif s <= 2.0 / 3.0:
        z = 0.145 + 0.145 * (s - 1/3) * 3
    else:
        z = 2 * 0.145 + 0.145 * (s - 2/3) * 3
    return np.array([0.0, 0.0, z], dtype=float)

DEFAULT_START = np.array(
    [_start_from_s_param(comp.s_param) for comp in VIRTUAL_COMPONENTS],
    dtype=float,
)  # (N,3)
# DEFAULT_START = np.array([
#     [0.0, 0.0, 0.3],   # lin_seg3  (s=1.0)
#     [0.0, 0.0, 0.145+0.0725],   # gauss_seg1 (s=1/3)
#     [0.0, 0.0, 0.145+0.0725],   # gauss_seg2 (s=2/3)
#     # [0.0, 0.0, 0.145+0.0725],   # gauss_seg3 (s=1.0)
# ], dtype=float)  # (N,3)

# ─────────────────────────────────────────────────────────────────────────────
# Publisher Config  (used by old publisher; kept for compatibility)
# ─────────────────────────────────────────────────────────────────────────────
# STEPS_M          = 0.0008   # [m] per step
# PUBLISH_PERIOD   = 0.01     # [s]  100 Hz
# LOOP_BACK        = False
# MOVING_GOAL      = True
# SYNCHRONIZED_ARRIVAL = False

# ─────────────────────────────────────────────────────────────────────────────
# Subscriber / Controller Config
# ─────────────────────────────────────────────────────────────────────────────
TAU_LIMIT  = 2.1   # [N·m] per motor
SAVE_PLOT           = True
SAVE_DATA           = True  # save raw arrays to .npz alongside the plot
COLLECTED_DATA      = True  # real data already collected -- reruns will NOT overwrite it (see GUI_MultiVMC_Torque.py)
PLOT_Z              = True   # include z-axis subplot in saved plot
PLOT_FORCE          = False  # include /force_shear subplot in saved plot
PLOT_VIRTUAL_FORCES = True  # if True, Cell 4 in the GUI shows virtual forces instead of motor torques

# ─────────────────────────────────────────────────────────────────────────────
# Pretension config
# tau_pretension[i] = PRETENSION_WEIGHTS[i] * q_motor[i]
#   applied only when q_motor[i] > PRETENSION_THRESHOLD (per motor).
# Set weights to 0.0 to disable pretension for a motor.
# ─────────────────────────────────────────────────────────────────────────────
SCALE = 6
PRETENSION_WEIGHTS    = SCALE*np.array([0.005, 0.005, 0.005, 0.005, 0.005, 0.005,
                                   0.005, 0.005, 0.005], dtype=float)  # [N·m/rad] x 9
PRETENSION_THRESHOLD  = np.deg2rad(30.0)   # [rad]  same threshold for all motors
PRETENSION_ENABLED    = True              # master switch for pretension (can still set weights/threshold above)

# ─────────────────────────────────────────────────────────────────────────────
# ── GUI metadata ─────────────────────────────────────────────────────────
# ─────────────────────────────────────────────────────────────────────────────

# Number of virtual components (convenience)
N_VC = len(VIRTUAL_COMPONENTS)

# One color per virtual component (up to 7 supported out of the box)
GUI_COLORS = ["red", "blue", "green", "darkorange", "purple", "cyan", "magenta"]

# ── Spring / Damper parameter introspection ───────────────────────────────────
# We assume each spring and damper expose their primary scalar parameter as the
# first entry of __dict__.  This works for LinearSpring(k), LinearDamper(c),
# GaussianSpring(strength, sigma) [takes 'strength'], TanhSpring, etc.
#
# If you add a spring with a differently-named leading parameter, just override
# SPRING_PARAM_NAMES[i] / DAMPER_PARAM_NAMES[i] below.

def _first_attr(obj, fallback_name: str, fallback_val: float):
    d = getattr(obj, "__dict__", {})
    if d:
        k = next(iter(d))
        return k, float(list(d.values())[0])
    return fallback_name, fallback_val

_spring_info = [_first_attr(c.spring, "k",  1.0) for c in VIRTUAL_COMPONENTS]
_damper_info = [_first_attr(c.damper, "c",  1.0) for c in VIRTUAL_COMPONENTS]

SPRING_PARAM_NAMES  = [t[0] for t in _spring_info]   # e.g. ["k", "k"]
DAMPER_PARAM_NAMES  = [t[0] for t in _damper_info]   # e.g. ["c", "c"]
SPRING_PARAM_INIT   = [t[1] for t in _spring_info]   # e.g. [690.0, 690.0]
DAMPER_PARAM_INIT   = [t[1] for t in _damper_info]   # e.g. [15.0, 15.0]

# ── Position-control graph ranges ─────────────────────────────────────────────
XY_RANGE = (-0.5, 0.5)   # [m] both x and y axes
Z_RANGE  = (0.0,  0.45)  # [m] z slider range

# ── Torque-plot rolling window ─────────────────────────────────────────────────
TAU_PLOT_WINDOW = 10.0   # [s] how many seconds are visible in the live torque chart

# ── 3D visualization s-sample density ─────────────────────────────────────────
FK_S_SAMPLES = 4        # number of FK evaluations along the robot for the 3D line

# ── Plotly output directory ───────────────────────────────────────────────────
PLOTLY_OUTPUT_DIR = "plots"