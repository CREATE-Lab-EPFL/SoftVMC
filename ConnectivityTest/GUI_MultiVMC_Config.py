import numpy as np
from MultiVMCTaskSpace import VirtualComponent

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from VMC_utils.VirtualModels import LinearDamper, LinearSpring

"""
Config for the connectivity sanity check — shared by GUI_MultiVMC_Torque.py
and MultiVMC_MovingGoal_Pub.py (or GUI_MultiVMC.py, if you'd rather drag the
goal by hand). Purpose: confirm the hardware/software chain works end to end
-- motors respond, kinematics look right, no crashes -- BEFORE trusting it
for any real experiment. Not tied to any paper figure. Nothing is ever saved
(SAVE_PLOT/SAVE_DATA=False below), so there's no COLLECTED_DATA flag here --
see the other experiment folders' configs for that.

One component, gentle stiffness, small/safe motion -- edit TEST_GOAL freely,
it isn't measured against anything physical.

REQUIREMENT: len(COMPONENT_SPECS) == number of virtual components == number of goals.
"""

# ─────────────────────────────────────────────────────────────────────────────
# Reference positions
# ─────────────────────────────────────────────────────────────────────────────
TEST_GOAL = np.array([0.03, 0.03, 0.40], dtype=float)  # small, safe nudge -- edit freely

# ─────────────────────────────────────────────────────────────────────────────
# Virtual Components + Goals
# ─────────────────────────────────────────────────────────────────────────────
COMPONENT_SPECS = [
    dict(
        component=VirtualComponent(
            # k=20, c=15 -- matches the damping VMC/GUI_MultiVMC_Config.py's own
            # lin_seg3 component uses (proven on this arm). Original placeholders
            # here (k=40, c=3) were untested and underdamped -- caused visible
            # oscillation on real hardware, compounded by TAU_LIMIT=1.0 clipping
            # the command and stripping out the damping term when it mattered most.
            spring=LinearSpring(20.0),
            damper=LinearDamper(15.0),
            s_param=1.0,
            name="test",
        ),
        goal=TEST_GOAL,  # GUI OVERRIDES THIS GOAL --- IGNORE ---
    ),
]

# ─────────────────────────────────────────────────────────────────────────────
# Derived: VIRTUAL_COMPONENTS / DEFAULT_START / DEFAULT_GOALS
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
# MultiVMC_MovingGoal_Pub.py config -- moves DEFAULT_START -> DEFAULT_GOALS and
# (with LOOP_BACK) back again, continuously, so you can just watch it track.
# ─────────────────────────────────────────────────────────────────────────────
STEPS_M              = 0.001   # [m] per step (0.001 m / 0.1 s = 0.01 m/s)
PUBLISH_PERIOD        = 0.1    # [s] 10 Hz
LOOP_BACK             = True   # oscillate back and forth instead of stopping once
MOVING_GOAL           = True
SYNCHRONIZED_ARRIVAL  = False  # irrelevant with a single component

# ─────────────────────────────────────────────────────────────────────────────
# Subscriber / Controller Config
# ─────────────────────────────────────────────────────────────────────────────
TAU_LIMIT           = 1.0   # [N·m] per motor -- conservative, this is just a motion check
SAVE_PLOT           = False  # never save anything from this folder
SAVE_DATA           = False
PLOT_Z              = True
PLOT_FORCE          = False  # no load cell involved in a pure connectivity check
PLOT_VIRTUAL_FORCES = True

# ─────────────────────────────────────────────────────────────────────────────
# Pretension config
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
