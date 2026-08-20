import numpy as np
from MultiVMCTaskSpace import VirtualComponent

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from VMC_utils.VirtualModels import ConstrainedLinearSpring, LinearDamper

"""
Config for the connectivity sanity check — shared by GUI_MultiVMC_Torque.py
and GUI_MultiVMC.py (drag the goal by hand) or MultiVMC_MovingGoal_Pub.py
(scripted). Purpose: confirm the hardware/software chain works end to end --
motors respond, kinematics look right, no crashes -- BEFORE trusting it for
any real experiment. Not tied to any paper figure. Nothing is ever saved
(SAVE_PLOT/SAVE_DATA=False below), so there's no COLLECTED_DATA flag here --
see the other experiment folders' configs for that.

k=15, c=1 below are NOT the VMC/GUI_MultiVMC_Config.py "lin_seg3" values
(k=90, c=15) -- those caused visible oscillation on real hardware and should
not be trusted; see that file's own history for why. These are instead
derived from BimanualHelyxVMC (same XL330-M288 motors, same Helyx arm
design), whose ControllerBimanual/VMC_Bimanual.py tip segment runs
k=45, c=2 stably, hardcoded against kt=0.00115 -- confirmed unchanged across
~2 months of active commits, so genuinely validated together, not stale.
current[mA] = torque[N*m] / kt (confirmed directly in dynamixel_node.cpp,
no clamp), so delivered torque for a fixed k/c scales with 1/kt. SoftVMC's
kt=0.000354 is ~3.25x smaller than BimanualHelyxVMC's 0.00115, so the same
k/c would deliver ~3.25x more actual torque here -- consistent with why
90/15 oscillated. Dividing BimanualHelyxVMC's tip values by that same 3.25
(k=45/3.25≈13.8, c=2/3.25≈0.62) and rounding gives k=15, c=1 -- also
matching c=1 already confirmed stable on this arm independently.
kt=0.000354 itself is NOT in question -- verified in an earlier session;
only the k/c gains were the unverified part. Still: raise further live via
the GUI, watching for oscillation, before trusting these beyond a sanity
check, and before raising TAU_LIMIT back toward VMC/GUI_MultiVMC_Config.py.

REQUIREMENT: len(COMPONENT_SPECS) == number of virtual components == number of goals.
"""

# ─────────────────────────────────────────────────────────────────────────────
# Reference positions -- identical to VMC/GUI_MultiVMC_Config.py's go_x_y
# ─────────────────────────────────────────────────────────────────────────────
go_x_y = np.array([0.2, 0.2, 0.3], dtype=float)

# ─────────────────────────────────────────────────────────────────────────────
# Virtual Components + Goals
# ─────────────────────────────────────────────────────────────────────────────
COMPONENT_SPECS = [
    dict(
        component=VirtualComponent(
            # Constrained to X only, exactly like VMC/GUI_MultiVMC_Config.py's
            # lin_seg3 -- the spring force ignores the goal's Y/Z components.
            # k=15, c=1 -- BimanualHelyxVMC's tip values (k=45, c=2), divided
            # by ~3.25 to account for SoftVMC's smaller kt -- see docstring.
            spring=ConstrainedLinearSpring(15.0, n=np.array([[1], [0], [0]])),
            damper=LinearDamper(1.0),
            s_param=1.0,
            name="test",
        ),
        goal=go_x_y,  # GUI OVERRIDES THIS GOAL --- IGNORE ---
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
# Only used if you run that script instead of GUI_MultiVMC.py.
# ─────────────────────────────────────────────────────────────────────────────
STEPS_M              = 0.001   # [m] per step (0.001 m / 0.1 s = 0.01 m/s)
PUBLISH_PERIOD        = 0.1    # [s] 10 Hz
LOOP_BACK             = True   # oscillate back and forth instead of stopping once
MOVING_GOAL           = True
SYNCHRONIZED_ARRIVAL  = False  # irrelevant with a single component

# ─────────────────────────────────────────────────────────────────────────────
# Subscriber / Controller Config
# ─────────────────────────────────────────────────────────────────────────────
TAU_LIMIT           = 0.5   # [N·m] per motor -- low hard ceiling while raising
                             # stiffness live from 0; raise this only once you've
                             # confirmed low-stiffness behavior is stable
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

# ── Position-control graph ranges -- matches VMC/GUI_MultiVMC_Config.py ───────
XY_RANGE = (-0.6, 0.6)   # [m] both x and y axes
Z_RANGE  = (0.0, 0.45)   # [m] z slider range

# ── Torque-plot rolling window ─────────────────────────────────────────────────
TAU_PLOT_WINDOW = 10.0     # [s] how many seconds are visible in the live torque chart

# ── 3D visualization s-sample density ─────────────────────────────────────────
FK_S_SAMPLES = 4           # number of FK evaluations along the robot for the 3D line
