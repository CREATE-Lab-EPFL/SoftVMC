"""
Button pressing against the load cell -- driven live from the GUI.

    python3 ../VMC/GUI_MultiVMC_Torque.py --config _button --run <name>
    python3 ../VMC/GUI_MultiVMC.py        --config _button

This file sets the three virtual elements and their parameters, nothing else.
Everything positional -- where the button is, where the obstacle is, how far
past the surface to aim -- is set by dragging in the GUI, so none of it is
hardcoded here.

  lin_tip     (s=1.0)  linear spring, drag its goal onto the button
  gauss_seg1  (s=1/3)  repulsive Gaussian, drag its centre onto the obstacle
  gauss_seg2  (s=2/3)  the same, further up the body

Both Gaussians start at strength 0 -- raise them with the GUI sliders once
their centres are placed, or the field peaks the moment it is switched on.
"""

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
from VMC.MultiVMCTaskSpace import VirtualComponent
from VMC_utils.VirtualModels import LinearSpring, LinearDamper, GaussianSpring
from VMC_utils.ConfigHelpers import default_start, param_info, rest_pose

CONFIG_NAME = "button"

# ── Virtual elements ─────────────────────────────────────────────────────────
K_REACH = 30.0   # [N/m]   tip spring
K_GAUSS = 0.0    # [N/m]   both Gaussians start off; raise them from the GUI
SIGMA   = 0.06   # [m]     Gaussian width
C_DAMP  = 1.0    # [N.s/m] all three

GAUSS_COMPONENTS = ("gauss_seg1", "gauss_seg2")   # one obstacle, two attachment points

VIRTUAL_COMPONENTS = [
    VirtualComponent(spring=LinearSpring(K_REACH),
                     damper=LinearDamper(C_DAMP),
                     s_param=1.0, name="lin_tip"),
    VirtualComponent(spring=GaussianSpring(strength=K_GAUSS, sigma=SIGMA),
                     damper=LinearDamper(C_DAMP),
                     s_param=1.0 / 3.0, name="gauss_seg1"),
    VirtualComponent(spring=GaussianSpring(strength=K_GAUSS, sigma=SIGMA),
                     damper=LinearDamper(C_DAMP),
                     s_param=2.0 / 3.0, name="gauss_seg2"),
]

# ── Conditions ───────────────────────────────────────────────────────────────
# The Gaussian strength each run name stands for. `--run medium` starts both
# Gaussians at 500 N/m -- no slider work needed to set up a named run; the
# sliders are still live if you want to move off it.
#
# The names mean the same strength in GUI_MultiVMC_Config_person.py, so the
# fixture and the person can be compared run for run. The person drops `weak`
# and keeps the top two.
GAUSS_CONDITIONS = {
    "no_gauss":    0.0,
    "weak":      250.0,
    "medium":    500.0,
    "strong":   1000.0,
}
CONDITION_APPLIES_TO = GAUSS_COMPONENTS   # both share one strength

# The obstacle centre is already placed, so a run's strength is ramped in from
# 0 over this many seconds rather than applied at once -- switched on at full
# strength the field would shove the arm off the obstacle in a single tick.
GAUSS_RAMP_S = 5.0   # [s]   0 disables the ramp

# ── Controller ───────────────────────────────────────────────────────────────
TAU_LIMIT   = 0.25   # [N·m] per motor
GRAVITY_DIR = "-y"   # arm rotated onto its side: gravity pulls along -y, not -z

# tau_pretension[i] = PRETENSION_WEIGHTS[i] * q_motor[i], always applied
PRETENSION_ENABLED   = True
PRETENSION_WEIGHTS   = np.full(9, 0.010, dtype=float)   # [N·m/rad] x 9 -- always on, no threshold

# ── Output ───────────────────────────────────────────────────────────────────
OUTPUT_FOLDER  = "button_pressing"   # --run NAME lands in outputs/<folder>/NAME.*
SAVE_PLOT      = True
SAVE_DATA      = True    # raw arrays to .npz alongside the plot
COLLECTED_DATA = False   # True = never save, the recorded runs are final

# Checkpoint the logs to <run>.autosave.npz every 30 s while the run is going,
# and print a status line. These runs are minutes long and hand-driven, so a
# crash at the end would otherwise cost the whole thing; the checkpoint is
# deleted once the real file is written, so one left behind means the run did
# not exit cleanly. The write runs on its own thread, off the control loop.
AUTOSAVE_S     = 30.0    # [s] 0 disables

# Asked in terminal 3, one per ENTER, while the run goes. The run ends with
# Ctrl+C, so nothing else in the log says when the tip actually arrived: press
# ENTER once the tip is on the button and the analysis takes the measurement
# window from there instead of guessing it.
MARK_PROMPTS   = ("Target reached?",)

PLOT_Z         = True
PLOT_FORCE     = True    # load cell is used here

# ── GUI placement constraints ────────────────────────────────────────────────
# Only two points are placed by hand per run: the tip's x/y and one shared
# centre for both Gaussians.
#   - the button's distance along the arm's axis is known, so the tip
#     goal's z is held at it (z is not height: gravity is -y)
#   - the two Gaussians share ONE goal, held at the midpoint between the tips
#     of section 1 and section 2 (z is linear in s on the straight arm, so that
#     midpoint is just rest_pose(0.5))
TIP_GOAL_Z   = 0.38                       # [m] button distance from the base, along
                                          #     the arm's axis -- z is NOT height here
GAUSS_GOAL_Z = float(rest_pose(0.5)[2])   # [m] = 0.2175, between sections 1 and 2

GOAL_Z_LOCK = {"lin_tip":    TIP_GOAL_Z,
               "gauss_seg1": GAUSS_GOAL_Z,
               "gauss_seg2": GAUSS_GOAL_Z}

# The obstacle is pre-placed, so a run can start without dragging anything.
# Drag either Gaussian in the GUI to move it; both follow.
GAUSS_GOAL_XY = (-0.05, 0.0)   # [m]
GOAL_LINK   = [GAUSS_COMPONENTS]

# Live from the start, like every other folder. The tip goal sits 5.5 cm below
# the resting tip, so the arm takes up that slack (~1.65 N at K_REACH = 30) as
# soon as the torque node is up -- and a named run's Gaussians are already at
# their strength. Untick "Publisher Active" in the GUI to place goals quietly.
PUBLISH_ON_START = True

# ── GUI framing ──────────────────────────────────────────────────────────────
XY_RANGE = (-0.35, 0.35)   # [m]
Z_RANGE  = (0.15, 0.50)    # [m]

# ── Derived ──────────────────────────────────────────────────────────────────
DEFAULT_START = default_start(VIRTUAL_COMPONENTS)   # straight-arm rest pose
for _i, _c in enumerate(VIRTUAL_COMPONENTS):        # start each goal where it belongs
    if _c.name in GOAL_Z_LOCK:
        DEFAULT_START[_i, 2] = GOAL_Z_LOCK[_c.name]
    if _c.name in GAUSS_COMPONENTS:
        DEFAULT_START[_i, :2] = GAUSS_GOAL_XY
N_VC          = len(VIRTUAL_COMPONENTS)

(SPRING_PARAM_NAMES, DAMPER_PARAM_NAMES,
 SPRING_PARAM_INIT,  DAMPER_PARAM_INIT) = param_info(VIRTUAL_COMPONENTS)
