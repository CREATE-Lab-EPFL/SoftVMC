"""
Button pressing against the load cell -- driven live from the GUI.

    python3 ../VMC/GUI_MultiVMC_Torque.py --config _button --run <name>   # terminal 2
    python3 ../VMC/GUI_MultiVMC.py        --config _button                # terminal 3

One run is one continuous recording of a session at the GUI. There is no
strength sweep and no condition: both Gaussians are held at K_GAUSS = 500 N/m
throughout, and the only things that move are the two goals, dragged by hand:

  lin_tip     (s=1.0)  linear spring -- drag its goal onto the button
  gauss_seg1  (s=1/3)  repulsive Gaussian, centre on the fixture
  gauss_seg2  (s=2/3)  the same, further up the body -- SHARES seg1's centre

x and y of those two points are the whole interface. Every z is locked, and
LOCK_PARAMS makes the stiffnesses read-only, so what the log holds is exactly
what was done and nothing else could have changed.
"""

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
from VMC.MultiVMCTaskSpace import VirtualComponent
from VMC_utils.VirtualModels import LinearSpring, LinearDamper, GaussianSpring
from VMC_utils.ConfigHelpers import default_start, param_info, rest_pose

CONFIG_NAME = "button"

# ── Virtual elements ─────────────────────────────────────────────────────────
K_REACH = 30.0    # [N/m]   tip spring
K_GAUSS = 500.0   # [N/m]   both Gaussians, for the whole run -- see below
SIGMA   = 0.06    # [m]     Gaussian width
C_DAMP  = 1.0     # [N.s/m] all three

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

# ── Fixed strengths ──────────────────────────────────────────────────────────
# There is no strength sweep and no per-run condition: K_GAUSS is the field,
# every run. LOCK_PARAMS makes the GUI show the three k and c values without
# letting them be edited, and stops it publishing /VC_Param at all -- so the
# torque node keeps exactly what this file gave it, and a run cannot silently
# become a different experiment halfway through.
LOCK_PARAMS = True

# K_GAUSS cannot be switched on at once: the obstacle centre is already placed,
# so a field at full strength shoves the arm off it within a single tick. Both
# Gaussians are eased in from 0 over RAMP_S, starting with the torque node --
# before the GUI is up, so it lands at t < 0 in the log. k_params records what
# was actually applied, tick by tick.
RAMP_COMPONENTS = GAUSS_COMPONENTS
RAMP_S          = 5.0   # [s]   0 disables the ramp

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
COLLECTED_DATA = True    # True = never save, the recorded runs are final

# Checkpoint the logs to <run>.autosave.npz every 30 s while the run is going,
# and print a status line. These runs are minutes long and hand-driven, so a
# crash at the end would otherwise cost the whole thing; the checkpoint is
# deleted once the real file is written, so one left behind means the run did
# not exit cleanly. The write runs on its own thread, off the control loop.
AUTOSAVE_S     = 30.0    # [s] 0 disables

# Marked with Ctrl+C in the torque node's terminal, one press each, while the
# run goes; the press after the last one ends the run. A run is one continuous
# recording of whatever was done with the GUI, so nothing else in the log says
# when the tip actually arrived: press once when it is on the target and the
# analysis takes its measurement window from there instead of guessing it.
MARK_PROMPTS   = ("Target reached?",)

PLOT_Z         = True
PLOT_FORCE     = True    # load cell is used here

# ── GUI placement constraints ────────────────────────────────────────────────
# Two points are placed by hand, and they are the whole experiment:
#   - lin_tip's x/y      -- the target, dragged onto the button
#   - the Gaussians' x/y -- the obstacle, shared by both (GOAL_LINK), so
#                           dragging either moves both
# Everything else is held: all three z are locked (z is the reach out along the
# arm's axis, NOT height -- gravity is -y), and the stiffnesses are LOCK_PARAMS.
#   - the button's distance along the arm's axis is known, so the tip goal's z
#     is held at it
#   - the two Gaussians share ONE goal, held at the midpoint between the tips
#     of section 1 and section 2 (z is linear in s on the straight arm, so that
#     midpoint is just rest_pose(0.5))
TIP_GOAL_Z   = 0.38                       # [m] button distance from the base, along
                                          #     the arm's axis -- z is NOT height here
GAUSS_GOAL_Z = float(rest_pose(0.5)[2])   # [m] = 0.2175, between sections 1 and 2

GOAL_Z_LOCK = {"lin_tip":    TIP_GOAL_Z,
               "gauss_seg1": GAUSS_GOAL_Z,
               "gauss_seg2": GAUSS_GOAL_Z}

# Where the two goals start, so the arm is somewhere sensible before anything
# is dragged. Both are meant to be moved.
TIP_GOAL_XY   = (0.0, 0.0)     # [m] straight ahead: the tip spring pulls only
                               # the 5.5 cm of take-up to TIP_GOAL_Z
GAUSS_GOAL_XY = (-0.15, 0.0)   # [m] x is chosen so the field OPPOSES the
                               # reach: repulsion pushes the attachment point
                               # away from the centre, so the centre must sit
                               # at more negative x than the body ever reaches,
                               # or the push points at the button instead.
GOAL_LINK   = [GAUSS_COMPONENTS]

# Live from the start, like every other folder. The tip goal sits 5.5 cm below
# the resting tip, so the arm takes up that slack (~1.65 N at K_REACH = 30) as
# soon as the torque node is up, while the Gaussians ramp in. Untick "Publisher
# Active" in the GUI to place the goals quietly.
PUBLISH_ON_START = True

# ── GUI framing ──────────────────────────────────────────────────────────────
XY_RANGE = (-0.35, 0.35)   # [m]
Z_RANGE  = (0.15, 0.50)    # [m]

# ── Derived ──────────────────────────────────────────────────────────────────
DEFAULT_START = default_start(VIRTUAL_COMPONENTS)   # straight-arm rest pose
for _i, _c in enumerate(VIRTUAL_COMPONENTS):        # start each goal where it belongs
    if _c.name in GOAL_Z_LOCK:
        DEFAULT_START[_i, 2] = GOAL_Z_LOCK[_c.name]
    DEFAULT_START[_i, :2] = (GAUSS_GOAL_XY if _c.name in GAUSS_COMPONENTS
                             else TIP_GOAL_XY)
N_VC          = len(VIRTUAL_COMPONENTS)

(SPRING_PARAM_NAMES, DAMPER_PARAM_NAMES,
 SPRING_PARAM_INIT,  DAMPER_PARAM_INIT) = param_info(VIRTUAL_COMPONENTS)
