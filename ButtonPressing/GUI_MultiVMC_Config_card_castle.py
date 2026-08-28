"""
Whole-body shaping around a house of cards -- driven live from the GUI.

    python3 ../VMC/GUI_MultiVMC_Torque.py --config _card_castle --run <case>   # terminal 2
    python3 ../VMC/GUI_MultiVMC.py        --config _card_castle --run <case>   # terminal 3

Both get the SAME --run: the GUI publishes all three stiffnesses at once, so one
told the wrong case would apply the wrong field. REQUIRE_RUN_NAME makes both
refuse to start without it.

The same experiment as GUI_MultiVMC_Config_person.py with a different obstacle:
a card castle standing at x = -0.25 m is the working area the robot must keep
out of. The tip is driven to the button by hand from the GUI; the body is kept
off the castle by the two Gaussians. Two runs, and the difference between them
is the benefit of shaping the whole body rather than just placing the tip:

    --run no_gauss    no field at all -- the body goes wherever reaching takes it
    --run gauss       the field on, at 1000 N/m

A card castle shows that difference in a way a person cannot: it falls over. So
`no_gauss` is expected to knock it down -- record `gauss` FIRST and rebuild
before the second run, not the other way round.

One run each, named by the case, so the two files ARE the experiment:
outputs/card_castle/{no_gauss,gauss}.{npz,png}. --run is required and only those
two names are accepted -- an unrecognised one is refused before anything is
built, so a case cannot be recorded under the wrong name.

  lin_tip     (s=1.0)  linear spring -- drag its goal onto the button
  gauss_seg1  (s=1/3)  repulsive Gaussian, centre on the castle
  gauss_seg2  (s=2/3)  the same, further up the body -- SHARES seg1's centre

Live from the GUI: x and y of those two points, the tip stiffness, the two
Gaussian strengths, and a tickbox that switches the Gaussians off and back on.
Every z is locked, and both Gaussians share one centre. Everything the hand did
is in the log -- the goals in `goal`, the stiffnesses in `k_params` -- so a run
reads back as what was actually done, not as what the config said.

Every value below is GUI_MultiVMC_Config_person.py's, unchanged, so the two are
directly comparable. Keep them in step.
"""

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
from VMC.MultiVMCTaskSpace import VirtualComponent
from VMC_utils.VirtualModels import LinearSpring, LinearDamper, GaussianSpring
from VMC_utils.ConfigHelpers import default_start, param_info, rest_pose

CONFIG_NAME = "card_castle"

# ── Virtual elements ─────────────────────────────────────────────────────────
K_REACH = 30.0    # [N/m]   tip spring
K_GAUSS = 0.0     # [N/m]   the run's level sets this; see GAUSS_CONDITIONS
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

# ── The two cases ────────────────────────────────────────────────────────────
# The Gaussian strength each run name stands for. Two runs, and the pair IS the
# experiment: the same reach with the body shaped away from the castle, and
# without. Nothing in between -- the comparison is the point, not a curve.
# Both Gaussians take the case's strength; the torque node ramps them in from
# 0 over RAMP_S when the run starts, so no slider work is needed to set one up.
#
# These are the field STRENGTHS, not forces: a Gaussian's force peaks at
# k*exp(-1/2)*sigma = k*0.0364 N when the attachment point is sigma from the
# centre, so `gauss` peaks at 36 N and falls away fast on either side of that.
# What the motors actually get is clipped at TAU_LIMIT.
#
# 1000 is the button config's `strong` and _person's `gauss`, so all three
# experiments are driven at an identical field.
GAUSS_CONDITIONS = {
    "no_gauss":      0.0,
    "gauss":      1000.0,
}

# --run is not optional here: an unnamed run belongs to neither case.
REQUIRE_RUN_NAME = True

# The case's strength is where a run STARTS. The GUI can move it from there --
# K_REACH and both Gaussian strengths are editable in cell (1) and take effect
# on "Apply Parameters" -- and k_params records what was applied tick by tick,
# so the log says which value was live when.
LOCK_PARAMS = False

# A "Gaussians active" tickbox in the GUI, so the same reach can be shown with
# the field and without it, in one recording and with nothing else changed. The
# tickbox owns WHETHER the field is on, the entries own HOW STRONG it is when it
# is -- so it comes back up to whatever was last applied, not to the case's own
# strength. Either way it is walked over RAMP_S rather than stepped: switching a
# placed 1000 N/m field on in a single tick is exactly what the start-up ramp
# exists to avoid, and switching it off that way drops the arm.
GAUSS_TOGGLE = True

# The strength cannot be switched on at once: the obstacle centre is already
# placed, so a field at full strength shoves the arm off it within a single
# tick. Both Gaussians are eased in from 0 over RAMP_S, starting with the torque
# node -- before the GUI is up, so it lands at t < 0 in the log. k_params
# records what was actually applied, tick by tick.
RAMP_COMPONENTS = GAUSS_COMPONENTS
RAMP_S          = 5.0   # [s]   0 disables the ramp

# ── Controller ───────────────────────────────────────────────────────────────
TAU_LIMIT   = 0.35   # [N·m] per motor -- HIGHER than the button's 0.25. This
                     # experiment asks the body to hold a shape against the
                     # field as well as reach, and at 0.25 the limiter, not the
                     # virtual model, was deciding the pose. Set here and
                     # nowhere else: the button's recorded runs are at 0.25 and
                     # must stay there.
GRAVITY_DIR = "-y"   # arm rotated onto its side: gravity pulls along -y, not -z

# tau_pretension[i] = PRETENSION_WEIGHTS[i] * q_motor[i], always applied
PRETENSION_ENABLED   = True
PRETENSION_WEIGHTS   = np.full(9, 0.010, dtype=float)   # [N·m/rad] x 9 -- always on, no threshold

# ── Output ───────────────────────────────────────────────────────────────────
OUTPUT_FOLDER  = "card_castle"       # --run NAME lands in outputs/<folder>/NAME.*

# Nothing here ever lands in experiments/: --run is required (REQUIRE_RUN_NAME)
# and every accepted name is a case, so every run goes straight to
# outputs/<OUTPUT_FOLDER>/<case>.{npz,png}. Re-running a case overwrites it.
SAVE_PLOT      = True
SAVE_DATA      = True    # raw arrays to .npz alongside the plot
COLLECTED_DATA = False   # True = never save, the recorded runs are final

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
PLOT_FORCE     = False   # no load cell in this variant

# ── GUI placement constraints ────────────────────────────────────────────────
# Two points are placed by hand, and they are the whole experiment:
#   - lin_tip's x/y      -- the target, dragged onto the button
#   - the Gaussians' x/y -- the obstacle, shared by both (GOAL_LINK), so
#                           dragging either moves both
# The z of all three is held (z is the reach out along the arm's axis, NOT
# height -- gravity is -y), so only x and y are ever dragged.
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
GAUSS_GOAL_XY = (-0.25, 0.0)   # [m] where the castle stands. Further out than
                               # the button fixture's -0.15, so it is gauss_seg2
                               # (s=2/3) that does the work here: with the body
                               # curving to about -17 cm it sits ~8 cm = 1.4
                               # sigma from the centre, near the force peak,
                               # while gauss_seg1 at ~-6 cm is 19 cm away and
                               # contributes about 1 N. x must stay more
                               # negative than the body reaches, or the push
                               # points AT the button instead of away from it.
GOAL_LINK   = [GAUSS_COMPONENTS]

# Live from the start, like every other folder. The tip goal sits 5.5 cm below
# the resting tip, so the arm takes up that slack (~1.65 N at K_REACH = 30) as
# soon as the torque node is up, while the Gaussians ramp in. Untick "Publisher
# Active" in the GUI to place the goals quietly.
PUBLISH_ON_START = True

# ── Ending a session ─────────────────────────────────────────────────────────
# "Home & Quit" in the GUI: back to MultiGoHome.HOME_POSITION in extended
# position control, then the window closes. Half the speed the button sweep
# homes at (run_sweep.py --home-velocity 80, ~18.3 rpm) -- that one runs with
# nothing near the arm, this one sweeps back past a stack of cards.
#
# The arm is handed back in CURRENT mode at the end, so it goes limp at
# HOME_POSITION. Check the castle is not under where it comes to rest.
# Closing this GUI -- by "Home & Quit" or by the window's X -- also tells the
# torque node to write its run and exit, so terminals 2 and 3 both end with one
# action instead of a Ctrl+C each. "Home & Quit" does it FIRST, before it stops
# the driver and homes, because saving is the step that cannot be redone.
END_RUN_ON_CLOSE = True
END_RUN_TIMEOUT  = 30.0   # [s] to let the save finish before the bus is taken

HOME_BUTTON   = True
HOME_VELOCITY = 40      # x0.229 rpm -> ~9.2 rpm
HOME_TIMEOUT  = 120.0   # [s] the arm ends a run about 1.15 turns wound

# ── GUI framing ──────────────────────────────────────────────────────────────
DRAG_SPEED_MAX = 100.0     # [cm/s] top of the drag-speed slider -- the goal
                           #        crosses the whole range in a second at it
XY_RANGE = (-1.00, 1.00)   # [m] the x-y plot's extent, and the clamp on a
                           #     dragged goal. Wide on purpose: the target sits
                           #     beyond the arm's 43.5 cm reach, and the whole
                           #     workspace plus the obstacle has to be placeable
                           #     without running into the edge. The cost is
                           #     precision -- the same drag covers more ground,
                           #     so drop "Drag speed" for fine placement.
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
