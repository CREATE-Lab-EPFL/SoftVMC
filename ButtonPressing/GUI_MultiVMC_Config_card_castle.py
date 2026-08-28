"""
House of cards -- one spring pulling the tip onto it, one pushing it off.

    python3 ../VMC/GUI_MultiVMC_Torque.py --config _card_castle --run <case>   # terminal 2
    python3 castle_demo.py --config _card_castle --run <case>                  # terminal 3

NO GUI. Two elements, both attached to the TIP (s = 1.0), with two different
targets -- the castle sits between the tip and where it is being sent:

  lin_tip    LinearSpring(K_REACH)     pulls the tip towards TARGET_XY (0, 0.40)
  gauss_tip  GaussianSpring(k, SIGMA)  pushes it away from CASTLE_XY (0, 0.20)

So the tip is asked to cross the castle, and the field is the only thing in the
way. Where it stalls is where the two cancel along the path -- with 35 against
500 that is 10.8 cm short of the cards, and it is STABLE: nearer the push wins
and the tip is driven back, further out the pull wins and it is drawn in.

    --run no_gauss    no field -- the tip goes for the target THROUGH the cards
    --run gauss       the field on at 500 N/m -- the tip stalls 10.8 cm short

The barrier is finite, not a wall. Past an unstable point 1.6 cm from the
castle the push falls off faster than the pull and the tip would pop through to
the far side; approaching from the start it never gets there, stopping at the
stable point instead. `castle_demo.py --dry-run` solves for both and prints
them, so a change to K_REACH, k_gauss, SIGMA or TARGET_XY can be checked before
the cards are built.

The run is scripted end to end (castle_demo.py), in this order:

    1. the field is brought up 0 -> k over RAMP_S, slowly, with the tip goal
       still on the tip so the linear spring pulls nothing
    2. only then does the tip goal walk to the castle, at DEMO_SPEED
    3. the arm settles, the window is marked, the run is written

Nothing is dragged by hand, so the two cases differ in the field and in nothing
else.

Both goals' z follows the tip's own z every tick (GOAL_Z_TRACK), so the whole
demo happens in one x-y plane and neither element pulls along the arm's axis.
"""

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
from VMC.MultiVMCTaskSpace import VirtualComponent
from VMC_utils.VirtualModels import LinearSpring, LinearDamper, GaussianSpring
from VMC_utils.ConfigHelpers import default_start, param_info, rest_pose

CONFIG_NAME = "card_castle"

# ── Virtual elements ─────────────────────────────────────────────────────────
K_REACH = 35.0    # [N/m]   tip spring, pulling onto the castle
K_GAUSS = 0.0     # [N/m]   the run's case sets this; see GAUSS_CONDITIONS
SIGMA   = 0.06    # [m]     Gaussian width
C_DAMP  = 3.0     # [N.s/m] both -- 3x the button's 1.0. The two springs oppose
                  #         each other here and the damper is what stops the tip
                  #         oscillating about the point where they balance.

GAUSS_COMPONENTS = ("gauss_tip",)   # one obstacle, one attachment point: the tip

VIRTUAL_COMPONENTS = [
    VirtualComponent(spring=LinearSpring(K_REACH),
                     damper=LinearDamper(C_DAMP),
                     s_param=1.0, name="lin_tip"),
    VirtualComponent(spring=GaussianSpring(strength=K_GAUSS, sigma=SIGMA),
                     damper=LinearDamper(C_DAMP),
                     s_param=1.0, name="gauss_tip"),
]

# ── The two cases ────────────────────────────────────────────────────────────
# The pair IS the experiment: the same scripted approach with the field and
# without. Nothing in between.
#
# 500 against K_REACH = 35 stalls the tip 10.8 cm short of the cards. The stall
# point moves out only slowly with strength -- the balance is logarithmic in the
# ratio -- so SIGMA and TARGET_XY are the better handles if 10.8 cm is wrong.
GAUSS_CONDITIONS = {
    "no_gauss":      0.0,
    "gauss":      1500.0,
}

# --run is not optional here: an unnamed run belongs to neither case.
REQUIRE_RUN_NAME = True

# The field cannot be switched on at once, and step 1 of the demo is to bring it
# up gradually. The tip goal stays put until the ramp is done, so the linear
# spring pulls nothing while it happens.
RAMP_COMPONENTS = GAUSS_COMPONENTS
RAMP_S          = 5.0    # [s]   0 disables the ramp

# ── Controller ───────────────────────────────────────────────────────────────
TAU_LIMIT   = 0.35   # [N·m] per motor -- as _person, above the button's 0.25
GRAVITY_DIR = "-y"   # arm rotated onto its side: gravity pulls along -y, not -z

PRETENSION_ENABLED   = True
PRETENSION_WEIGHTS   = np.full(9, 0.010, dtype=float)   # [N·m/rad] x 9

# ── The demo, step by step ───────────────────────────────────────────────────
# castle_demo.py reads these, so both cases run to the same clock and --run is
# the only thing that differs between them.
DEMO_SETTLE_S = 5.0    # [s] after the field's ramp: let the arm settle first
DEMO_SPEED    = 5.0    # [cm/s] the tip goal walks to the target at this
DEMO_ARRIVE_S = 8.0    # [s] hold after it gets there, so the tip can settle
DEMO_WINDOW_S = 10.0   # [s] the measurement window, after the mark
                       #     -> 36 s in total. --settle/--speed/--arrive/--window
                       #        override any of them for a single run.

# ── Output ───────────────────────────────────────────────────────────────────
OUTPUT_FOLDER  = "card_castle"       # --run NAME lands in outputs/<folder>/NAME.*
SAVE_PLOT      = True
SAVE_DATA      = True
COLLECTED_DATA = False   # True = never save, the recorded runs are final
AUTOSAVE_S     = 30.0    # [s] checkpoint to <run>.autosave.npz; 0 disables

# One mark, put in by castle_demo.py when the tip has arrived and settled -- the
# analysis takes its measurement window from there.
MARK_PROMPTS   = ("Target reached?",)

PLOT_Z         = True
PLOT_FORCE     = False   # no load cell in this variant

# ── Geometry ─────────────────────────────────────────────────────────────────
# Two points define the demo, and they are NOT the same: the castle, and where
# the tip is being sent. The castle sits between the start and the target, so
# the tip is asked to cross it.
CASTLE_XY     = (0.0, 0.25)    # [m] the cards. x across, y UP (gravity is -y).
                               #     Measure this off the real castle.
TARGET_XY     = (0.0, 0.50)    # [m] where lin_tip pulls -- beyond the castle,
                               #     on the far side, so the straight path to it
                               #     runs through the cards.
TIP_GOAL_XY   = (0.0, 0.0)     # [m] where the tip goal STARTS: on the tip
                               #     itself, so nothing pulls until it walks
GAUSS_GOAL_XY = CASTLE_XY      # the obstacle, fixed for the whole run

# Both goals' z is set to the tip's own z every tick, so the demo happens in one
# x-y plane and neither element pulls along the arm's axis. GOAL_Z_LOCK is only
# the value before the first FK -- the straight arm's tip, which is already it.
GOAL_Z_TRACK = ("lin_tip", "gauss_tip")
GOAL_Z_LOCK  = {_c.name: float(rest_pose(_c.s_param)[2]) for _c in VIRTUAL_COMPONENTS}
TIP_GOAL_Z   = GOAL_Z_LOCK["lin_tip"]     # [m] = 0.435

# NOT linked: the tip goal walks to the castle while the castle stays where it
# is. Linking them would drag the obstacle along with the target.
GOAL_LINK    = []

PUBLISH_ON_START = True

# ── Ending the run ───────────────────────────────────────────────────────────
# castle_demo.py homes the arm itself when the run is written -- stop the
# driver (which closes start_arm.sh), home in extended position control, hand
# back in current mode. The arm goes LIMP at HOME_POSITION: check the castle is
# not under where it comes to rest.
HOME_VELOCITY = 40      # x0.229 rpm -> ~9.2 rpm
HOME_TIMEOUT  = 120.0   # [s]

# ── GUI framing (unused -- this variant has no GUI) ──────────────────────────
DRAG_SPEED_MAX = 100.0
XY_RANGE = (-1.00, 1.00)   # [m]
Z_RANGE  = (0.15, 0.50)    # [m]

# ── Derived ──────────────────────────────────────────────────────────────────
DEFAULT_START = default_start(VIRTUAL_COMPONENTS)
for _i, _c in enumerate(VIRTUAL_COMPONENTS):
    DEFAULT_START[_i, 2] = GOAL_Z_LOCK[_c.name]
    DEFAULT_START[_i, :2] = (GAUSS_GOAL_XY if _c.name in GAUSS_COMPONENTS
                             else TIP_GOAL_XY)
N_VC          = len(VIRTUAL_COMPONENTS)

(SPRING_PARAM_NAMES, DAMPER_PARAM_NAMES,
 SPRING_PARAM_INIT,  DAMPER_PARAM_INIT) = param_info(VIRTUAL_COMPONENTS)
