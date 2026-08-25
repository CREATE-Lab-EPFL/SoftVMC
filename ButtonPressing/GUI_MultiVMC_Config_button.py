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
from VMC_utils.ConfigHelpers import default_start, param_info

CONFIG_NAME = "button"

# ── Virtual elements ─────────────────────────────────────────────────────────
K_REACH = 10.0   # [N/m]   tip spring -- 25 was stiff enough to make it oscillate
K_GAUSS = 0.0    # [N/m]   both Gaussians start off; raise them from the GUI
SIGMA   = 0.06   # [m]     Gaussian width
C_DAMP  = 1.0    # [N.s/m] all three

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

# ── Controller ───────────────────────────────────────────────────────────────
TAU_LIMIT   = 0.25   # [N·m] per motor
GRAVITY_DIR = "-y"   # arm rotated onto its side: gravity pulls along -y, not -z

# tau_pretension[i] = PRETENSION_WEIGHTS[i] * q_motor[i], only above the threshold
PRETENSION_ENABLED   = True
PRETENSION_WEIGHTS   = np.full(9, 0.03, dtype=float)  # [N·m/rad] x 9
PRETENSION_THRESHOLD = np.deg2rad(30.0)               # [rad]

# ── Output ───────────────────────────────────────────────────────────────────
OUTPUT_FOLDER  = "button_pressing"   # --run NAME lands in outputs/<folder>/NAME.*
SAVE_PLOT      = True
SAVE_DATA      = True    # raw arrays to .npz alongside the plot
COLLECTED_DATA = False   # True = never save, the recorded runs are final
PLOT_Z         = True
PLOT_FORCE     = True    # load cell is used here

# ── GUI framing ──────────────────────────────────────────────────────────────
XY_RANGE = (-0.35, 0.35)   # [m]
Z_RANGE  = (0.15, 0.50)    # [m]

# ── Derived ──────────────────────────────────────────────────────────────────
DEFAULT_START = default_start(VIRTUAL_COMPONENTS)   # straight-arm rest pose
N_VC          = len(VIRTUAL_COMPONENTS)

(SPRING_PARAM_NAMES, DAMPER_PARAM_NAMES,
 SPRING_PARAM_INIT,  DAMPER_PARAM_INIT) = param_info(VIRTUAL_COMPONENTS)
