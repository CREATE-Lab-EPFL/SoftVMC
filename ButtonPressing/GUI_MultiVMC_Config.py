import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
from VMC.MultiVMCTaskSpace import VirtualComponent
from VMC_utils.VirtualModels import (
    ConstrainedLinearSpring, LinearDamper, LinearSpring,
    TanhSpring, TanhDamper, GaussianSpring, DeadzoneLimitSpring
)

"""
Config for button-pressing VMC experiments.

Three scenarios:
    1. avoid   — Gaussian repulsive field keeps end-effector away from button
    2. exploit — linear spring presses end-effector into button surface
    3. minimize — tanh spring: light contact, bounded force
"""

# ── Reference positions ───────────────────────────────────────────────────────
rest_pos = np.array([0.0, 0.0, 3 * 0.145], dtype=float)  # straight-up pose

# Button surface position (to be measured on the real robot)
button_pos = np.array([0.05, 0.0, 0.38], dtype=float)

# ── Scenario: avoid (Gaussian repulsion from button) ─────────────────────────
K_GAUSS   = 0.05   # N·m stiffness of Gaussian field
SIGMA     = 0.03   # [m] width of Gaussian
K_DAMP    = 0.5    # N·m·s/rad
S_GAUSS   = 1.0    # s-parameter (end-effector)

VIRTUAL_COMPONENTS = [
    VirtualComponent(
        spring=GaussianSpring(k=K_GAUSS, sigma=SIGMA, center=button_pos),
        damper=LinearDamper(c=K_DAMP),
        s_param=S_GAUSS,
        name="gauss_repel",
    ),
]

DEFAULT_START = rest_pos
N_VC = len(VIRTUAL_COMPONENTS)
TAU_LIMIT = 1.5       # [N·m] per-motor torque clamp
XY_RANGE  = 0.15      # [m]  GUI plot half-range in XY
Z_RANGE   = (0.2, 0.55)
TAU_PLOT_WINDOW = 200 # samples
FK_S_SAMPLES    = 10
PLOT_VIRTUAL_FORCES = True

SPRING_PARAM_NAMES  = ["k", "sigma"]
DAMPER_PARAM_NAMES  = ["c"]
SPRING_PARAM_INIT   = [K_GAUSS, SIGMA]
DAMPER_PARAM_INIT   = [K_DAMP]
GUI_COLORS          = ["C1"]
