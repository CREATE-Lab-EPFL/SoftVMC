import numpy as np
from MultiVMCTaskSpace import VirtualComponent

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from VMC_utils.VirtualModels import LinearDamper, LinearSpring, TanhSpring, TanhDamper, GaussianSpring
from JacobiansMulti import DELTA1, DELTA2, DELTA3
"""
    This config file is used to facilitate testing virtual components and parameter adjustements.
    In theory, we could define VIRTUAL_COMPONENTS + subscriber configs in the subscriber and 
    DEFAULT_GOALS + publisher configs in the publisher, but since they are closely linked 
    (each component has a goal), we keep them together here for clarity.

    REQUIREMENT: NUMBER OF VIRTUAL COMPONENTS = NUMBER OF GOALS 
    Assigning goals: Once s_param is defined for each component, choose goal position by adjusting Z_SEG1,2,3 in q.._short
                     where Z_SEG can be deduced from the s_param. Not an obbligation but it should work better by doing this. 
    Motor End-Effector Goals allows us to visualize the torque produced by each component more easily, since the goal is directly on the motor position. 
    
"""
# -------------- Motor End-Effector Goals  --------------

# ---- Goal helpers ----
POS_SCALE0 = 12
SCALE1 = 19
SCALE2 = 12
R = 0.01 * POS_SCALE0 # scaling 1 cm by POS_SCALE

def goal_from_angle(theta: float, z: float, r: float = R) -> np.ndarray:
    return np.array([r * np.cos(theta), r * np.sin(theta), z], dtype=float)

# z per segment -> change according to desired segment
Z_SEG1 = 1 * 0.145 - 0.01
Z_SEG2 = 2 * 0.145 - 0.01
Z_SEG3 = 3 * 0.145 - 0.01

# ---- DELTA1 => q11..q13 (motors attached to segment 1)----
q11_short = goal_from_angle(DELTA1[0], Z_SEG3)
q12_short = goal_from_angle(DELTA1[1], Z_SEG3)
q13_short = goal_from_angle(DELTA1[2], Z_SEG3)

# ---- DELTA2 => q14..q16 (motors attached to segment 2) ----
q14_short = goal_from_angle(DELTA2[0], Z_SEG3) 
q15_short = goal_from_angle(DELTA2[1], Z_SEG3)
q16_short = goal_from_angle(DELTA2[2], Z_SEG3, r=0.01* SCALE1)

# ---- DELTA3 => q17..q19 (motors attached to segment 3) ----
q17_short = goal_from_angle(DELTA3[0], Z_SEG3, r=0.01 * SCALE2)
q18_short = goal_from_angle(DELTA3[1], Z_SEG3)
q19_short = goal_from_angle(DELTA3[2], Z_SEG3)

# Segment 3 EE rest position
rest_pos = np.array([0.0, 0.0, 3 * 0.145], dtype=float)
gauss_pos = goal_from_angle(DELTA3[2], 0.145)

# -------------- Virtual Components --------------

# VIRTUAL_COMPONENTS = [
#     VirtualComponent(
#         spring=LinearSpring(50.0),
#         damper=LinearDamper(5.0),
#         s_param=1.0,
#         name="seg3_tip_q17",
#     ),
#     VirtualComponent(
#         spring=LinearSpring(230.0),
#         damper=LinearDamper(5.0),
#         s_param=1.0 / 3.0,
#         name="seg1_tip_q11",
#     ),
# ]

VIRTUAL_COMPONENTS = [
    VirtualComponent(
        spring=LinearSpring(150.0),
        damper=LinearDamper(5.0),
        s_param=1.0,
        name="seg3_tip_q19",
    ),
    VirtualComponent(
        spring=GaussianSpring(strength=400.0, sigma=0.06),
        damper=LinearDamper(5.0),
        s_param=1.0 / 3.0,
        name="seg1_tip_q19",
    ),
    VirtualComponent(
        spring=GaussianSpring(strength=330.0, sigma=0.06),
        damper=LinearDamper(5.0),
        s_param=2.0 / 3.0,
        name="seg2_tip_q19",
    ),
]

# shape (N,3), one [x,y,z] per virtual component
DEFAULT_GOALS = np.array([
    q19_short,            
    gauss_pos,
    gauss_pos,              
], dtype=float)

# Start position for each virtual component depends on s_param:
def _start_from_s_param(s: float) -> np.ndarray:
    if s <= (1.0 / 3.0):   # segment 1
        s_local = s * 3
        z = 1 * 0.145 * s_local
    elif s <= (2.0 / 3.0): # segment 2
        s_local = (s - 1/3) * 3
        z = 2 * 0.145 * s_local
    else:                  # segment 3
        s_local = (s - 2/3) * 3
        z = 3 * 0.145 * s_local
    return np.array([0.0, 0.0, z], dtype=float)

DEFAULT_START = np.array(
    [_start_from_s_param(comp.s_param) for comp in VIRTUAL_COMPONENTS],
    dtype=float
)

# -------------- Publisher Config --------------
# 100 steps per second = 1 cm/s  (reference velocity)
STEPS_M = 0.0008         # 0.1 mm 
PUBLISH_PERIOD = 0.01    # 100 Hz
LOOP_BACK = False       # ping-pong starts<->ends

# True  -> all components share one alpha timeline (arrive together even if distances differ)
# False -> each component uses its own step count / alpha timeline
SYNCHRONIZED_ARRIVAL = False

# -------------- Subscriber Config --------------
TAU_LIMIT = 0.15                    # [N·m] per motor safety limit
ANGLE_LIMIT_RAD = np.radians(540)  # NO LONGER USED IN SUBSCRIBER
SAVE_PLOT = True
PLOT_Z = True   # True -> include z-position subplot, False -> only x and y