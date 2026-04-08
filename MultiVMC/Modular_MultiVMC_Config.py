import numpy as np
from MultiVMCTaskSpace import VirtualComponent

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from VMC_utils.VirtualModels import LinearDamper, LinearSpring, TanhSpring, TanhDamper, GaussianSpring

"""
    This config file is used to facilitate testing virtual components and parameter adjustements.
    In theory, we could define VIRTUAL_COMPONENTS + subscriber configs in the subscriber and 
    DEFAULT_GOALS + publisher configs in the publisher, but since they are closely linked 
    (each component has a goal), we keep them together here for clarity.

    REQUIREMENT: NUMBER OF VIRTUAL COMPONENTS = NUMBER OF GOALS 
"""


# -------------- Virtual Components --------------

# Positions for DEFAULT_GOALS
POS_SCALE = 7
q1_short = np.array([0.01 * POS_SCALE,                   0.0,                              3 * 0.145 - 0.001])
q1_long = np.array([-0.01 * POS_SCALE,                   0.0,                              3 * 0.145 - 0.001])
q2_short = np.array([-0.01 * POS_SCALE * np.sin(np.pi/6), 0.01 * POS_SCALE * np.cos(np.pi/6), 3 * 0.145 - 0.001])
q3_short = np.array([-0.01 * POS_SCALE * np.sin(np.pi/6), -0.01 * POS_SCALE * np.cos(np.pi/6), 3 * 0.145 - 0.001])
rest_pos = np.array([0.0, 0.0, 3 * 0.145])

VIRTUAL_COMPONENTS = [
    VirtualComponent(
        spring=LinearSpring(25.0),
        damper=LinearDamper(10.0),
        s_param=1.0,
        name="robot_tip",
    ),
    # VirtualComponent(
    #     spring=LinearSpring(50.0),
    #     damper=LinearDamper(5.0),
    #     s_param=1.0 / 3.0,
    #     name="seg1_tip",
    # ),
]

# shape (N,3), one [x,y,z] per virtual component
DEFAULT_GOALS = np.array([
    q1_short,                 # robot_tip
    # q1_long,                  # seg1_tip
], dtype=float)

# Start position for ALL virtual components: [0, 0, 3*0.145]
# Same shape as DEFAULT_GOALS (N,3)
DEFAULT_START = np.tile(
    np.array([0.0, 0.0, 3 * 0.145], dtype=float),
    (DEFAULT_GOALS.shape[0], 1)
)

# -------------- Publisher Config --------------
# 100 steps per second = 1 cm/s  (reference velocity)
STEPS_M = 0.0001         # 0.1 mm 
PUBLISH_PERIOD = 0.01    # 100 Hz
LOOP_BACK = False       # ping-pong starts<->ends

# True  -> all components share one alpha timeline (arrive together even if distances differ)
# False -> each component uses its own step count / alpha timeline
SYNCHRONIZED_ARRIVAL = False


# -------------- Subscriber Config --------------
TAU_LIMIT = 1.0                    # [N·m] per motor safety limit
ANGLE_LIMIT_RAD = np.radians(120)  # [rad] per motor angle limit
SAVE_PLOT = True