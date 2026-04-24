import numpy as np
from MultiVMCTaskSpace import VirtualComponent

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from VMC_utils.VirtualModels import ConstrainedLinearSpring, LinearDamper, LinearSpring, TanhSpring, TanhDamper, GaussianSpring
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

TERMINAL 1
cd ros2_dynamixel && colcon build && source install/setup.bash
./ros2_run.sh
# sudo echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# ros2 run dynamixel_interface dynamixel_node --ros-args -p baudrate:=1000000 -p motor_ids:=[11,12,13,14,15,16,17,18,19] -p control_mode:=torque

TERMINAL 2 (WAIT FOR LOGGER INFO BEFORE RUNNING TERMINAL 3)
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
python3 Modular_MultiVMC_MovingGoal_Sub.py

TERMINAL 3 (ONLY AFTER running Modular_MultiVMC_MovingGoal_Sub.py)
source /opt/ros/humble/setup.bash
python3 Modular_MultiVMC_MovingGoal_Pub.py

TERMINAL 4 
python3 MultiGoHome.py --read (COPY OUTPUT TO MultiGoHome.py)
python3 MultiGoHome.py 

"""

# -------------- Virtual Component Goals  --------------

# for reference: z_seg1: 0.145 , z_seg2: 0.29, z_seg3: 0.435
rest_pos_seg3 = np.array([0.0, 0.0, 3 * 0.145], dtype=float)
go_y = np.array([0.0, 0.3, 0.3], dtype=float)
go_x = np.array([0.3, 0.0, 0.35], dtype=float)
gauss_y = np.array([0.0, 0.15, 0.2], dtype=float)


# -------------- Virtual Components + Goals  --------------
COMPONENT_SPECS = [
    dict(
        component=VirtualComponent(
            # spring=ConstrainedLinearSpring(300.0, np.array([0.0, 1.0, 0.0])),
            spring=LinearSpring(230.0),
            damper=LinearDamper(5.0),
            s_param=1.0,
            name="lin_y",
        ),
        goal=np.array([0.0, 0.15, 0.25], dtype=float),
    ),
    # dict(
    #     component=VirtualComponent(
    #         spring=LinearSpring(230.0),
    #         damper=LinearDamper(5.0),
    #         s_param=1.0,
    #         name="lin_x",
    #     ),
    #     goal=go_x,
    # ),
    # dict(
    #     component=VirtualComponent(
    #         spring=GaussianSpring(strength=400.0, sigma=0.06),
    #         damper=LinearDamper(5.0),
    #         s_param=1.0 / 3.0,
    #         name="gauss_y_seg1",
    #     ),
    #     goal=gauss_y,
    # ),
    # dict(
    #     component=VirtualComponent(
    #         spring=GaussianSpring(strength=330.0, sigma=0.06),
    #         damper=LinearDamper(5.0),
    #         s_param=2.0 / 3.0,
    #         name="gauss_y_seg2",
    #     ),
    #     goal=gauss_y,
    # ),
]

# Derived outputs used by subscriber/publisher
VIRTUAL_COMPONENTS = [spec["component"] for spec in COMPONENT_SPECS]
DEFAULT_GOALS = np.array([spec["goal"] for spec in COMPONENT_SPECS], dtype=float)  # shape (N,3)

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
TAU_LIMIT = 0.4                    # [N·m] per motor safety limit
SAVE_PLOT = True
PLOT_Z = True   # True -> include z-position subplot, False -> only x and y

# # ---- Previous configs (for reference) ----

# # ---- Goal helpers ----
# POS_SCALE0 = 12
# SCALE1 = 19
# SCALE2 = 12
# R = 0.01 * POS_SCALE0 # scaling 1 cm by POS_SCALE

# def goal_from_angle(theta: float, z: float, r: float = R) -> np.ndarray:
#     return np.array([r * np.cos(theta), r * np.sin(theta), z], dtype=float)

# # z per segment -> change according to desired segment
# Z_SEG1 = 1 * 0.145 - 0.01
# Z_SEG2 = 2 * 0.145 - 0.01
# Z_SEG3 = 3 * 0.145 - 0.15

# # ---- DELTA1 => q11..q13 (motors attached to segment 1)----
# q11_short = goal_from_angle(DELTA1[0], Z_SEG3)
# q12_short = goal_from_angle(DELTA1[1], Z_SEG3)
# q13_short = goal_from_angle(DELTA1[2], Z_SEG3)

# # ---- DELTA2 => q14..q16 (motors attached to segment 2) ----
# q14_short = goal_from_angle(DELTA2[0], Z_SEG3) 
# q15_short = goal_from_angle(DELTA2[1], Z_SEG3)
# q16_short = goal_from_angle(DELTA2[2], Z_SEG3, r=0.01* SCALE1)

# # ---- DELTA3 => q17..q19 (motors attached to segment 3) ----
# q17_short = goal_from_angle(DELTA3[0], Z_SEG3, r=0.01 * SCALE2)
# q18_short = goal_from_angle(DELTA3[1], Z_SEG3)
# q19_short = goal_from_angle(DELTA3[2], Z_SEG3) # [0.10392305    0.06   0.425     ]

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