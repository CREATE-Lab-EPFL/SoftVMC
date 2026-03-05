import sys, os
# SoftVMC/ -> VMC_utils
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

import numpy as np
# import rclpy

# from SingleController import SingleController, CONTROL_FREQUENCY
from SingleVMCTaskSpace import VMC

from FK_Single import FK_Single_ee

CONTROL_FREQUENCY = 900
STEPS = 100
q_motor = np.array([0.0, 0.0, 0.0])  # [q1, q2, q3] motor angles [rad]
q_motor_dot = np.array([0.0, 0.0, 0.0])  # [q1_dot, q2_dot, q3_dot] motor velocities [rad/s]

q_motor_goal = np.array([0.0, 0.0, 0.0])  # target motor angles [rad]

# MAX DIST: +- 0.01 m in x, y, z from rest (0, 0, 0.145) [m]

vmc          = VMC(
    spring_const=40.0,
    damp_const=1.0,
    target_pos=np.array([0.01, 0.0, 0.140]) # feasible position? 
)


for i in range(STEPS):
    tau = vmc.single_torques(q_motor, q_motor_dot)
    
    # simulation motor: convert torque to motor angle updates taking into account CONTROL_FREQUENCY
    # compute q_motor_dot assuming we use CONTROL_FREQUENCY
    
    # compute updated pose
    pose =FK_Single_ee(q_motor)

    print(f"Step {i+1}/{STEPS}, Motor torques: {tau}, Motor angles: {q_motor} ")
    print(f"End-effector pose: {pose} [m, rad]")