"""
Task-space VMC example.

Virtual springs and dampers are placed at the MCP base and fingertip in Cartesian space.
The resulting forces are mapped to motor torques via J^T. Gravity, friction and joint-limit
compensation are added on top.

Usage (from SoftVMC root):
    ros2 run dynamixel_interface dynamixel_node          # terminal 1
    python3 examples_and_ros2lib/fingerVMCexamples/FingerReferenceCodes/FingerVMCPressing.py
"""

import sys, os
# SoftVMC/ -> VMC_utils
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

import numpy as np
import rclpy

from SingleController import SingleController, CONTROL_FREQUENCY
from SingleVMCTaskSpace import VMC
from FK_Single import FK_Single_ee

rclpy.init()
controller   = SingleController()

test1 = np.array([0.01, 0.01, 0.144]) # q1, q2 equally shorter, q3 longer
rest_pos = np.array([0.0, 0.0, 0.145])
# sin(pi/6) = 0.5, cos(pi/6) = 0.866
POS_SCALE = 3
q1_short = np.array([0.01*POS_SCALE, 0.0, 0.144]) # q1 shorter (tau1 neg), q2,q3 equaly longer (tau2,tau3 pos)
q2_short = np.array([-0.01*POS_SCALE*np.sin(np.pi/6), 0.01*POS_SCALE*np.cos(np.pi/6), 0.144]) # q2 shorter (tau2 neg), q1,q3 equally longer (tau1,tau3 pos)
q3_short = np.array([-0.01*POS_SCALE*np.sin(np.pi/6), -0.01*POS_SCALE*np.cos(np.pi/6), 0.144]) # q3 shorter (tau3 neg), q1,q2 equally longer (tau1,tau2 pos)

waypoints      = [q1_short, q2_short, q3_short]
# waypoints      = [q3_short, rest_pos]
waypoint_index = 0
q_motor_prev   = np.zeros(3)
stable_count   = -50             # negative to allow some initial settling time before the first switch
STABLE_STEPS   = 50              # must be stable for this many steps before switching
TAU_LIMIT      = 0.5
max_iter       = 6

vmc = VMC(spring_const=400.0, damp_const=1.0)

def control_loop():
    global waypoint_index, q_motor_prev, stable_count, max_iter

    # State from ROS2 topics (degrees -> radians)
    q_motor     = np.radians(controller.get_joint_positions())
    q_motor_dot = np.radians(controller.get_joint_velocities())

    if max_iter <= 0:
        # print("Max iterations reached. Stopping.")
        goal = rest_pos
    else:
        goal = waypoints[waypoint_index]
    # VMC torques (Jacobian transpose of virtual spring/damper forces)
    tau_vmc  = vmc.single_torques(q_motor, q_motor_dot, target_pos=goal)  # we can change the target on the fly for testing
    
     # Check if motors have stopped moving
    if np.linalg.norm(q_motor - q_motor_prev) < 0.01: 
        stable_count += 1
    else:
        stable_count = 0   # reset if still moving
    
    # Only switch after being stable for STABLE_STEPS consecutive steps
    if stable_count >= STABLE_STEPS:
        if max_iter > 0:
            print(f"Reached waypoint {waypoint_index}: "f"{waypoints[waypoint_index]}")
            print(f"Current q_motor: {q_motor}")
            print(f"Current position: {FK_Single_ee(q_motor)}\n")
        if max_iter == 0:
            print(f"Rest position reached.")
            print(f"Current q_motor: {q_motor}")
            print(f"Current position: {FK_Single_ee(q_motor)}")
        stable_count   = 0
        waypoint_index = (waypoint_index + 1) % len(waypoints) # update waypoint 
        max_iter -= 1

    q_motor_prev = q_motor.copy()

    # limit torque to 0.5 N·m for safety
    if np.any(np.abs(tau_vmc) > TAU_LIMIT):
        print("Torque limit exceeded! Clipping.")
    tau_vmc = np.clip(tau_vmc, -TAU_LIMIT, TAU_LIMIT)

    controller.publish_torques(tau_vmc)

controller.create_timer(1.0 / CONTROL_FREQUENCY, control_loop)

try:
    rclpy.spin(controller)
except KeyboardInterrupt:
    pass
finally:
    controller.destroy_node()
    rclpy.shutdown()
