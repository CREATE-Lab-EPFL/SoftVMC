"""
Finger-space VMC example.

Virtual springs and dampers act directly on joint angles [theta_MCP, theta_PIP, theta_DIP].
Simpler than task-space VMC: no FK required, suitable for posture regulation and stiffness shaping.
Gravity, friction and joint-limit compensation are added on top.

Usage (from SoftVMC root):
    ros2 run dynamixel_interface dynamixel_node          # terminal 1
    python3 examples_and_ros2lib/fingerVMCexamples/FingerReferenceCodes/FingerVMCFingerSpace.py
"""

import sys, os
# fingerVMCexamples/ -> KinematicsFinger, VMCFinger
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
# SoftVMC/ -> VMC_utils
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

import numpy as np
import rclpy

from VMCFinger.FingerController import FingerController, CONTROL_FREQUENCY
from VMCFinger.FingerVMCFingerSpace import VMC
from VMCFinger.FingerGravFricLim import GravFricLim

rclpy.init()
controller   = FingerController()
vmc          = VMC()          # edit vmc.target / vmc.spring.stiffness to tune
compensation = GravFricLim()


def control_loop():
    # State from ROS2 topics (degrees -> radians)
    q_motor     = np.radians(controller.get_joint_positions())
    q_dot_motor = np.radians(controller.get_joint_velocities())

    # VMC torques (spring/damper on joint angles, mapped via J^T)
    tau_vmc  = vmc.finger_torques(q_motor, q_dot_motor)

    # Compensation (gravity + joint limits + Stribeck friction)
    tau_comp = compensation.compute_compensation_torques(q_motor, q_dot_motor, tau_vmc)

    controller.publish_torques(tau_vmc + tau_comp)


controller.create_timer(1.0 / CONTROL_FREQUENCY, control_loop)

try:
    rclpy.spin(controller)
except KeyboardInterrupt:
    pass
finally:
    controller.destroy_node()
    rclpy.shutdown()
