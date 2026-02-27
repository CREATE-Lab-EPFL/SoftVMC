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

from VMCSingle.SingleController import SingleController, CONTROL_FREQUENCY
from VMCSingle.SingleVMCTaskSpace import VMC

TENDON_RADIUS = 0.05  # [m] radius of the tendon spool

rclpy.init()
controller   = SingleController()
vmc          = VMC()          # edit vmc.target_DIP / vmc.spring_DIP.stiffness to tune

def control_loop():
    # State from ROS2 topics (degrees -> radians -> m via TENDON_RADIUS)
    DELTAL     = TENDON_RADIUS*np.radians(controller.get_joint_positions())
    DELTAL_dot = TENDON_RADIUS*np.radians(controller.get_joint_velocities())

    # VMC torques (Jacobian transpose of virtual spring/damper forces)
    tau_vmc  = vmc.single_torques(DELTAL, DELTAL_dot)
    
    controller.publish_torques(tau_vmc)

controller.create_timer(1.0 / CONTROL_FREQUENCY, control_loop)

try:
    rclpy.spin(controller)
except KeyboardInterrupt:
    pass
finally:
    controller.destroy_node()
    rclpy.shutdown()
