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

rclpy.init()
controller   = SingleController()

vmc          = VMC(
    spring_const=40.0,
    damp_const=1.0,
    target_pos=np.array([0.100, 0.010, 0.135])
)

def control_loop():
    # State from ROS2 topics (degrees -> radians)
    q_motor     = np.radians(controller.get_joint_positions())
    q_motor_dot = np.radians(controller.get_joint_velocities())

    # VMC torques (Jacobian transpose of virtual spring/damper forces)
    tau_vmc  = vmc.single_torques(q_motor, q_motor_dot)
    
    controller.publish_torques(tau_vmc)

controller.create_timer(1.0 / CONTROL_FREQUENCY, control_loop)

try:
    rclpy.spin(controller)
except KeyboardInterrupt:
    pass
finally:
    controller.destroy_node()
    rclpy.shutdown()
