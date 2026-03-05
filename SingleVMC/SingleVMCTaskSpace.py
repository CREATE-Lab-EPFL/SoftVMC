import numpy as np

import sys, os
# SoftVMC/ -> VMC_utils
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from VMC_utils.VirtualModels import LinearSpring, LinearDamper
from FK_Single import FK_Single, FK_Single_ee
from JacobiansSingle import q_motor2pose_ee

class VMC:
    """
    Task-space VMC for the single segment.

    A Virtual spring and damper are attached at to Cartesian points (end-effector).
    Forces are mapped to motor torques via Jacobian transpose:  tau = J^T @ F_virtual
    """
    def __init__(self, spring_const=40.0, damp_const=1.0, target_pos=np.array([0.0, 0.0, 0.145])): # default rest pose
        # Jacobians (built once)
        self.J_tp = q_motor2pose_ee()

        # Virtual springs [N/m] and dampers [N·s/m]
        self.spring = LinearSpring(spring_const)
        self.damper = LinearDamper(damp_const)
 
        self.target = target_pos

    def single_torques(self, q_motor, q_motor_dot):
        """
        Args:
            q_motor:     [q1, q2, q3] motor angles [rad]
            q_motor_dot: [q1_dot, q2_dot, q3_dot] motor velocities [rad/s]
        Returns:
            tau: [tau_1, tau_2, tau_3] motor torques [N·m]
        """
        tau = np.zeros(3)
       
        J = np.array(self.J_tp(q_motor[0], q_motor[1], q_motor[2]))   # (6, 3)
        pose = FK_Single_ee(q_motor) # [x, y, z, yaw, pitch, roll] [m, rad]
        pose_dot = J @ q_motor_dot 

        # we focus only on the position part of the pose for the VMC, ignoring orientation for simplicity
        pos = pose[:3]
        vel = pose_dot[:3]
        J_xyz = J[:3, :]
        tau = J_xyz.T @ (self.spring.compute_force(pos, self.target) + self.damper.compute_force(vel))
        return tau

# Example usage
if __name__ == "__main__":
    vmc = VMC()
    q_motor = np.array([0.0, 0.0, 0.0])  # motor angles [rad]
    q_motor_dot = np.array([0.0, 0.0, 0.0])  # motor velocities [rad/s]
    tau = vmc.single_torques(q_motor, q_motor_dot)
    print("Motor torques:", tau)