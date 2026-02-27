import numpy as np

import sys, os
# SoftVMC/ -> VMC_utils
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from VMC_utils.VirtualModels import LinearSpring, LinearDamper
from FK_Single import FK_Single
from JacobiansSingle import tendon2pose

class VMC:
    """
    Task-space VMC for the single segment.

    A Virtual spring and damper are attached at to Cartesian points (end-effector).
    Forces are mapped to motor torques via Jacobian transpose:  tau = J^T @ F_virtual
    """

    def __init__(self):
        # Jacobians (built once)
        self.J_tp = tendon2pose()

        # Virtual springs [N/m] and dampers [N·s/m]
        self.spring = LinearSpring(40.0)
        self.damper = LinearDamper(1.0)

        # Target Cartesian positions [m]  — tune these
        self.target = np.array([0.100, 0.010, 0.135])   #  rest length (0,0,.145) [m]

    def single_torques(self, DELTAL, DELTAL_dot):
        """
        Args:
            DELTAL:     [DELTAL1, DELTAL2, DELTAL3] tendon lengths [m]
            DELTAL_dot: [DELTAL1_dot, DELTAL2_dot, DELTAL3_dot] tendon velocities [m/s]
        Returns:
            tau: [tau_1, tau_2, tau_3] motor torques [N·m]
        """
        tau = np.zeros(3)
       
        J = np.array(self.J_tp(DELTAL[0], DELTAL[1], DELTAL[2]))   # (6, 3)
        pose = FK_Single(DELTAL) # [x, y, z, yaw, pitch, roll] [m, rad]
        pose_dot = J @ DELTAL_dot 

        # we focus only on the position part of the pose for the VMC, ignoring orientation for simplicity
        pos = pose[:3]
        vel = pose_dot[:3]
        J_xyz = J[:3, :]
        tau = J_xyz.T @ (self.spring.compute_force(pos, self.target) + self.damper.compute_force(vel))
        return tau

# Example usage
if __name__ == "__main__":
    vmc = VMC()
    DELTAL = np.array([0.0, 0.0, 0.0])  # tendon lengths [m]
    DELTAL_dot = np.array([0.0, 0.0, 0.0])  # tendon velocities [m/s]
    tau = vmc.single_torques(DELTAL, DELTAL_dot)
    print("Motor torques:", tau)