import numpy as np
from VMC_utils.VirtualModels import LinearSpring, LinearDamper
from KinematicsSingle.FK_Single import FK_Single
from KinematicsSingle.SINGLE_jacobians import tendon2pose

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
        self.target = np.array([0.100, 0.010, 0.135])   #  rest length (0,0,0.145) [m]

    def _jacobian(self, DELTAL):
        fn = self.J_tp
        return np.array(fn(DELTAL[0], DELTAL[1], DELTAL[2]))   # (6, 3)

    def single_torques(self, DELTAL, DELTAL_dot):
        """
        Args:
            DELTAL:     [DELTAL1, DELTAL2, DELTAL3] tendon lengths [mm]
            DELTAL_dot: [DELTAL1_dot, DELTAL2_dot, DELTAL3_dot] tendon velocities [mm/s]
        Returns:
            tau: [tau_1, tau_2, tau_3] motor torques [N·m]
        """
        tau = np.zeros(3)
       
        J = self._jacobian(DELTAL)
        pose = FK_Single(DELTAL) # [x, y, z, yaw, pitch, roll] [mm, rad]
        pose_dot = J @ DELTAL_dot 

        # we focus only on the position part of the pose for the VMC, ignoring orientation for simplicity
        pos = pose[:3]*1e-3     #  [mm] -> [m]
        vel = pose_dot[:3]*1e-3 #  [mm/s] -> [m/s]
        J_xyz = J[:3, :]  # no need for unit conversion for position mm/mm = m/m but needed for orientation rad/mm 
        tau = J_xyz.T @ (self.spring.compute_force(pos, self.target) + self.damper.compute_force(vel))
        return tau
