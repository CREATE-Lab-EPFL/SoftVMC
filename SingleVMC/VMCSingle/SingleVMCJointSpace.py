import numpy as np
from VMC_utils.VirtualModels import LinearSpring, LinearDamper
from KinematicsSingle.SINGLE_jacobians import tendon2q
from KinematicsSingle.FK_Single import FK_tendon2q

class VMC:
    """
    Joint-space for the single segment.

    Virtual springs and dampers act directly on q-param [Dx, Dy, Dl].
    Torques are mapped to motors via Jacobian transpose:  tau_motor = J^T @ F_q 
    """

    def __init__(self):
        # Jacobian q-param [Dx, Dy, Dl] -> DELTAL [D1, D2, D3] (built once) 
        self.J_tq = tendon2q() 

        # Virtual spring [N·m/rad] and damper [N·m·s/rad]  — tune these
        self.spring = LinearSpring(0.2)
        self.damper = LinearDamper(0.03)

        # Target q values [mm]  — tune this
        self.target = np.array([10.0, 0.0, 10.0])   # [Dx, Dy, Dl]

    def finger_torques(self, DELTAL, q_dot_motor):
        """
        Args:
            DELTAL:     [DELTAL1, DELTAL2, DELTAL3] tendon lengths [mm]
            DELTAL_dot: [DELTAL1_dot, DELTAL2_dot, DELTAL3_dot] tendon velocities [mm/s]
        Returns:
            tau: [tau_1, tau_2, tau_3] motor torques [N·m]
        """
        J     = np.array(self.J_tq(DELTAL[0], DELTAL[1], DELTAL[2]))   # (3, 3)
        q     = FK_tendon2q(DELTAL[0], DELTAL[1], DELTAL[2])         # current joint (3,)
        q_dot = J @ q_dot_motor                                    # joint velocities (3,)

        tau_joint = self.spring.compute_force(q, self.target) \
                  + self.damper.compute_force(q_dot)               # virtual joint torques (3,)
        return J.T @ tau_joint                                         # motor torques (2,)
