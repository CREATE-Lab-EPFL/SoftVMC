import numpy as np

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from VMC_utils.GravityCompensation import GravityCompensation

from JacobiansMulti import q_motor2pose

SEGMENT_MASS = 0.040  # [kg] mass of each segment (adjust as needed)

class MultiGravComp:
    """
    Gravity compensation for a 3-segment continuum robot (9 motors).

    Method:
      - Approximate each segment by one lumped mass located at a chosen backbone
        point s in [0, 1].
      - Use positional Jacobian J_pos(s) (top 3 rows of q_motor2pose(s), shape 3x9).
      - Map compensation force to motor torques via tau = J_pos^T * F_comp.
      - Sum contributions of all lumped masses.
    """

    def __init__(
        self,
        masses=(SEGMENT_MASS, SEGMENT_MASS, SEGMENT_MASS),    # [kg] one lump per segment
        s_param=(1.0 / 6.0, 0.5, 5.0 / 6.0),                  # Center of mass points for 3 segments
        R_world_to_system=np.eye(3),                          # robot frame = world frame (z up)
        gravity=9.81
    ):
        self.masses = np.asarray(masses, dtype=float)
        self.s_param = np.asarray(s_param, dtype=float)

        # One GravityCompensation object per lumped mass
        self.gravity_comp = [
            GravityCompensation(
                mass=float(m),
                R_world_to_system=R_world_to_system,
                gravity=gravity
            )
            for m in self.masses
        ]

        # Build Jacobian callables once
        # q_motor2pose(s) returns J: d([x,y,z,yaw,pitch,roll]) / d(q_motor), shape (6,9)
        self.J_pose_func = [q_motor2pose(float(s)) for s in self.s_param]

    def _jac_pos(self, q_motor, i):
        """
        Position Jacobian (3x9) for lump i at s_param[i].
        """
        J_pose = np.array(self.J_pose_func[i](*q_motor), dtype=float)  # (6,9)
        return J_pose[:3, :]                                           # (3,9)

    def gravity_compensation(self, q_motor):
        """
        Gravity compensation torques [N·m], shape (9,).
        """
        q_motor = np.asarray(q_motor, dtype=float).reshape(-1)
        tau = np.zeros(9, dtype=float)

        for i in range(len(self.masses)):
            J_pos = self._jac_pos(q_motor, i)                 # (3,9)
            F_comp = self.gravity_comp[i].compute_force()     # (3,)
            tau += J_pos.T @ F_comp                           # (9,)

        return tau


if __name__ == "__main__":
    # Quick sanity test at zero configuration
    gc = MultiGravComp(
        masses=(SEGMENT_MASS, SEGMENT_MASS, SEGMENT_MASS),
        s_param=(1.0 / 6.0, 0.5, 5.0 / 6.0),                # Center of mass points for 3 segments
        R_world_to_system=np.eye(3),
        gravity=9.81
    )

    q0 = np.zeros(9)
    tau_g = gc.gravity_compensation(q0)

    np.set_printoptions(precision=6, suppress=True)
    print("tau_gravity at q=0 [N·m]:", tau_g)