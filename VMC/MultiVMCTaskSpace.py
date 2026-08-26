import sys, os
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_KINE = os.path.join(_ROOT, 'Kinematics')
for _p in [_ROOT, _KINE]:
    if _p not in sys.path: sys.path.insert(0, _p)

import numpy as np
from dataclasses import dataclass
from typing import List, Sequence

from FK_Multi import FK_Multi
from JacobiansMulti import q_motor2pose

@dataclass
class VirtualComponent:
    spring: object
    damper: object
    s_param: float
    name: str = ""


class VMC_Multi_Modular:
    """
    Multi-component task-space VMC.

    components: list of VirtualComponent(s), each with its own spring/damper/s_param.
    target_pos: flattened list/array of length 3*n_comp:
                [x1,y1,z1, x2,y2,z2, ..., xN,yN,zN]
    Output torque is sum of all component torques.
    """
    # Sequence: generic iterable (e.g. list or tuple) of VirtualComponent objects
    def __init__(self, components: Sequence[VirtualComponent]):
        # List: Python list whose elements are of type VirtualComponent
        self.components: List[VirtualComponent] = list(components)

        # Prebuild Jacobians for each component s_param
        self.J_funcs = [q_motor2pose(c.s_param) for c in self.components]

    def multi_torques_and_forces(self, q_motor, q_motor_dot, target_pos):
        """
        (tau_total, F_total) -- F_total is the SUM of the per-component forces.

        Those forces act at different points along the backbone, so their sum
        is a bookkeeping quantity, not a force on any one body. Use
        multi_torques_forces_states() when you need them apart.
        """
        state = self.multi_torques_forces_states(q_motor, q_motor_dot, target_pos)
        return state["tau"], state["F_comp"].sum(axis=0)

    def multi_torques_forces_states(self, q_motor, q_motor_dot, target_pos):
        """
        One control step, with everything it computed on the way out.

        The forward kinematics and the Jacobian of every component are already
        evaluated here; returning them means a caller that logs positions does
        not pay for a second FK pass, and logs exactly the pose the torque was
        computed from rather than a slightly later one.

        Returns a dict:
            tau       (9,)      [N.m]   summed motor torque
            tau_comp  (N, 9)    [N.m]   each component's share
            F_comp    (N, 3)    [N]     each component's task-space force
            pos       (N, 3)    [m]     FK position of each attachment point
            vel       (N, 3)    [m/s]   its task-space velocity
        """
        q_motor = np.asarray(q_motor, dtype=float).reshape(-1)          # (9,)
        q_motor_dot = np.asarray(q_motor_dot, dtype=float).reshape(-1)  # (9,)
        n = len(self.components)
        targets = np.asarray(target_pos, dtype=float).reshape(n, 3)

        tau_comp = np.zeros((n, 9), dtype=float)
        F_comp   = np.zeros((n, 3), dtype=float)
        pos_comp = np.zeros((n, 3), dtype=float)
        vel_comp = np.zeros((n, 3), dtype=float)

        for i, comp in enumerate(self.components):
            J = np.array(self.J_funcs[i](*q_motor), dtype=float)     # (6,9)
            pose = np.asarray(FK_Multi(q_motor, comp.s_param), dtype=float).reshape(-1)  # (6,)
            pose_dot = J @ q_motor_dot                                # (6,)

            pos = pose[:3]
            vel = pose_dot[:3]
            J_xyz = J[:3, :]                                          # (3,9)

            F_virtual = comp.spring.compute_force(pos, targets[i]) + comp.damper.compute_force(vel)
            tau_comp[i] = J_xyz.T @ F_virtual                         # (9,)
            F_comp[i]   = F_virtual
            pos_comp[i] = pos
            vel_comp[i] = vel

        return dict(tau=tau_comp.sum(axis=0), tau_comp=tau_comp,
                    F_comp=F_comp, pos=pos_comp, vel=vel_comp)
