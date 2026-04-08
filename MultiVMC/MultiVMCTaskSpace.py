import numpy as np
from dataclasses import dataclass
from typing import List, Sequence

from FK_Multi import FK_Multi, FK_Multi_ee
from JacobiansMulti import q_motor2pose, q_motor2pose_ee

@dataclass
class VirtualComponent:
    spring: object
    damper: object
    s_param: float
    name: str = ""


class VMC_Multi_EE:
    """
    Task-space VMC for the multi-segment robot with end-effector pose as the task variable.
    A virtual spring attached between end-effector and target position.
    A virtual damper on the end-effector velocity.
    Forces are mapped to motor torques via Jacobian transpose: tau = J^T @ F_virtual
    """

    def __init__(self, spring, damper):
        # Jacobian callable (built once)
        self.J_q2p = q_motor2pose_ee()

        # Virtual spring [N/m] and damper [N·s/m]
        self.spring = spring
        self.damper = damper

    def multi_torques(self, q_motor, q_motor_dot, target_pos=None):
        """
        Args:
            q_motor:     [q11,q12,q13, q21,q22,q23, q31,q32,q33] motor angles [rad], shape (9,)
            q_motor_dot: motor velocities [rad/s], shape (9,)
            target_pos:  [x, y, z] target position [m], shape (3,)
        Returns:
            tau: motor torques [N·m], shape (9,)
        """
        q_motor = np.asarray(q_motor, dtype=float).reshape(-1)
        q_motor_dot = np.asarray(q_motor_dot, dtype=float).reshape(-1)

        J = np.array(self.J_q2p(*q_motor), dtype=float)   # (6, 9)
        pose = np.asarray(FK_Multi_ee(q_motor), dtype=float).reshape(-1)  # (6,)
        pose_dot = J @ q_motor_dot  # (6,)

        # Position-only VMC (ignore orientation terms)
        pos = pose[:3]
        vel = pose_dot[:3]
        J_xyz = J[:3, :]  # (3,9)

        F_virtual = self.spring.compute_force(pos, target_pos) + self.damper.compute_force(vel)  # (3,)
        tau = J_xyz.T @ F_virtual  # (9,)
        return tau


class VMC_Multi:
    """
    Task-space VMC for the multi-segment robot.
    A virtual spring attached between ANY point along the robot (point parametrized by s_param) and target position.
    A virtual damper on the velocity of the point defined by s_param.
    Forces are mapped to motor torques via Jacobian transpose: tau = J^T @ F_virtual
    """

    def __init__(self, spring, damper, s_param=1.0):
        # Jacobian callable (built once)
        self.J_q2p = q_motor2pose(s_param)

        # Virtual spring [N/m] and damper [N·s/m]
        self.spring = spring
        self.damper = damper
        self.s_param = s_param

    def multi_torques(self, q_motor, q_motor_dot, target_pos=None):
        """
        Args:
            q_motor:     [q11,q12,q13, q21,q22,q23, q31,q32,q33] motor angles [rad], shape (9,)
            q_motor_dot: motor velocities [rad/s], shape (9,)
            target_pos:  [x, y, z] target position [m], shape (3,)
        Returns:
            tau: motor torques [N·m], shape (9,)
        """
        q_motor = np.asarray(q_motor, dtype=float).reshape(-1)
        q_motor_dot = np.asarray(q_motor_dot, dtype=float).reshape(-1)

        J = np.array(self.J_q2p(*q_motor), dtype=float)   # (6, 9)
        pose = np.asarray(FK_Multi(q_motor, self.s_param), dtype=float).reshape(-1)  # (6,)
        pose_dot = J @ q_motor_dot  # (6,)

        # Position-only VMC (ignore orientation terms)
        pos = pose[:3]
        vel = pose_dot[:3]
        J_xyz = J[:3, :]  # (3,9)

        F_virtual = self.spring.compute_force(pos, target_pos) + self.damper.compute_force(vel)  # (3,)
        tau = J_xyz.T @ F_virtual  # (9,)
        return tau

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

    # def _reshape_targets(self, target_pos):
    #     t = np.asarray(target_pos, dtype=float).reshape(-1)
    #     n = len(self.components)
    #     if t.size != 3 * n:
    #         raise ValueError(
    #             f"Expected target_pos size {3*n} for {n} components, got {t.size}."
    #         )
    #     return t.reshape(n, 3)

    def multi_torques(self, q_motor, q_motor_dot, target_pos):
        q_motor = np.asarray(q_motor, dtype=float).reshape(-1)          # (9,)
        q_motor_dot = np.asarray(q_motor_dot, dtype=float).reshape(-1)  # (9,)
        targets = np.asarray(target_pos, dtype=float).reshape(len(self.components), 3)
        # targets = self._reshape_targets(target_pos)  # (n_comp,3)

        tau_total = np.zeros(9, dtype=float)

        for i, comp in enumerate(self.components):
            J = np.array(self.J_funcs[i](*q_motor), dtype=float)     # (6,9)
            pose = np.asarray(FK_Multi(q_motor, comp.s_param), dtype=float).reshape(-1)  # (6,)
            pose_dot = J @ q_motor_dot                                # (6,)

            pos = pose[:3]
            vel = pose_dot[:3]
            J_xyz = J[:3, :]                                          # (3,9)

            F_virtual = comp.spring.compute_force(pos, targets[i]) + comp.damper.compute_force(vel)
            tau_i = J_xyz.T @ F_virtual                               # (9,)
            tau_total += tau_i

        return tau_total

# Example usage
# if __name__ == "__main__":
#     vmc = VMC_Multi_EE(gauss_spring, lin_damper)  # import from VMC_utils
#     q_motor = np.zeros(9)      # motor angles [rad]
#     q_motor_dot = np.zeros(9)  # motor velocities [rad/s]
#     target_pos = np.array([0.0, 0.0, 3 * 0.145])  # target position [m]
#     tau = vmc.multi_torques(q_motor, q_motor_dot, target_pos)
#     print("Motor torques:", tau)