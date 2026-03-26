import numpy as np
from FK_Multi import FK_Multi, FK_Multi_ee
from JacobiansMulti import q_motor2pose, q_motor2pose_ee

class VMC_Multi_EE:
    """
    Task-space VMC for the multi-segment robot with end-effector pose as the task variable.
    A virtual spring attached between end-effector and target position.
    A virtual damper on the end-effector velocity.
    Forces are mapped to motor torques via Jacobian transpose:  tau = J^T @ F_virtual
    """
    def __init__(self, spring, damper):
        # Jacobians (built once)
        self.J_q2p = q_motor2pose_ee()

        # Virtual springs [N/m] and dampers [N·s/m]
        self.spring = spring
        self.damper = damper
 
    def single_torques(self, q_motor, q_motor_dot, target_pos=None):
        """
        Args:
            q_motor:     [q1, q2, q3] motor angles [rad]
            q_motor_dot: [q1_dot, q2_dot, q3_dot] motor velocities [rad/s]
            target_pos:  [x, y, z] target position [m]
        Returns:
            tau: [tau_1, tau_2, tau_3] motor torques [N·m]
        """
        tau = np.zeros(3)
       
        J = np.array(self.J_q2p(q_motor[0], q_motor[1], q_motor[2]))   # (6, 3)
        pose = FK_Multi_ee(q_motor) # [x, y, z, yaw, pitch, roll] [m, rad]
        pose_dot = J @ q_motor_dot 

        # we focus only on the position part of the pose for the VMC, ignoring orientation for simplicity
        pos = pose[:3]
        vel = pose_dot[:3]
        J_xyz = J[:3, :]
        tau = J_xyz.T @ (self.spring.compute_force(pos, target_pos) + self.damper.compute_force(vel))
        return tau
    
class VMC_Multi:
    """
    Task-space VMC for the multi-segment robot.
    A virtual spring attached between ANY point along the segement (point parametrized by s_param) and target position.
    A virtual damper on the velocity of the point defined by s_param.
    Forces are mapped to motor torques via Jacobian transpose:  tau = J^T @ F_virtual
    """
    def __init__(self, spring, damper, s_param=1.0): 
        # Jacobians (built once)
        self.J_q2p = q_motor2pose(s_param)

        # Virtual springs [N/m] and dampers [N·s/m]
        self.spring = spring
        self.damper = damper
        self.s_param = s_param
 
    def single_torques(self, q_motor, q_motor_dot, target_pos=None):
        """
        Args:
            q_motor:     [q1, q2, q3] motor angles [rad]
            q_motor_dot: [q1_dot, q2_dot, q3_dot] motor velocities [rad/s]
            target_pos:  [x, y, z] target position [m]
        Returns:
            tau: [tau_1, tau_2, tau_3] motor torques [N·m]
        """
        tau = np.zeros(3)
       
        J = np.array(self.J_q2p(q_motor[0], q_motor[1], q_motor[2]))   # (6, 3)
        pose = FK_Multi(q_motor,self.s_param) # [x, y, z, yaw, pitch, roll] [m, rad]
        pose_dot = J @ q_motor_dot 

        # we focus only on the position part of the pose for the VMC, ignoring orientation for simplicity
        pos = pose[:3]
        vel = pose_dot[:3]
        J_xyz = J[:3, :]
        tau = J_xyz.T @ (self.spring.compute_force(pos, target_pos) + self.damper.compute_force(vel))
        return tau

# Example usage
# if __name__ == "__main__":
#     vmc = VMC_Single_EE(gauss_spring, lin_damper) # import from VMC_utils
#     q_motor = np.array([0.0, 0.0, 0.0])  # motor angles [rad]
#     q_motor_dot = np.array([0.0, 0.0, 0.0])  # motor velocities [rad/s]
#     target_pos = np.array([0.0, 0.0, 0.145])  # target position [m]
#     tau = vmc.single_torques(q_motor, q_motor_dot, target_pos)
#     print("Motor torques:", tau)