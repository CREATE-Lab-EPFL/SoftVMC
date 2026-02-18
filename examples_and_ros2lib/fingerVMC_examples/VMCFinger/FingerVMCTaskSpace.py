import numpy as np
from VMC_utils.VirtualModels import LinearSpring, LinearDamper
from KinematicsFinger.FK_Finger import FK
from KinematicsFinger.JacobiansFinger import q2MCP, q2DIP

# Finger link lengths [m]
A = 0.040   # proximal phalanx
B = 0.030   # middle phalanx
C = 0.0175  # distal phalanx


class VMC:
    """
    Task-space VMC for the finger.

    Virtual springs and dampers are attached at two Cartesian points (MCP base, fingertip).
    Forces are mapped to motor torques via Jacobian transpose:  tau = J^T @ F_virtual
    """

    def __init__(self):
        # Jacobians (built once)
        self.J_MCP = q2MCP()
        self.J_DIP = q2DIP()

        # Attachment points in local link frames [m]
        self.r_MCP = np.array([0.0, A,   0.0])   # proximal end of proximal phalanx
        self.r_DIP = np.array([0.0, C,   0.0])   # fingertip

        # Virtual springs [N/m] and dampers [N·s/m]
        self.spring_MCP = LinearSpring(40.0)
        self.spring_DIP = LinearSpring(40.0)
        self.damper_MCP = LinearDamper(1.0)
        self.damper_DIP = LinearDamper(1.0)

        # Target Cartesian positions [m]  — tune these
        self.target_MCP = np.array([0.0, A,    0.0])
        self.target_DIP = np.array([0.0, 0.05, 0.05])

    def _jacobian(self, q_motor, link):
        r  = self.r_MCP if link == 'MCP' else self.r_DIP
        fn = self.J_MCP if link == 'MCP' else self.J_DIP
        return np.array(fn(q_motor[0], q_motor[1], r[0], r[1], r[2]))   # (3, 2)

    def finger_torques(self, q_motor, q_dot_motor):
        """
        Args:
            q_motor:     [q_MCP, q_PIP] motor angles [rad]
            q_dot_motor: [q_dot_MCP, q_dot_PIP] motor velocities [rad/s]
        Returns:
            tau: [tau_MCP, tau_PIP] motor torques [N·m]
        """
        tau = np.zeros(2)
        for link, spring, damper, target in [
            ('MCP', self.spring_MCP, self.damper_MCP, self.target_MCP),
            ('DIP', self.spring_DIP, self.damper_DIP, self.target_DIP),
        ]:
            r   = self.r_MCP if link == 'MCP' else self.r_DIP
            J   = self._jacobian(q_motor, link)
            pos = FK(q_motor, link, r)
            vel = J @ q_dot_motor
            tau += J.T @ (spring.compute_force(pos, target) + damper.compute_force(vel))
        return tau
