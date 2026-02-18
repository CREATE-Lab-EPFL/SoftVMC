import numpy as np
from VMC_utils.VirtualModels import LinearSpring, LinearDamper
from KinematicsFinger.FK_Finger import motor_to_joint
from KinematicsFinger.JacobiansFinger import q2joint


class VMC:
    """
    Finger-space VMC for the finger.

    Virtual springs and dampers act directly on joint angles [theta_MCP, theta_PIP, theta_DIP].
    Torques are mapped to motors via Jacobian transpose:  tau_motor = J^T @ tau_joint
    Simpler than task-space VMC — no FK needed, works well for posture control.
    """

    def __init__(self):
        # Jacobian motor -> joint angles (built once)
        self.J_joint = q2joint()

        # Virtual spring [N·m/rad] and damper [N·m·s/rad]  — tune these
        self.spring = LinearSpring(0.2)
        self.damper = LinearDamper(0.03)

        # Target joint angles [rad]  — tune this
        self.target = np.deg2rad(np.array([30.0, 30.0, 30.0]))   # [MCP, PIP, DIP]

    def finger_torques(self, q_motor, q_dot_motor):
        """
        Args:
            q_motor:     [q_MCP, q_PIP] motor angles [rad]
            q_dot_motor: [q_dot_MCP, q_dot_PIP] motor velocities [rad/s]
        Returns:
            tau: [tau_MCP, tau_PIP] motor torques [N·m]
        """
        J         = np.array(self.J_joint(q_motor[0], q_motor[1]))   # (3, 2)
        theta     = motor_to_joint(q_motor)                            # current joint angles (3,)
        theta_dot = J @ q_dot_motor                                    # joint velocities (3,)

        tau_joint = self.spring.compute_force(theta, self.target) \
                  + self.damper.compute_force(theta_dot)               # virtual joint torques (3,)
        return J.T @ tau_joint                                         # motor torques (2,)
