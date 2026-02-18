import numpy as np
from VMC_utils.VirtualModels import DeadzoneLimitSpring
from VMC_utils.GravityCompensation import GravityCompensation
from KinematicsFinger.FK_Finger import motor_to_joint
from KinematicsFinger.JacobiansFinger import q2joint, q2MCP, q2PIP, q2DIP


class GravFricLim:
    """
    Gravity + Stribeck friction + joint-limit compensation for the finger.

    Call compute_compensation_torques() and add the result to the VMC torques.
    """

    def __init__(self):
        # Jacobians (built once)
        self.J_motors = q2joint()
        self.J_MCP    = q2MCP()
        self.J_PIP    = q2PIP()
        self.J_DIP    = q2DIP()

        # Rotation from world to finger frame (finger points in -y, -z world directions)
        R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

        # Link masses [kg] and centres of gravity in local frame [m]
        self.gravity_comp = {
            'MCP': GravityCompensation(mass=0.0057, R_world_to_system=R),
            'PIP': GravityCompensation(mass=0.0040, R_world_to_system=R),
            'DIP': GravityCompensation(mass=0.0250, R_world_to_system=R),
        }
        self.cog = {
            'MCP': np.array([-0.000639,  0.0200,  0.001276]),
            'PIP': np.array([ 0.000925,  0.0150,  0.001138]),
            'DIP': np.array([-0.000716,  0.0105,  0.000955]),
        }

        # Stribeck friction model: tau_f = F_max * exp(-(v/v_lim)^2) * sign(tau)
        self.F_max = 0.20   # [N·m] static friction amplitude
        self.v_lim = 0.03   # [rad/s] Stribeck velocity

        # Joint limits [rad] and limit-spring stiffness [N·m/rad]
        self.limit_springs = {
            'MCP': DeadzoneLimitSpring(1.0, 0.0, 1.5708),
            'PIP': DeadzoneLimitSpring(1.0, 0.0, 1.2217),
            'DIP': DeadzoneLimitSpring(1.0, 0.0, 1.2217),
        }

    def _jac(self, q_motor, link):
        r  = self.cog[link]
        fn = {'MCP': self.J_MCP, 'PIP': self.J_PIP, 'DIP': self.J_DIP}[link]
        return np.array(fn(q_motor[0], q_motor[1], r[0], r[1], r[2]))   # (3, 2)

    def gravity_compensation(self, q_motor):
        """Gravity compensation torques [N·m] for all three links."""
        tau = np.zeros(2)
        for link in ('MCP', 'PIP', 'DIP'):
            tau += self._jac(q_motor, link).T @ self.gravity_comp[link].compute_force()
        return tau

    def deadzone_spring_torques(self, q_motor):
        """Joint-limit repulsion torques [N·m]."""
        theta = motor_to_joint(q_motor)
        tau_joint = np.array([
            self.limit_springs[j].compute_force(theta[i])
            for i, j in enumerate(('MCP', 'PIP', 'DIP'))
        ])
        return np.array(self.J_motors(*q_motor)).T @ tau_joint

    def friction_compensation(self, q_dot_motor, tau_commanded):
        """Stribeck friction compensation torques [N·m]."""
        return self.F_max * np.exp(-(q_dot_motor / self.v_lim) ** 2) * np.sign(tau_commanded)

    def compute_compensation_torques(self, q_motor, q_dot_motor, tau_vmc):
        """
        Full compensation: gravity + joint limits + friction.

        Args:
            q_motor:     [q_MCP, q_PIP] motor angles [rad]
            q_dot_motor: [q_dot_MCP, q_dot_PIP] motor velocities [rad/s]
            tau_vmc:     [tau_MCP, tau_PIP] VMC torques [N·m]
        Returns:
            tau_comp: [tau_MCP, tau_PIP] compensation torques [N·m]
        """
        tau_grav  = self.gravity_compensation(q_motor)
        tau_lim   = self.deadzone_spring_torques(q_motor)
        tau_fric  = self.friction_compensation(q_dot_motor, tau_grav + tau_lim + tau_vmc)
        return tau_grav + tau_lim + tau_fric
