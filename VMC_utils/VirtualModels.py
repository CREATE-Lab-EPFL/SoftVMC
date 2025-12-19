import numpy as np

# Units of measure:
# - POSITIONS: [m]
# - FORCES: [N]
# - VELOCITIES: [m/s]
# - DAMPING: [N·s/m]
# - STIFFNESS: [N/m]

# ============================================================================
# SPRINGS (position-based forces)
# ============================================================================

class LinearSpring:
    """Linear spring force: F = K * (target - current)"""
    
    def __init__(self, stiffness):
        """
        Args:
            stiffness: K spring stiffness (N/m)
        """
        self.stiffness = stiffness
    
    def compute_force(self, pos_current, pos_target):
        """
        Args:
            pos_current: [x, y, z] current position (m)
            pos_target: [x, y, z] target position (m)
        
        Returns:
            F: [Fx, Fy, Fz] force vector (N)
        """
        error = pos_target - pos_current
        return self.stiffness * error


class TanhSpring:
    """Tanh spring with component-wise saturation"""
    
    def __init__(self, stiffness, max_force):
        """
        Args:
            stiffness: K spring stiffness at origin (N/m)
            max_force: F_max maximum force magnitude per component (N)
        """
        self.stiffness = stiffness
        self.max_force = max_force
    
    def compute_force(self, pos_current, pos_target):
        """
        Args:
            pos_current: [x, y, z] current position (m)
            pos_target: [x, y, z] target position (m)
        
        Returns:
            F: [Fx, Fy, Fz] force vector (N)
        """
        error = pos_target - pos_current
        return self.max_force * np.tanh(self.stiffness * error / self.max_force)


class GaussianSpring:
    """Gaussian repulsive spring for obstacle avoidance"""
    
    def __init__(self, strength, sigma):
        """
        Args:
            strength: A force amplitude (N)
            sigma: spread of Gaussian (m)
        """
        self.strength = strength
        self.sigma = sigma
    
    def compute_force(self, pos_current, pos_obstacle):
        """
        Args:
            pos_current: [x, y, z] current position (m)
            pos_obstacle: [x, y, z] obstacle position (m)
        
        Returns:
            F: [Fx, Fy, Fz] repulsive force vector (N)
        """
        delta = pos_current - pos_obstacle
        distance_sq = np.dot(delta, delta)
        decay = np.exp(-distance_sq / (2 * self.sigma**2))
        return self.strength * decay * delta


class DeadzoneLimitSpring:
    """Deadzone spring for joint limits (ReLU-like behavior)"""
    
    def __init__(self, stiffness, angle_lower, angle_upper):
        """
        Args:
            stiffness: K spring stiffness (N·m/rad)
            angle_lower: lower joint limit (rad)
            angle_upper: upper joint limit (rad)
        """
        self.stiffness = stiffness
        self.angle_lower = angle_lower
        self.angle_upper = angle_upper
    
    def compute_force(self, angle_current):
        """
        Args:
            angle_current: current joint angle (rad)
        
        Returns:
            F: scalar torque pushing back toward valid range (N·m)
        """
        if angle_current < self.angle_lower:
            return self.stiffness * (self.angle_lower - angle_current)
        elif angle_current > self.angle_upper:
            return self.stiffness * (self.angle_upper - angle_current)
        else:
            return 0.0


# ============================================================================
# DAMPERS (velocity-based forces)
# ============================================================================

class LinearDamper:
    """Linear damper force: F = -D * (v_current - v_target)"""
    
    def __init__(self, damping):
        """
        Args:
            damping: D damping coefficient (N·s/m)
        """
        self.damping = damping
    
    def compute_force(self, velocity_current):
        """
        Args:
            velocity_current: [vx, vy, vz] current velocity (mm/s)
        
        Returns:
            F: [Fx, Fy, Fz] damping force (N)
        """
        return -self.damping * velocity_current


class TanhDamper:
    """Tanh damper with component-wise saturation"""

    def __init__(self, damping, max_force):
        """
        Args:
            damping: D damping coefficient (N·s/m)
            max_force: F_max maximum damping force per component (N)
        """
        self.damping = damping
        self.max_force = max_force

    def compute_force(self, velocity_current):
        """
        Args:
            velocity_current: [vx, vy, vz] current velocity (m/s)

        Returns:
            F: [Fx, Fy, Fz] damping force (N)
        """
        return -self.max_force * np.tanh(self.damping * velocity_current / self.max_force)