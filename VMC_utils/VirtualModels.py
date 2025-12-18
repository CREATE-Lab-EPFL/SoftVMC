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
    
    def compute_force(self, velocity_current, velocity_target):
        """
        Args:
            velocity_current: [vx, vy, vz] current velocity (mm/s)
            velocity_target: [vx, vy, vz] target velocity (mm/s)
        
        Returns:
            F: [Fx, Fy, Fz] damping force (N)
        """
        relative_velocity = velocity_current - velocity_target
        return -self.damping * relative_velocity


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

    def compute_force(self, velocity_current, velocity_target):
        """
        Args:
            velocity_current: [vx, vy, vz] current velocity (m/s)
            velocity_target: [vx, vy, vz] target velocity (m/s)

        Returns:
            F: [Fx, Fy, Fz] damping force (N)
        """
        relative_velocity = velocity_current - velocity_target
        return -self.max_force * np.tanh(self.damping * relative_velocity / self.max_force)

# ============================================================================
# EXAMPLE USAGE
# ============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("VIRTUAL MECHANISM COMPONENTS TEST")
    print("=" * 60)
    
    # Create spring objects with parameters
    print("\n--- SPRINGS ---")
    spring_linear = LinearSpring(stiffness=2000.0)  # N/m (was 2.0 N/mm)
    spring_tanh = TanhSpring(stiffness=2000.0, max_force=5.0)  # N/m (was 2.0 N/mm)
    spring_gaussian = GaussianSpring(strength=10.0, sigma=0.005)  # m (was 5.0 mm)
    
    pos_current = np.array([0.010, 0.020, 0.005])  # m (was 10.0, 20.0, 5.0 mm)
    pos_target = np.array([0.015, 0.025, 0.010])  # m (was 15.0, 25.0, 10.0 mm)
    pos_obstacle = np.array([0.012, 0.022, 0.006])  # m (was 12.0, 22.0, 6.0 mm)
    
    F1 = spring_linear.compute_force(pos_current, pos_target)
    F2 = spring_tanh.compute_force(pos_current, pos_target)
    F3 = spring_gaussian.compute_force(pos_current, pos_obstacle)
    
    print(f"Linear spring:      {F1} N")
    print(f"Tanh spring:        {F2} N")
    print(f"Gaussian repulsive: {F3} N")
    
    # Create damper objects
    print("\n--- DAMPERS ---")
    damper_linear = LinearDamper(damping=500.0)  # N·s/m (was 0.5 N·s/mm)
    damper_tanh = TanhDamper(damping=500.0, max_force=3.0)  # N·s/m (was 0.5 N·s/mm)
    
    velocity_current = np.array([0.005, -0.003, 0.002])  # m/s (was 5.0, -3.0, 2.0 mm/s)
    velocity_target = np.array([0.0, 0.0, 0.0])  # m/s
    
    F4 = damper_linear.compute_force(velocity_current, velocity_target)
    F5 = damper_tanh.compute_force(velocity_current, velocity_target)
    
    print(f"Linear damper:      {F4} N")
    print(f"Tanh damper:        {F5} N")
    
    # Test deadzone limit spring for joint limits
    print("\n--- JOINT LIMIT SPRINGS ---")
    deadzone_MCP = DeadzoneLimitSpring(stiffness=0.5, angle_lower=-1.57, angle_upper=0.0)
    deadzone_PIP = DeadzoneLimitSpring(stiffness=0.5, angle_lower=-1.22, angle_upper=0.0)
    
    # Test cases
    angles_test = [-1.8, -1.0, 0.0, 0.2]  # Below, within, at limit, above
    print("Angle (rad) | MCP Force (N·m) | PIP Force (N·m)")
    print("-" * 50)
    for angle in angles_test:
        tau_mcp = deadzone_MCP.compute_force(angle)
        tau_pip = deadzone_PIP.compute_force(angle)
        print(f"{angle:11.2f} | {tau_mcp:15.3f} | {tau_pip:15.3f}")
    
    print("\n" + "=" * 60)
    print("Reusing components with different inputs:")
    print("=" * 60)
    
    # Reuse same spring with different positions
    pos_new = np.array([0.012, 0.023, 0.007])  # m (was 12.0, 23.0, 7.0 mm)
    F_reused = spring_linear.compute_force(pos_new, pos_target)
    print(f"\nLinear spring with new position: {F_reused} N")