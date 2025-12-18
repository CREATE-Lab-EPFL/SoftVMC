import numpy as np

# Units of measure:
# - POSITIONS: [mm]
# - MASSES: [kg]
# - FORCES: [N]
# - GRAVITY: [9.81 m/s²]


class GravityCompensation:
    """
    Gravity compensation with arbitrary system orientation.
    
    Gravity is always -z in world frame.
    System may be oriented differently: provide transformation matrix.
    """
    
    def __init__(self, mass, gravity=9.81):
        """
        Args:
            mass: M total body mass (kg)
            gravity: g gravitational acceleration (m/s²)
        """
        self.mass = mass
        self.gravity = gravity
        self.world_gravity = np.array([0.0, 0.0, -self.mass * self.gravity])  # -z direction
    
    def compute_force(self, R_world_to_system=None):
        """
        Compute gravity compensation force in system frame.
        
        Args:
            R_world_to_system: 3x3 rotation matrix from world to system frame
                              If None, assumes system = world frame
        
        Returns:
            F: [Fx, Fy, Fz] compensation force in system frame (N)
        """
        if R_world_to_system is None:
            # No transformation: gravity compensation is +z in world frame
            return np.array([0.0, 0.0, self.mass * self.gravity])
        else:
            # Transform world gravity to system frame, then negate for compensation
            gravity_in_system = R_world_to_system @ self.world_gravity
            return -gravity_in_system


if __name__ == "__main__":
    # Test
    gc = GravityCompensation(mass=0.5)
    
    # Default: world frame (no transformation)
    F_world = gc.compute_force()
    print(f"World frame: {F_world} N")
    
    # Finger frame: finger z-axis = world -z-axis
    R = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, -1]])
    F_system = gc.compute_force(R)
    print(f"Finger system: {F_system} N")