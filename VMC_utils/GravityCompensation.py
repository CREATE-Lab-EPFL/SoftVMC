import numpy as np

# Units of measure:
# - POSITIONS: [mm]
# - MASSES: [kg]
# - FORCES: [N]
# - GRAVITY: [9.81 m/s²]


# ─────────────────────────────────────────────────────────────────────────────
# Gravity direction helpers
#
# Gravity is always -z in the world frame. When the arm is mounted rotated,
# the same gravity vector points somewhere else in the *robot* frame -- e.g.
# -y for the arm laid on its side. These helpers turn that observed direction
# into the R_world_to_system the compensator expects, so configs and CLI flags
# can just say "-y" instead of hand-writing a rotation matrix.
# ─────────────────────────────────────────────────────────────────────────────

GRAVITY_DIR_CHOICES = ("+x", "-x", "+y", "-y", "+z", "-z")

_AXIS_VECTORS = {
    "+x": (1.0, 0.0, 0.0), "-x": (-1.0, 0.0, 0.0),
    "+y": (0.0, 1.0, 0.0), "-y": (0.0, -1.0, 0.0),
    "+z": (0.0, 0.0, 1.0), "-z": (0.0, 0.0, -1.0),
}


def gravity_dir_vector(direction):
    """
    Unit vector of the gravity direction *in the system (robot) frame*.

    Args:
        direction: axis label ("-z", "-y", "x", ...) or any 3-vector

    Returns:
        (3,) unit vector
    """
    if isinstance(direction, str):
        key = direction.strip().lower()
        if key and key[0] not in "+-":
            key = "+" + key
        if key not in _AXIS_VECTORS:
            raise ValueError(
                f"unknown gravity direction {direction!r}; "
                f"expected one of {GRAVITY_DIR_CHOICES} or a 3-vector"
            )
        return np.array(_AXIS_VECTORS[key], dtype=float)

    v = np.asarray(direction, dtype=float).reshape(-1)
    if v.size != 3:
        raise ValueError(f"gravity direction must have 3 components, got {v.size}")
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("gravity direction must be a non-zero vector")
    return v / n


def rotation_for_gravity_dir(direction="-z"):
    """
    Rotation matrix R_world_to_system for a given system-frame gravity direction.

    Returns the R such that R @ [0, 0, -1] == unit(direction), i.e. world gravity
    (-z) lands on the observed direction. Built as the minimal rotation from +z
    onto -unit(direction) (Rodrigues), so "-z" gives the identity.

    Args:
        direction: axis label ("-z", "-y", ...) or any 3-vector, in the system frame

    Returns:
        (3,3) rotation matrix
    """
    a = np.array([0.0, 0.0, 1.0])       # world "up"
    b = -gravity_dir_vector(direction)  # where "up" must end up in the system frame

    v = np.cross(a, b)
    s = float(np.linalg.norm(v))
    c = float(np.dot(a, b))

    if s < 1e-12:
        # Parallel: identity. Anti-parallel: 180° about x.
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])

    vx = np.array([[0.0, -v[2],  v[1]],
                   [v[2],  0.0, -v[0]],
                   [-v[1], v[0],  0.0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / s ** 2)


class GravityCompensation:
    """
    Gravity compensation with arbitrary system orientation.
    
    Gravity is always -z in world frame.
    System may be oriented differently: provide transformation matrix.
    """
    
    def __init__(self, mass, R_world_to_system=np.eye(3), gravity=9.81):
        """
        Args:
            mass: M total body mass (kg)
            gravity: g gravitational acceleration (m/s²)
        """
        self.mass = mass
        self.gravity = gravity
        self.world_gravity = np.array([0.0, 0.0, -self.mass * self.gravity])  # -z direction
        self.R_world_to_system = R_world_to_system
    
    def compute_force(self):
        """
        Compute gravity compensation force in system frame.
        
        Args:
            R_world_to_system: 3x3 rotation matrix from world to system frame
                              Default is identity (system = world frame)
        
        Returns:
            F: [Fx, Fy, Fz] compensation force in system frame (N)
        """
        # Transform world gravity to system frame, then negate for compensation
        gravity_in_system = self.R_world_to_system @ self.world_gravity
        return -gravity_in_system


if __name__ == "__main__":

    # Finger frame: finger z-axis = world -z-axis (180° rotation around x)
    R = np.array([[1, 0, 0],
                  [0, -1, 0],
                  [0, 0, -1]])

    # Test
    gc = GravityCompensation(mass=0.5, R_world_to_system=R)
    
    # Default: world frame (identity matrix)
    F_finger = gc.compute_force()
    print(f"Finger frame: {F_finger} N")

    # Arm rotated onto its side: gravity pulls along -y in the robot frame,
    # so the compensation force must point +y.
    for d in ("-z", "-y", "+y", "+x"):
        gc_d = GravityCompensation(mass=0.5, R_world_to_system=rotation_for_gravity_dir(d))
        print(f"gravity {d:>2}: compensation {np.round(gc_d.compute_force(), 4)} N")
