import numpy as np

# Units of measure:
# - LENGTHS: [m]
# - ANGLES: [rad]

def FK_tendon2q(DELTAL):
    """
    FK from tendons (DELTAL) to lateral cuvatures+elongation (q)
    Args: DELTAL = [D1 D2 D3] [mm]
    Returns: q = [Dx, Dy, Dl] [mm]
    """
    delta = np.array([0, 2*np.pi/3, -2*np.pi/3])
    A = np.array([[-np.cos(di), -np.sin(di), 1.0] for di in delta])  # (3, 3)
    q = np.linalg.inv(A) @ DELTAL

    return q

def FK_q2pose(q):
    """
    FK from lateral cuvatures+elongation (q) to pose.
    Args: q = [Dx, Dy, Dl] [mm]
    Returns: pose = [x, y, z, yaw, pitch, roll] [mm, rad]
    """
    Dx, Dy, Dl = q
    d  = 30    # section radius [mm]  (diameter = 60 mm)
    L0 = 145   # rest length [mm]

    D = np.sqrt(Dx**2 + Dy**2 + 1e-12)  # curvature magnitude (regularised)

    # PCC rotation matrix (Rodrigues on curvature axis)
    R = np.array([
        [1 + Dx**2/D**2*(np.cos(D/d)-1),  Dx*Dy/D**2*(np.cos(D/d)-1),  Dx/D*np.sin(D/d)],
        [Dx*Dy/D**2*(np.cos(D/d)-1),  1 + Dy**2/D**2*(np.cos(D/d)-1),  Dy/D*np.sin(D/d)],
        [-Dx/D*np.sin(D/d),               -Dy/D*np.sin(D/d),            np.cos(D/d)]
    ])

    # PCC tip translation
    t = np.array([
        [(d*(L0+Dl)/D**2)*Dx*(1-np.cos(D/d))],
        [(d*(L0+Dl)/D**2)*Dy*(1-np.cos(D/d))],
        [(d*(L0+Dl)/D**2)*D*np.sin(D/d)]
    ])

    # ZYX Euler angles from R
    yaw   = np.arctan2(R[1,0], R[0,0] + 1e-12)
    pitch = np.arcsin(-R[2,0])
    roll  = np.arctan2(R[2,1], R[2,2] + 1e-12)

    pose = np.vstack((t, np.array([[yaw], [pitch], [roll]])))

    return pose.flatten()

def FK_Single(DELTAL):
    """
    Forward kinematics for the single VMC.

    Args:
        DELTAL: [D1, D2, D3] tendon elongations [mm]

    Returns:
        pose: [x, y, z, yaw, pitch, roll] of the end-effector [mm, rad]
    """
    q = FK_tendon2q(DELTAL)
    pose = FK_q2pose(q)
    return pose


# Example usage
if __name__ == "__main__":
    # Motor config: (0 , 2pi/3, -2pi/3) -> At rest (D1, D2, D3) = (0, 0, 0) -> pose = (0, 0, 145, 0, 0, 0)

    # Motor values (tendon elongations in mm)
    DELTAL = np.array([10.0, 5.0, 5.0])  

    pose = FK_Single(DELTAL)
    print(f"Input tendon elongations (DELTAL): {DELTAL} [mm]")
    print(f"End-effector pose: {pose} [mm, rad]")