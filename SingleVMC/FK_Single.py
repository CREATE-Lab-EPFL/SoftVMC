import numpy as np

# Units: LENGTHS [m], ANGLES [rad]
MOTOR_RADIUS = 0.003 # [m] radius of the tendon spool
SECTION_RADIUS = 0.030  # [m] radius of the section (diameter = 60 mm)
REST_LENGTH = 0.145     # [m] rest length of the section


# def FK_tendon2q(DELTAL, delta=np.array([0, 2*np.pi/3, -2*np.pi/3])):
def FK_q_motor2pcc(q_motor, delta):
    """
    FK from q_motor = DELTAL / MOTOR_RADIUS to lateral cuvatures+elongation (q)
    Args: q_motor = [q1 q2 q3] [rad]
    Returns: pcc = [Dx, Dy, Dl] [m]
    """
    DELTAL = q_motor * MOTOR_RADIUS  # convert from motor space to tendon elongation space
    A = np.array([[-np.cos(di), -np.sin(di), 1.0] for di in delta])  # (3, 3)
    pcc = np.linalg.inv(A) @ DELTAL

    return pcc

def FK_pcc2pose(pcc):
    """
    FK from lateral cuvatures+elongation (pcc) to pose.
    Args: pcc = [Dx, Dy, Dl] [m]
    Returns: pose = [x, y, z, yaw, pitch, roll] [m, rad]
    """
    Dx, Dy, Dl = pcc
    d  = SECTION_RADIUS # section radius [m]  (diameter = 60 mm)
    L0 = REST_LENGTH  # rest length [m]

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

def FK_Single(q_motor, delta=np.array([0, 2*np.pi/3, -2*np.pi/3])):
    """
    Forward kinematics for the single VMC.
    Args: q_motor: [q1, q2, q3] motor angles [rad]
    Returns: pose: [x, y, z, yaw, pitch, roll] of the end-effector [m, rad]
    """
    pcc = FK_q_motor2pcc(q_motor, delta)
    pose = FK_pcc2pose(pcc)
    return pose


# Example usage
if __name__ == "__main__":
    # Motor config: (0 , 2pi/3, -2pi/3) -> At rest (q1, q2, q3) = (0, 0, 0) -> pose = (0, 0, 145, 0, 0, 0)

    # Motor values [rad]
    q_motor = np.deg2rad(np.array([10, 10, 10]))  

    pose = FK_Single(q_motor)
    print(f"Input motor angles (q_motor): {q_motor} [rad]")
    print(f"End-effector pose: {pose} [m, rad]")