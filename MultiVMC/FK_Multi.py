import numpy as np
from JacobiansMulti import MOTOR_RADIUS, SECTION_RADIUS, REST_LENGTH, A_INV1, A_INV2, A_INV3

# Helper functions for FK 
def R_MATRIX(Dx, Dy, D, d=SECTION_RADIUS, s_param=1.0):
        theta_s = s_param*(D/d)
        return np.array([
            [1 + Dx**2/D**2*(np.cos(theta_s)-1),  Dx*Dy/D**2*(np.cos(theta_s)-1),  Dx/D*np.sin(theta_s)],
            [Dx*Dy/D**2*(np.cos(theta_s)-1),  1 + Dy**2/D**2*(np.cos(theta_s)-1),  Dy/D*np.sin(theta_s)],
            [-Dx/D*np.sin(theta_s),               -Dy/D*np.sin(theta_s),            np.cos(theta_s)]
        ])

def t_vector(Dx, Dy, D, Dl, L0, d=SECTION_RADIUS, s_param=1.0):
    theta_s = s_param*(D/d)
    return np.array([
        [(s_param*d*(L0+Dl)/D**2)*Dx*(1-np.cos(theta_s))],
        [(s_param*d*(L0+Dl)/D**2)*Dy*(1-np.cos(theta_s))],
        [(s_param*d*(L0+Dl)/D**2)*D*np.sin(theta_s)]
    ])

# def FK_tendon2q(DELTAL, delta=np.array([0, 2*np.pi/3, -2*np.pi/3])):
def FK_q_motor2pcc(q_motor: np.ndarray) -> np.ndarray:
    """
    FK from q_motor = DELTAL / MOTOR_RADIUS to lateral curvatures+elongation (pcc)
    Args:
        q_motor: [q11 q12 q13 q21 q22 q23 q31 q32 q33] [rad]
    Returns:
        pcc: [Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3] [m]
    """
    DELTAL = q_motor * MOTOR_RADIUS  # tendon elongations
    pcc1 = A_INV1 @ DELTAL[0:3]
    pcc2 = A_INV2 @ DELTAL[3:6]
    pcc3 = A_INV3 @ DELTAL[6:9]
    return pcc1, pcc2, pcc3

def FK_pcc2pose_ee(pcc1, pcc2, pcc3):
    """
    FK from lateral cuvatures+elongation (pcc) to pose.
    Args: pcc = [Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3] [m]
    Returns: pose = [x, y, z, yaw, pitch, roll] [m, rad]
    """
    Dx1, Dy1, Dl1 = pcc1
    Dx2, Dy2, Dl2 = pcc2
    Dx3, Dy3, Dl3 = pcc3
    L0_1 = L0_2 = L0_3 = REST_LENGTH   # rest length [m]

    D1 = np.sqrt(Dx1**2 + Dy1**2 + 1e-12)
    D2 = np.sqrt(Dx2**2 + Dy2**2 + 1e-12)
    D3 = np.sqrt(Dx3**2 + Dy3**2 + 1e-12)

    R1 = R_MATRIX(Dx1, Dy1, D1)
    R2 = R_MATRIX(Dx2, Dy2, D2)
    R3 = R_MATRIX(Dx3, Dy3, D3)
    R  = R1 @ R2 @ R3

    t1 = t_vector(Dx1, Dy1, D1, Dl1, L0_1)
    t2 = t_vector(Dx2, Dy2, D2, Dl2, L0_2)
    t3 = t_vector(Dx3, Dy3, D3, Dl3, L0_3)
    t  = t1 + R1@t2 + R1@R2@t3

    # ZYX Euler angles from R
    yaw   = np.arctan2(R[1,0], R[0,0] + 1e-12)
    pitch = np.arcsin(-R[2,0])
    roll  = np.arctan2(R[2,1], R[2,2] + 1e-12)

    pose = np.vstack((t, np.array([[yaw], [pitch], [roll]])))

    return pose.flatten()

# def FK_pcc2pose(pcc, s_param):
#     """
#     FK from lateral cuvatures+elongation (pcc) to pose.
#     Args: pcc = [Dx, Dy, Dl] [m]
#           s_param: s = [0,1] defines position along the segment (0 = base, 1 = tip).
#     Returns: pose = [x, y, z, yaw, pitch, roll] [m, rad]
#     """
#     Dx, Dy, Dl = pcc
#     d  = SECTION_RADIUS # section radius [m]  (diameter = 60 mm)
#     L0 = REST_LENGTH  # rest length [m]

#     D = np.sqrt(Dx**2 + Dy**2 + 1e-12)  # curvature magnitude (regularised)

#     # PCC rotation matrix (Rodrigues on curvature axis)
#     theta_s = s_param*(D/d)
#     R = np.array([
#         [1 + Dx**2/D**2*(np.cos(theta_s)-1),  Dx*Dy/D**2*(np.cos(theta_s)-1),  Dx/D*np.sin(theta_s)],
#         [Dx*Dy/D**2*(np.cos(theta_s)-1),  1 + Dy**2/D**2*(np.cos(theta_s)-1),  Dy/D*np.sin(theta_s)],
#         [-Dx/D*np.sin(theta_s),               -Dy/D*np.sin(theta_s),            np.cos(theta_s)]
#     ])

#     # PCC tip translation
#     t = np.array([
#         [(s_param*d*(L0+Dl)/D**2)*Dx*(1-np.cos(theta_s))],
#         [(s_param*d*(L0+Dl)/D**2)*Dy*(1-np.cos(theta_s))],
#         [(s_param*d*(L0+Dl)/D**2)*D*np.sin(theta_s)]
#     ])

#     # ZYX Euler angles from R
#     yaw   = np.arctan2(R[1,0], R[0,0] + 1e-12)
#     pitch = np.arcsin(-R[2,0])
#     roll  = np.arctan2(R[2,1], R[2,2] + 1e-12)

#     pose = np.vstack((t, np.array([[yaw], [pitch], [roll]])))

#     return pose.flatten()

def FK_Single_ee(q_motor):
    """
    Forward kinematics for the single VMC.
    Args: q_motor: [q1, q2, q3] motor angles [rad]
    Returns: pose: [x, y, z, yaw, pitch, roll] of the end-effector [m, rad]
    """
    pcc1, pcc2, pcc3 = FK_q_motor2pcc(q_motor)
    pose = FK_pcc2pose_ee((pcc1, pcc2, pcc3))

    return pose

# def FK_Single(q_motor, s_param):
#     """
#     Forward kinematics for the single VMC.
#     Args: q_motor: [q1, q2, q3] motor angles [rad]
#           s_param: s = [0,1] defines position along the segment (0 = base, 1 = tip).
#     Returns: pose: [x, y, z, yaw, pitch, roll] of the end-effector [m, rad]
#     """
#     pcc = FK_q_motor2pcc(q_motor)
#     pose = FK_pcc2pose(pcc, s_param)

#     return pose


# Example usage
if __name__ == "__main__":
    # Motor config: (0 , 2pi/3, -2pi/3) -> At rest (q1, q2, q3) = (0, 0, 0) -> pose = (0, 0, 145, 0, 0, 0)

    # Motor values [rad]
    q_motor = np.deg2rad(np.array([10, 10, 10]))  
    # s_param = 1.0  # tip of the segment

    # pose_ee = FK_Single_ee(q_motor)
    # pose = FK_Single(q_motor, s_param)

    # print(f"End-effector pose (using FK_Single_ee): {pose_ee} [m, rad]")
    # print(f"Input motor angles (q_motor): {q_motor} [rad]")
    # print(f"Pose: {pose} [m, rad] for s_param={s_param}")