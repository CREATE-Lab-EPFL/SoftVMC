import numpy as np
from sympy import symbols, sqrt, atan2, cos, sin, Matrix, lambdify, asin, pi# type: ignore

# Units: LENGTHS [m], ANGLES [rad]
MOTOR_RADIUS = 0.003 # [m] radius of the tendon spool
SECTION_RADIUS = 0.030  # [m] radius of the section (diameter = 60 mm)
REST_LENGTH = 0.145     # [m] rest length of the section

# Motor arrangement angles [rad]
DELTA1 = np.array([0.0,       2*np.pi/3, -2*np.pi/3])
DELTA2 = np.array([np.pi/3,   np.pi,     -np.pi/3])
DELTA3 = np.array([5*np.pi/6, 3*np.pi/2, np.pi/6])
_DELTA1 = Matrix([0,          2*pi/3,    -2*pi/3])          # 0°, +120°, -120°
_DELTA2 = Matrix([pi/3,       pi,        -pi/3])            # 60°, 180°, -60°
_DELTA3 = Matrix([5*pi/6,     3*pi/2,    pi/6])             # 150°, 270°, 30°

# Precompute A and inv(A) once
A1 = np.array([[-np.cos(di), -np.sin(di), 1.0] for di in DELTA1], dtype=float)  # (3,3)
A2 = np.array([[-np.cos(di), -np.sin(di), 1.0] for di in DELTA2], dtype=float)  # (3,3)
A3 = np.array([[-np.cos(di), -np.sin(di), 1.0] for di in DELTA3], dtype=float)  # (3,3)
A_INV1 = np.linalg.inv(A1)
A_INV2 = np.linalg.inv(A2)
A_INV3 = np.linalg.inv(A3)

# Helper functions for Jacobians
# def R_MATRIX(Dx, Dy, D, d=SECTION_RADIUS):
#         return Matrix([
#             [1 + Dx**2/D**2*(cos(D/d)-1),  Dx*Dy/D**2*(cos(D/d)-1),  Dx/D*sin(D/d)],
#             [Dx*Dy/D**2*(cos(D/d)-1),  1 + Dy**2/D**2*(cos(D/d)-1),  Dy/D*sin(D/d)],
#             [-Dx/D*sin(D/d),               -Dy/D*sin(D/d),            cos(D/d)]
#         ])

# def t_vector(Dx, Dy, D, Dl, L0, d=SECTION_RADIUS):
#     return Matrix([
#         [(d*(L0+Dl)/D**2)*Dx*(1-cos(D/d))],
#         [(d*(L0+Dl)/D**2)*Dy*(1-cos(D/d))],
#         [(d*(L0+Dl)/D**2)*D*sin(D/d)]
#     ])

def R_MATRIX(Dx, Dy, D, d=SECTION_RADIUS, s_param=1.0):
        theta_s = s_param*(D/d)
        return Matrix([
            [1 + Dx**2/D**2*(cos(theta_s)-1),  Dx*Dy/D**2*(cos(theta_s)-1),  Dx/D*sin(theta_s)],
            [Dx*Dy/D**2*(cos(theta_s)-1),  1 + Dy**2/D**2*(cos(theta_s)-1),  Dy/D*sin(theta_s)],
            [-Dx/D*sin(theta_s),               -Dy/D*sin(theta_s),            cos(theta_s)]
        ])

def t_vector(Dx, Dy, D, Dl, L0, d=SECTION_RADIUS, s_param=1.0):
    theta_s = s_param*(D/d)
    return Matrix([
        [(s_param*d*(L0+Dl)/D**2)*Dx*(1-cos(theta_s))],
        [(s_param*d*(L0+Dl)/D**2)*Dy*(1-cos(theta_s))],
        [(s_param*d*(L0+Dl)/D**2)*D*sin(theta_s)]
    ])

# def q2tendon(delta=None):
def pcc2q_motor():
    """
    Jacobian d(q_motor)/d(pcc), shape (9,9).
    pcc = [Dx1,Dy1,Dl1, Dx2,Dy2,Dl2, Dx3,Dy3,Dl3], DELTAL stacks the 3 tendon sets
    q_motor = [q1, q2, q3] with q_motor = DELTAL / MOTOR_RADIUS
    """

    Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3 = symbols(
        "Dx1 Dy1 Dl1 Dx2 Dy2 Dl2 Dx3 Dy3 Dl3") # pcc param
    
    DELTAL1 = Matrix([Dl1 - Dx1*cos(di) - Dy1*sin(di) for di in _DELTA1])
    DELTAL2 = Matrix([Dl2 - Dx2*cos(di) - Dy2*sin(di) for di in _DELTA2])
    DELTAL3 = Matrix([Dl3 - Dx3*cos(di) - Dy3*sin(di) for di in _DELTA3])
    DELTAL  = Matrix.vstack(DELTAL1, DELTAL2, DELTAL3)    
    
    q_motor = DELTAL / MOTOR_RADIUS     # q_motor = A @ pcc / MOTOR_RADIUS
    pcc_vec = Matrix([Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3])

    J = q_motor.jacobian(pcc_vec) # J = d(q_motor)/d(pcc) = A / MOTOR_RADIUS
    return lambdify((Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3), J, modules=['numpy'])

# def q2coordinates():
def pcc2pose_ee():
    """
    Jacobian d(pose)/d(pcc), shape (6,9). 
    pose = [x, y, z, yaw, pitch, roll] of the tip of segment 3.
    Note: the matrix is non-square in the sense that sections are coupled through composition.
    """

    Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3 = symbols(
        "Dx1 Dy1 Dl1 Dx2 Dy2 Dl2 Dx3 Dy3 Dl3")

    L0_1 = L0_2 = L0_3 = REST_LENGTH   # rest length [m]

    # curvature magnitude (regularised)
    D1 = sqrt(Dx1**2 + Dy1**2 + 1e-12)
    D2 = sqrt(Dx2**2 + Dy2**2 + 1e-12)
    D3 = sqrt(Dx3**2 + Dy3**2 + 1e-12)
    
    R1 = R_MATRIX(Dx1, Dy1, D1)
    R2 = R_MATRIX(Dx2, Dy2, D2)
    R3 = R_MATRIX(Dx3, Dy3, D3)
    R  = R1 * R2 * R3

    t1 = t_vector(Dx1, Dy1, D1, Dl1, L0_1)
    t2 = t_vector(Dx2, Dy2, D2, Dl2, L0_2)
    t3 = t_vector(Dx3, Dy3, D3, Dl3, L0_3)
    t  = t1 + R1*t2 + R1*R2*t3

    # Euler angles from R using ZYX convention
    yaw   = atan2(R[1,0], R[0,0] + 1e-12)
    pitch = asin(-R[2,0])
    roll  = atan2(R[2,1], R[2,2] + 1e-12)

    T = Matrix.vstack(t, Matrix([yaw, pitch, roll]))

    pcc_vec = Matrix([Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3])
    J = T.jacobian(pcc_vec)
    return lambdify((Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3), J, modules=['numpy'])


# def tendon2pose(delta=[0, np.radians(120), np.radians(-120)]):
def q_motor2pose_ee():
    """
    Jacobian d(pose)/d(q_motor), shape (6,9). 
    DELTAL = q_motor * MOTOR_RADIUS (we do the conversion inside this function) 
    """

    # Motor values (q12 = segment1 motor2) [rad] 
    q11, q12, q13 = symbols("q11 q12 q13")
    q21, q22, q23 = symbols("q21 q22 q23")
    q31, q32, q33 = symbols("q31 q32 q33")

    L0_1 = L0_2 = L0_3 = REST_LENGTH   # rest length [m]

    # MOTOR TO PCC CONVERSION
    # pcc = A_inv * DELTAL = A_inv * (q * MOTOR_RADIUS)
    pcc1 = Matrix(A_INV1.tolist()) @ (Matrix([q11, q12, q13])*MOTOR_RADIUS)
    pcc2 = Matrix(A_INV2.tolist()) @ (Matrix([q21, q22, q23])*MOTOR_RADIUS)
    pcc3 = Matrix(A_INV3.tolist()) @ (Matrix([q31, q32, q33])*MOTOR_RADIUS)

    Dx1, Dy1, Dl1 = pcc1[0], pcc1[1], pcc1[2]
    Dx2, Dy2, Dl2 = pcc2[0], pcc2[1], pcc2[2]
    Dx3, Dy3, Dl3 = pcc3[0], pcc3[1], pcc3[2]

    D1 = sqrt(Dx1**2 + Dy1**2 + 1e-12)
    D2 = sqrt(Dx2**2 + Dy2**2 + 1e-12)
    D3 = sqrt(Dx3**2 + Dy3**2 + 1e-12)

    R1 = R_MATRIX(Dx1, Dy1, D1)
    R2 = R_MATRIX(Dx2, Dy2, D2)
    R3 = R_MATRIX(Dx3, Dy3, D3)
    R  = R1 * R2 * R3

    t1 = t_vector(Dx1, Dy1, D1, Dl1, L0_1)
    t2 = t_vector(Dx2, Dy2, D2, Dl2, L0_2)
    t3 = t_vector(Dx3, Dy3, D3, Dl3, L0_3)
    t  = t1 + R1*t2 + R1*R2*t3

    yaw   = atan2(R[1,0], R[0,0] + 1e-12)
    pitch = asin(-R[2,0])
    roll  = atan2(R[2,1], R[2,2] + 1e-12)

    T = Matrix.vstack(t, Matrix([yaw, pitch, roll]))
    # T is already a function of q since we substituted pcc = A_inv @ (q * MOTOR_RADIUS) above 
    # so we can directly differentiate w.r.t q
    q_vec = Matrix([q11, q12, q13, q21, q22, q23, q31, q32, q33])
    J = T.jacobian(q_vec)  # d(pose)/d(q_motor)

    return lambdify((q11, q12, q13, q21, q22, q23, q31, q32, q33), J, modules=['numpy'], cse=True)


# def pcc2pose(s_param):
#     """
#     Jacobian for any point along the segment 
#     Jacobian d(pose)/d(pcc), shape (6,9). 
#     pose(s_param) = [x, y, z, yaw, pitch, roll] .
#     Args: s_param: s = [0,1] defines position along the segment (0 = base, 1 = tip).
#     """

#     Dx, Dy, Dl = symbols("Dx Dy Dl")
#     d  = SECTION_RADIUS  # section radius [m]  (diameter = 60 mm)
#     L0 = REST_LENGTH   # rest length [m]
    
#     D = sqrt(Dx**2 + Dy**2 + 1e-12)  # curvature magnitude (regularised)

#     # PCC rotation matrix (Rodrigues on curvature axis)
#     theta_s = s_param*(D/d)
#     R = Matrix([
#         [1 + Dx**2/D**2*(cos(theta_s)-1),  Dx*Dy/D**2*(cos(theta_s)-1),  Dx/D*sin(theta_s)],
#         [Dx*Dy/D**2*(cos(theta_s)-1),  1 + Dy**2/D**2*(cos(theta_s)-1),  Dy/D*sin(theta_s)],
#         [-Dx/D*sin(theta_s),               -Dy/D*sin(theta_s),            cos(theta_s)]
#     ])

#     # PCC tip translation
#     t = Matrix([
#         [(s_param*d*(L0+Dl)/D**2)*Dx*(1-cos(theta_s))],
#         [(s_param*d*(L0+Dl)/D**2)*Dy*(1-cos(theta_s))],
#         [(s_param*d*(L0+Dl)/D**2)*D*sin(theta_s)]
#     ])

#     # ZYX Euler angles from R
#     yaw   = atan2(R[1,0], R[0,0] + 1e-12)
#     pitch = asin(-R[2,0])
#     roll  = atan2(R[2,1], R[2,2] + 1e-12)

#     T = Matrix.vstack(t, Matrix([yaw, pitch, roll]))
#     J = T.jacobian(Matrix([Dx, Dy, Dl]))
#     return lambdify((Dx, Dy, Dl), J, modules=['numpy'])


# def q_motor2pose(s_param):
#     """
#     Jacobian d(pose)/d(q_motor), shape (6,3). 
#     DELTAL = q_motor * MOTOR_RADIUS (we do the conversion inside this function)
#     Args: s_param: s = [0,1] defines position along the segment (0 = base, 1 = tip).
#     """

#     q1, q2, q3 = symbols("q1 q2 q3")
#     d  = SECTION_RADIUS  # section radius [m]  (diameter = 60 mm)
#     L0 = REST_LENGTH   # rest length [m]

#     pcc = A_INV * (Matrix([q1, q2, q3])*MOTOR_RADIUS)   
#     Dx, Dy, Dl = pcc[0], pcc[1], pcc[2]

#     D = sqrt(Dx**2 + Dy**2 + 1e-12)

#     # PCC rotation matrix (Rodrigues on curvature axis)
#     theta_s = s_param*(D/d)
#     R = Matrix([
#         [1 + Dx**2/D**2*(cos(theta_s)-1),  Dx*Dy/D**2*(cos(theta_s)-1),  Dx/D*sin(theta_s)],
#         [Dx*Dy/D**2*(cos(theta_s)-1),  1 + Dy**2/D**2*(cos(theta_s)-1),  Dy/D*sin(theta_s)],
#         [-Dx/D*sin(theta_s),               -Dy/D*sin(theta_s),            cos(theta_s)]
#     ])

#     # PCC tip translation
#     t = Matrix([
#         [(s_param*d*(L0+Dl)/D**2)*Dx*(1-cos(theta_s))],
#         [(s_param*d*(L0+Dl)/D**2)*Dy*(1-cos(theta_s))],
#         [(s_param*d*(L0+Dl)/D**2)*D*sin(theta_s)]
#     ])

#     yaw   = atan2(R[1,0], R[0,0] + 1e-12)
#     pitch = asin(-R[2,0])
#     roll  = atan2(R[2,1], R[2,2] + 1e-12)

#     T = Matrix.vstack(t, Matrix([yaw, pitch, roll]))
#     # T is already a function of q since we substituted pcc = A_inv @ (q * MOTOR_RADIUS) above 
#     # so we can directly differentiate w.r.t q
#     J = T.jacobian(Matrix([q1, q2, q3]))  # d(pose)/d(q_motor)

#     return lambdify((q1, q2, q3), J, modules=['numpy'], cse=True)


if __name__ == "__main__":
    import time

    print("Building Jacobians (symbolic differentiation, takes a few seconds)...")
    # t0 = time.time(); J_qt = pcc2q_motor();      print(f"  pcc2q_motor      done  ({time.time()-t0:.1f}s)")
    # t0 = time.time(); J_qp = pcc2pose_ee(); print(f"  pcc2pose_ee done  ({time.time()-t0:.1f}s)")
    # t0 = time.time(); J_tp = q_motor2pose_ee();   print(f"  q_motor2pose_ee   done  ({time.time()-t0:.1f}s)")

    # # Validate: tendon2pose(L) == q2coordinates(q) @ inv(q2tendon(q))  at 3 random configs
    # print("\nValidation — max |tendon2pose - J_qp @ inv(J_qt)| at 3 random configurations:")
    # rng = np.random.default_rng(0)
    # for i in range(3):
    #     q = rng.uniform([-0, -0.1, 0.5], [1., -.1, 2.])   # random [Dx, Dy, Dl] in mm
    #     L = A @ q                                      # corresponding tendon lengths

    #     q_motor = L / MOTOR_RADIUS                                    # q_motor = DELTAL / r
    #     J_direct = np.array(J_tp(*q_motor))                            # (6,3) from q_motor2pose_ee
    #     J_chain  = np.array(J_qp(*q)) @ np.linalg.inv(np.array(J_qt(*q)))  # (6,3) by chain rule

    #     err = np.max(np.abs(J_direct - J_chain))
    #     print(f"  config {i+1}: q={np.round(q,2)},  max error = {err:.2e}")
    #     print(f"    J_direct:\n{np.round(J_direct,3)}")
    #     print(f"    J_chain:\n{np.round(J_chain,3)}\n")
