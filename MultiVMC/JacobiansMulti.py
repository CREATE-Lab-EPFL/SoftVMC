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
def R_MATRIX(Dx, Dy, D, s_param=1.0):
        d = SECTION_RADIUS
        theta_s = s_param*(D/d)
        return Matrix([
            [1 + Dx**2/D**2*(cos(theta_s)-1),  Dx*Dy/D**2*(cos(theta_s)-1),  Dx/D*sin(theta_s)],
            [Dx*Dy/D**2*(cos(theta_s)-1),  1 + Dy**2/D**2*(cos(theta_s)-1),  Dy/D*sin(theta_s)],
            [-Dx/D*sin(theta_s),               -Dy/D*sin(theta_s),            cos(theta_s)]
        ])

def t_vector(Dx, Dy, D, Dl, L0, s_param=1.0):
    d = SECTION_RADIUS
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
    pcc1 = Matrix(A_INV1.tolist()) * (Matrix([q11, q12, q13])*MOTOR_RADIUS)
    pcc2 = Matrix(A_INV2.tolist()) * (Matrix([q21, q22, q23])*MOTOR_RADIUS)
    pcc3 = Matrix(A_INV3.tolist()) * (Matrix([q31, q32, q33])*MOTOR_RADIUS)

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
    # T is already a function of q since we substituted pcc = A_inv * (q * MOTOR_RADIUS) above 
    # so we can directly differentiate w.r.t q
    q_vec = Matrix([q11, q12, q13, q21, q22, q23, q31, q32, q33])
    J = T.jacobian(q_vec)  # d(pose)/d(q_motor)

    return lambdify((q11, q12, q13, q21, q22, q23, q31, q32, q33), J, modules=['numpy'], cse=True)


def pcc2pose(s_param):
    """
    Jacobian for any point along the segment 
    Jacobian d(pose)/d(pcc), shape (6,9). 
    pose(s_param) = [x, y, z, yaw, pitch, roll] .
    Args: s_param: s = [0,1] defines position along the segment (0 = base, 1 = tip).
    """
    # Declare pcc symbols
    Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3 = symbols(
        "Dx1 Dy1 Dl1 Dx2 Dy2 Dl2 Dx3 Dy3 Dl3")

    L0_1 = L0_2 = L0_3 = REST_LENGTH   # rest length [m]

    # curvature magnitude (regularised)
    D1 = sqrt(Dx1**2 + Dy1**2 + 1e-12)
    D2 = sqrt(Dx2**2 + Dy2**2 + 1e-12)
    D3 = sqrt(Dx3**2 + Dy3**2 + 1e-12)

    if s_param <= 1/3:
        # in segment 1
        R = R_MATRIX(Dx1, Dy1, D1, s_param)
        t = t_vector(Dx1, Dy1, D1, Dl1, L0_1, s_param)

    elif s_param <= 2/3:
        # full segment 1 + segment 2
        R1 = R_MATRIX(Dx1, Dy1, D1)
        R2 = R_MATRIX(Dx2, Dy2, D2, s_param)

        t1 = t_vector(Dx1, Dy1, D1, Dl1, L0_1)
        t2 = t_vector(Dx2, Dy2, D2, Dl2, L0_2, s_param)

        R = R1 * R2
        t = t1 + R1 * t2

    else:
        # full segment 1 + full segment 2 + segment 3
        R1 = R_MATRIX(Dx1, Dy1, D1)
        R2 = R_MATRIX(Dx2, Dy2, D2)
        R3 = R_MATRIX(Dx3, Dy3, D3, s_param)

        t1 = t_vector(Dx1, Dy1, D1, Dl1, L0_1)
        t2 = t_vector(Dx2, Dy2, D2, Dl2, L0_2)
        t3 = t_vector(Dx3, Dy3, D3, Dl3, L0_3, s_param)

        R = R1 * R2 * R3
        t = t1 + R1 * t2 + R1 * R2 * t3

    # ZYX Euler angles from R
    yaw   = atan2(R[1,0], R[0,0] + 1e-12)
    pitch = asin(-R[2,0])
    roll  = atan2(R[2,1], R[2,2] + 1e-12)

    T = Matrix.vstack(t, Matrix([yaw, pitch, roll]))

    pcc_vec = Matrix([Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3])
    J = T.jacobian(pcc_vec)
    return lambdify((Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3), J, modules=['numpy'])


def q_motor2pose(s_param):
    """
    Jacobian d(pose)/d(q_motor), shape (6,9). 
    DELTAL = q_motor * MOTOR_RADIUS (we do the conversion inside this function)
    Args: s_param: s = [0,1] defines position along the segment (0 = base, 1 = tip).
    """

        # Motor values (q12 = segment1 motor2) [rad] 
    q11, q12, q13 = symbols("q11 q12 q13")
    q21, q22, q23 = symbols("q21 q22 q23")
    q31, q32, q33 = symbols("q31 q32 q33")

    L0_1 = L0_2 = L0_3 = REST_LENGTH   # rest length [m]

    # MOTOR TO PCC CONVERSION
    # pcc = A_inv * DELTAL = A_inv * (q * MOTOR_RADIUS)
    pcc1 = Matrix(A_INV1.tolist()) * (Matrix([q11, q12, q13])*MOTOR_RADIUS)
    pcc2 = Matrix(A_INV2.tolist()) * (Matrix([q21, q22, q23])*MOTOR_RADIUS)
    pcc3 = Matrix(A_INV3.tolist()) * (Matrix([q31, q32, q33])*MOTOR_RADIUS)

    Dx1, Dy1, Dl1 = pcc1[0], pcc1[1], pcc1[2]
    Dx2, Dy2, Dl2 = pcc2[0], pcc2[1], pcc2[2]
    Dx3, Dy3, Dl3 = pcc3[0], pcc3[1], pcc3[2]

    D1 = sqrt(Dx1**2 + Dy1**2 + 1e-12)
    D2 = sqrt(Dx2**2 + Dy2**2 + 1e-12)
    D3 = sqrt(Dx3**2 + Dy3**2 + 1e-12)

    if s_param <= 1/3:
        # segment 1
        R = R_MATRIX(Dx1, Dy1, D1, s_param)
        t = t_vector(Dx1, Dy1, D1, Dl1, L0_1, s_param)

    elif s_param <= 2/3:
        # full segment 1 + segment 2
        R1 = R_MATRIX(Dx1, Dy1, D1)
        R2 = R_MATRIX(Dx2, Dy2, D2, s_param)

        t1 = t_vector(Dx1, Dy1, D1, Dl1, L0_1)
        t2 = t_vector(Dx2, Dy2, D2, Dl2, L0_2, s_param)

        R = R1 * R2
        t = t1 + R1 * t2

    else:
        # full segment 1 + full segment 2 + segment 3
        R1 = R_MATRIX(Dx1, Dy1, D1)
        R2 = R_MATRIX(Dx2, Dy2, D2)
        R3 = R_MATRIX(Dx3, Dy3, D3, s_param)

        t1 = t_vector(Dx1, Dy1, D1, Dl1, L0_1)
        t2 = t_vector(Dx2, Dy2, D2, Dl2, L0_2)
        t3 = t_vector(Dx3, Dy3, D3, Dl3, L0_3, s_param)

        R = R1 * R2 * R3
        t = t1 + R1 * t2 + R1 * R2 * t3

    yaw   = atan2(R[1,0], R[0,0] + 1e-12)
    pitch = asin(-R[2,0])
    roll  = atan2(R[2,1], R[2,2] + 1e-12)

    T = Matrix.vstack(t, Matrix([yaw, pitch, roll]))
    # T is already a function of q since we substituted pcc = A_inv * (q * MOTOR_RADIUS) above 
    # so we can directly differentiate w.r.t q
    q_vec = Matrix([q11, q12, q13, q21, q22, q23, q31, q32, q33])
    J = T.jacobian(q_vec)  # d(pose)/d(q_motor)

    return lambdify((q11, q12, q13, q21, q22, q23, q31, q32, q33), J, modules=['numpy'], cse=True)


if __name__ == "__main__":
    import time
    import numpy as np

    np.set_printoptions(precision=4, suppress=True)

    print("Building Jacobians (symbolic differentiation, may take some time)...")
    t0 = time.time(); J_pcc2q   = pcc2q_motor();      print(f"  pcc2q_motor      done ({time.time()-t0:.1f}s)")
    t0 = time.time(); J_pcc  = pcc2pose_ee();      print(f"  pcc2pose_ee      done ({time.time()-t0:.1f}s)")
    t0 = time.time(); J_q    = q_motor2pose_ee();  print(f"  q_motor2pose_ee  done ({time.time()-t0:.1f}s)")

    # Extra: Jacobians for intermediate backbone points
    s_tests = [0.2, 0.5, 0.9]
    J_pcc_s = {}
    J_q_s = {}

    for s in s_tests:
        t0 = time.time(); J_pcc_s[s] = pcc2pose(s);    print(f"  pcc2pose(s={s:.2f})   done ({time.time()-t0:.1f}s)")
        t0 = time.time(); J_q_s[s]   = q_motor2pose(s); print(f"  q_motor2pose(s={s:.2f}) done ({time.time()-t0:.1f}s)")

    print("\nValidation EE: J(q_motor->pose_ee) ?= J(pcc->pose_ee) @ inv(J(pcc->q_motor))")
    rng = np.random.default_rng(42)
    n_tests = 3

    for i in range(n_tests):
        # Random multi-segment PCC config [Dx1,Dy1,Dl1, Dx2,Dy2,Dl2, Dx3,Dy3,Dl3]
        pcc = np.array([
            rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01),
            rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01),
            rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01),
        ], dtype=float)

        # q_motor from pcc via your linear map (segment-wise)
        pcc1, pcc2, pcc3 = pcc[0:3], pcc[3:6], pcc[6:9]
        q1 = (A1 @ pcc1) / MOTOR_RADIUS
        q2 = (A2 @ pcc2) / MOTOR_RADIUS
        q3 = (A3 @ pcc3) / MOTOR_RADIUS
        q_motor = np.concatenate([q1, q2, q3])

        J_pcc2q_num  = np.array(J_pcc2q(*pcc), dtype=float)    # (9,9) d(q_motor)/d(pcc)
        J_pcc_num = np.array(J_pcc(*pcc), dtype=float)   # (6,9) d(pose)/d(pcc)
        J_q_num   = np.array(J_q(*q_motor), dtype=float) # (6,9) d(pose)/d(q_motor)

        J_chain = J_pcc_num @ np.linalg.inv(J_pcc2q_num)
        err = np.max(np.abs(J_q_num - J_chain))

        print(f"  test {i+1}: max|J_direct - J_chain| = {err:.3e}")
        if err > 1e-6:
            print("    WARNING: error above tolerance (1e-6).")

    print("\nValidation s_param: J(q_motor->pose(s)) ?= J(pcc->pose(s)) @ inv(J(pcc->q_motor))")
    for s in s_tests:
        print(f"  s = {s:.2f}")
        for i in range(n_tests):
            pcc = np.array([
                rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01),
                rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01),
                rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01),
            ], dtype=float)

            pcc1, pcc2, pcc3 = pcc[0:3], pcc[3:6], pcc[6:9]
            q1 = (A1 @ pcc1) / MOTOR_RADIUS
            q2 = (A2 @ pcc2) / MOTOR_RADIUS
            q3 = (A3 @ pcc3) / MOTOR_RADIUS
            q_motor = np.concatenate([q1, q2, q3])

            J_pcc2q_num    = np.array(J_pcc2q(*pcc), dtype=float)         # (9,9)
            J_pcc_s_num = np.array(J_pcc_s[s](*pcc), dtype=float)   # (6,9)
            J_q_s_num   = np.array(J_q_s[s](*q_motor), dtype=float) # (6,9)

            J_chain_s = J_pcc_s_num @ np.linalg.inv(J_pcc2q_num)
            err_s = np.max(np.abs(J_q_s_num - J_chain_s))
            print(f"    test {i+1}: max error = {err_s:.3e}")

    print("\nDone.")