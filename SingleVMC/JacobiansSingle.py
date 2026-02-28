import numpy as np
from sympy import symbols, sqrt, atan2, cos, sin, Matrix, lambdify, asin, pi# type: ignore

# Units: LENGTHS [m], ANGLES [rad]
MOTOR_RADIUS = 0.003 # [m] radius of the tendon spool
SECTION_RADIUS = 0.030  # [m] radius of the section (diameter = 60 mm)
REST_LENGTH = 0.145     # [m] rest length of the section

# def q2tendon(delta=None):
def pcc2q_motor(delta=None):
    """
    Jacobian d(q_motor)/d(pcc), shape (3,3).
    pcc = [Dx, Dy, Dl], q_motor = [q1, q2, q3] with q_motor = DELTAL / MOTOR_RADIUS
    """

    if delta is None:
        delta = Matrix([0, 2*pi/3, -2*pi/3])

    Dx, Dy, Dl = symbols("Dx Dy Dl") # pcc param
    DELTAL = Matrix([Dl - Dx*cos(di) - Dy*sin(di) for di in delta]) # deltaL = A @ pcc
    q_motor = DELTAL / MOTOR_RADIUS                                # q_motor = A @ pcc / MOTOR_RADIUS

    J = q_motor.jacobian(Matrix([Dx, Dy, Dl])) # J = d(q_motor)/d(pcc) = A / MOTOR_RADIUS
    return lambdify((Dx, Dy, Dl), J, modules=['numpy'])


# def q2coordinates():
def pcc2pose_ee():
    """Jacobian d(pose)/d(pcc), shape (6,3). pose = [x, y, z, yaw, pitch, roll]."""

    Dx, Dy, Dl = symbols("Dx Dy Dl")
    d  = SECTION_RADIUS  # section radius [m]  (diameter = 60 mm)
    L0 = REST_LENGTH   # rest length [m]

    D = sqrt(Dx**2 + Dy**2 + 1e-12)  # curvature magnitude (regularised)

    # PCC rotation matrix (Rodrigues on curvature axis)
    R = Matrix([
        [1 + Dx**2/D**2*(cos(D/d)-1),  Dx*Dy/D**2*(cos(D/d)-1),  Dx/D*sin(D/d)],
        [Dx*Dy/D**2*(cos(D/d)-1),  1 + Dy**2/D**2*(cos(D/d)-1),  Dy/D*sin(D/d)],
        [-Dx/D*sin(D/d),               -Dy/D*sin(D/d),            cos(D/d)]
    ])

    # PCC tip translation
    t = Matrix([
        [(d*(L0+Dl)/D**2)*Dx*(1-cos(D/d))],
        [(d*(L0+Dl)/D**2)*Dy*(1-cos(D/d))],
        [(d*(L0+Dl)/D**2)*D*sin(D/d)]
    ])

    # ZYX Euler angles from R
    yaw   = atan2(R[1,0], R[0,0] + 1e-12)
    pitch = asin(-R[2,0])
    roll  = atan2(R[2,1], R[2,2] + 1e-12)

    T = Matrix.vstack(t, Matrix([yaw, pitch, roll]))
    J = T.jacobian(Matrix([Dx, Dy, Dl]))
    return lambdify((Dx, Dy, Dl), J, modules=['numpy'])


# def tendon2pose(delta=[0, np.radians(120), np.radians(-120)]):
def q_motor2pose_ee(delta=[0, np.radians(120), np.radians(-120)]):
    """
    Jacobian d(pose)/d(q_motor), shape (6,3). 
    DELTAL = q_motor * MOTOR_RADIUS (we do the conversion inside this function) 
    delta: tendon angular positions [rad], default [0, 2pi/3, -2pi/3].
    """

    q1, q2, q3 = symbols("q1 q2 q3")
    d  = SECTION_RADIUS  # section radius [m]  (diameter = 60 mm)
    L0 = REST_LENGTH   # rest length [m]

    # A[i] = [-cos(delta[i]), -sin(delta[i]), 1]  ->  DELTAL = A @ [Dx, Dy, Dl]
    A_num = np.array([[-np.cos(di), -np.sin(di), 1.0] for di in delta])
    A_inv = Matrix(np.linalg.inv(A_num).tolist())   # numeric inverse as SymPy matrix

    # pcc = A_inv * DELTAL = A_inv * (q * MOTOR_RADIUS)
    pcc = A_inv @ (Matrix([q1, q2, q3])*MOTOR_RADIUS)   
    Dx, Dy, Dl = pcc[0], pcc[1], pcc[2]

    D = sqrt(Dx**2 + Dy**2 + 1e-12)

    R = Matrix([
        [1 + Dx**2/D**2*(cos(D/d)-1),  Dx*Dy/D**2*(cos(D/d)-1),  Dx/D*sin(D/d)],
        [Dx*Dy/D**2*(cos(D/d)-1),  1 + Dy**2/D**2*(cos(D/d)-1),  Dy/D*sin(D/d)],
        [-Dx/D*sin(D/d),               -Dy/D*sin(D/d),            cos(D/d)]
    ])

    t = Matrix([
        [(d*(L0+Dl)/D**2)*Dx*(1-cos(D/d))],
        [(d*(L0+Dl)/D**2)*Dy*(1-cos(D/d))],
        [(d*(L0+Dl)/D**2)*D*sin(D/d)]
    ])

    yaw   = atan2(R[1,0], R[0,0] + 1e-12)
    pitch = asin(-R[2,0])
    roll  = atan2(R[2,1], R[2,2] + 1e-12)

    T = Matrix.vstack(t, Matrix([yaw, pitch, roll]))
    # T is already a function of q since we substituted pcc = A_inv @ (q * MOTOR_RADIUS) above 
    # so we can directly differentiate w.r.t q
    J = T.jacobian(Matrix([q1, q2, q3]))  # d(pose)/d(q_motor)

    return lambdify((q1, q2, q3), J, modules=['numpy'], cse=True)


def pcc2pose(s_param):
    """
    Jacobian for any point along the segment 
    d(pose)/d(pcc), shape (6,3). pose = [x, y, z, yaw, pitch, roll].
    Args: s_param: s = [0,1] defines position along the segment (0 = base, 1 = tip).
    """

    Dx, Dy, Dl = symbols("Dx Dy Dl")
    d  = SECTION_RADIUS  # section radius [m]  (diameter = 60 mm)
    L0 = REST_LENGTH   # rest length [m]
    
    D = sqrt(Dx**2 + Dy**2 + 1e-12)  # curvature magnitude (regularised)

    # PCC rotation matrix (Rodrigues on curvature axis)
    theta_s = s_param*(D/d)
    R = Matrix([
        [1 + Dx**2/D**2*(cos(theta_s)-1),  Dx*Dy/D**2*(cos(theta_s)-1),  Dx/D*sin(theta_s)],
        [Dx*Dy/D**2*(cos(theta_s)-1),  1 + Dy**2/D**2*(cos(theta_s)-1),  Dy/D*sin(theta_s)],
        [-Dx/D*sin(theta_s),               -Dy/D*sin(theta_s),            cos(theta_s)]
    ])

    # PCC tip translation
    t = Matrix([
        [(s_param*d*(L0+Dl)/D**2)*Dx*(1-cos(theta_s))],
        [(s_param*d*(L0+Dl)/D**2)*Dy*(1-cos(theta_s))],
        [(s_param*d*(L0+Dl)/D**2)*D*sin(theta_s)]
    ])

    # ZYX Euler angles from R
    yaw   = atan2(R[1,0], R[0,0] + 1e-12)
    pitch = asin(-R[2,0])
    roll  = atan2(R[2,1], R[2,2] + 1e-12)

    T = Matrix.vstack(t, Matrix([yaw, pitch, roll]))
    J = T.jacobian(Matrix([Dx, Dy, Dl]))
    return lambdify((Dx, Dy, Dl), J, modules=['numpy'])


def q_motor2pose(s_param,delta=[0, np.radians(120), np.radians(-120)]):
    """
    Jacobian d(pose)/d(q_motor), shape (6,3). 
    DELTAL = q_motor * MOTOR_RADIUS (we do the conversion inside this function)
    Args: s_param: s = [0,1] defines position along the segment (0 = base, 1 = tip).
          delta: tendon angular positions [rad], default [0, 2pi/3, -2pi/3].
    """

    q1, q2, q3 = symbols("q1 q2 q3")
    d  = SECTION_RADIUS  # section radius [m]  (diameter = 60 mm)
    L0 = REST_LENGTH   # rest length [m]

    # A[i] = [-cos(delta[i]), -sin(delta[i]), 1]  ->  DELTAL = A @ [Dx, Dy, Dl]
    A_num = np.array([[-np.cos(di), -np.sin(di), 1.0] for di in delta])
    A_inv = Matrix(np.linalg.inv(A_num).tolist())   # numeric inverse as SymPy matrix

    # pcc = A_inv * DELTAL = A_inv * (q * MOTOR_RADIUS)
    pcc = A_inv * (Matrix([q1, q2, q3])*MOTOR_RADIUS)   
    Dx, Dy, Dl = pcc[0], pcc[1], pcc[2]

    D = sqrt(Dx**2 + Dy**2 + 1e-12)

    # PCC rotation matrix (Rodrigues on curvature axis)
    theta_s = s_param*(D/d)
    R = Matrix([
        [1 + Dx**2/D**2*(cos(theta_s)-1),  Dx*Dy/D**2*(cos(theta_s)-1),  Dx/D*sin(theta_s)],
        [Dx*Dy/D**2*(cos(theta_s)-1),  1 + Dy**2/D**2*(cos(theta_s)-1),  Dy/D*sin(theta_s)],
        [-Dx/D*sin(theta_s),               -Dy/D*sin(theta_s),            cos(theta_s)]
    ])

    # PCC tip translation
    t = Matrix([
        [(s_param*d*(L0+Dl)/D**2)*Dx*(1-cos(theta_s))],
        [(s_param*d*(L0+Dl)/D**2)*Dy*(1-cos(theta_s))],
        [(s_param*d*(L0+Dl)/D**2)*D*sin(theta_s)]
    ])

    yaw   = atan2(R[1,0], R[0,0] + 1e-12)
    pitch = asin(-R[2,0])
    roll  = atan2(R[2,1], R[2,2] + 1e-12)

    T = Matrix.vstack(t, Matrix([yaw, pitch, roll]))
    # T is already a function of q since we substituted pcc = A_inv @ (q * MOTOR_RADIUS) above 
    # so we can directly differentiate w.r.t q
    J = T.jacobian(Matrix([q1, q2, q3]))  # d(pose)/d(q_motor)

    return lambdify((q1, q2, q3), J, modules=['numpy'], cse=True)


if __name__ == "__main__":
    import time

    print("Building Jacobians (symbolic differentiation, takes a few seconds)...")
    t0 = time.time(); J_qt = pcc2q_motor();      print(f"  pcc2q_motor      done  ({time.time()-t0:.1f}s)")
    t0 = time.time(); J_qp = pcc2pose_ee(); print(f"  pcc2pose_ee done  ({time.time()-t0:.1f}s)")
    t0 = time.time(); J_tp = q_motor2pose_ee();   print(f"  q_motor2pose_ee   done  ({time.time()-t0:.1f}s)")

    delta = [0, np.radians(120), np.radians(-120)]
    A = np.array([[-np.cos(di), -np.sin(di), 1.0] for di in delta])  # tendon map: L = A @ q

    # Validate: tendon2pose(L) == q2coordinates(q) @ inv(q2tendon(q))  at 3 random configs
    print("\nValidation — max |tendon2pose - J_qp @ inv(J_qt)| at 3 random configurations:")
    rng = np.random.default_rng(0)
    for i in range(3):
        q = rng.uniform([-10, -10, -5], [10, 10, 5])   # random [Dx, Dy, Dl] in mm
        L = A @ q                                      # corresponding tendon lengths

        J_direct = np.array(J_tp(*L))                               # (6,3) from tendon2pose
        J_chain  = np.array(J_qp(*q)) @ np.linalg.inv(np.array(J_qt(*q)))  # (6,3) by chain rule

        err = np.max(np.abs(J_direct - J_chain))
        print(f"  config {i+1}: q={np.round(q,2)},  max error = {err:.2e}")
