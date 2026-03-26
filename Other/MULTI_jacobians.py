import numpy as np
from sympy import symbols, sqrt, atan2, cos, sin, Matrix, lambdify, asin, pi # type: ignore

# Units: LENGTHS [mm], ANGLES [rad]
# For a single section use SINGLE_jacobians.py instead.

# Default tendon angular positions per section [rad]
# Each section has 3 tendons at [base, base+120°, base-120°]
# _DELTA1 = [np.radians(0),   np.radians(120),  np.radians(-120)]
# _DELTA2 = [np.radians(60),  np.radians(180),  np.radians(-60)]
# _DELTA3 = [np.radians(150), np.radians(270),  np.radians(30)]
# SymPy angles NO numpy (new)
_DELTA1 = Matrix([0,          2*pi/3,      -2*pi/3])          # 0°, +120°, -120°
_DELTA2 = Matrix([pi/3,       pi,          -pi/3])            # 60°, 180°, -60°
_DELTA3 = Matrix([5*pi/6,     3*pi/2,      pi/6])             # 150°, 270°, 30°

# def pcc2q_motor():
def q2tendon(delta1=None, delta2=None, delta3=None):
    """
    Jacobian d(DELTAL)/d(q), shape (9,9).
    q = [Dx1,Dy1,Dl1, Dx2,Dy2,Dl2, Dx3,Dy3,Dl3], DELTAL stacks the 3 tendon sets.
    delta_i: list of 3 tendon angles [rad] for section i.
    """
    if delta1 is None: delta1 = _DELTA1
    if delta2 is None: delta2 = _DELTA2
    if delta3 is None: delta3 = _DELTA3

    Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3 = symbols(
        "Dx1 Dy1 Dl1 Dx2 Dy2 Dl2 Dx3 Dy3 Dl3")

    # DELTAL1 = Matrix([Dl1 - Dx1*np.cos(d) - Dy1*np.sin(d) for d in delta1])
    # DELTAL2 = Matrix([Dl2 - Dx2*np.cos(d) - Dy2*np.sin(d) for d in delta2])
    # DELTAL3 = Matrix([Dl3 - Dx3*np.cos(d) - Dy3*np.sin(d) for d in delta3])
    # No numpy in sympy expressions, use cos/sin from sympy (new)
    DELTAL1 = Matrix([Dl1 - Dx1*cos(di) - Dy1*sin(di) for di in delta1])
    DELTAL2 = Matrix([Dl2 - Dx2*cos(di) - Dy2*sin(di) for di in delta2])
    DELTAL3 = Matrix([Dl3 - Dx3*cos(di) - Dy3*sin(di) for di in delta3])
    DELTAL  = Matrix.vstack(DELTAL1, DELTAL2, DELTAL3)

    q_vec = Matrix([Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3])
    J = DELTAL.jacobian(q_vec)
    return lambdify((Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3), J, modules=['numpy'])

# def pcc2pose_ee():
def q2coordinates():
    """
    Jacobian d(pose)/d(q), shape (6,9). pose = [x,y,z,yaw,pitch,roll] of the tip of section 3.
    Note: the matrix is non-square in the sense that sections are coupled through composition.
    """
    Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3 = symbols(
        "Dx1 Dy1 Dl1 Dx2 Dy2 Dl2 Dx3 Dy3 Dl3")
    d    = 30    # section radius [mm]
    L0_1 = L0_2 = L0_3 = 145

    D1 = sqrt(Dx1**2 + Dy1**2 + 1e-12)
    D2 = sqrt(Dx2**2 + Dy2**2 + 1e-12)
    D3 = sqrt(Dx3**2 + Dy3**2 + 1e-12)

    def R_MATRIX(Dx, Dy, D):
        return Matrix([
            [1 + Dx**2/D**2*(cos(D/d)-1),  Dx*Dy/D**2*(cos(D/d)-1),  Dx/D*sin(D/d)],
            [Dx*Dy/D**2*(cos(D/d)-1),  1 + Dy**2/D**2*(cos(D/d)-1),  Dy/D*sin(D/d)],
            [-Dx/D*sin(D/d),               -Dy/D*sin(D/d),            cos(D/d)]
        ])

    def t_vector(Dx, Dy, D, Dl, L0):
        return Matrix([
            [(d*(L0+Dl)/D**2)*Dx*(1-cos(D/d))],
            [(d*(L0+Dl)/D**2)*Dy*(1-cos(D/d))],
            [(d*(L0+Dl)/D**2)*D*sin(D/d)]
        ])

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
    q_vec = Matrix([Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3])
    J = T.jacobian(q_vec)
    return lambdify((Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3), J, modules=['numpy'])


def q2xyz():
    """
    Jacobian d([x1,y1,z1, x2,y2,z2, x3,y3,z3])/d(q), shape (9,9).
    Tip positions of each section in global frame.
    """
    Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3 = symbols(
        "Dx1 Dy1 Dl1 Dx2 Dy2 Dl2 Dx3 Dy3 Dl3")
    d    = 30
    L0_1 = L0_2 = L0_3 = 145

    D1 = sqrt(Dx1**2 + Dy1**2 + 1e-12)
    D2 = sqrt(Dx2**2 + Dy2**2 + 1e-12)
    D3 = sqrt(Dx3**2 + Dy3**2 + 1e-12)

    def R_MATRIX(Dx, Dy, D):
        return Matrix([
            [1 + Dx**2/D**2*(cos(D/d)-1),  Dx*Dy/D**2*(cos(D/d)-1),  Dx/D*sin(D/d)],
            [Dx*Dy/D**2*(cos(D/d)-1),  1 + Dy**2/D**2*(cos(D/d)-1),  Dy/D*sin(D/d)],
            [-Dx/D*sin(D/d),               -Dy/D*sin(D/d),            cos(D/d)]
        ])

    def t_vector(Dx, Dy, D, Dl, L0):
        return Matrix([
            [(d*(L0+Dl)/D**2)*Dx*(1-cos(D/d))],
            [(d*(L0+Dl)/D**2)*Dy*(1-cos(D/d))],
            [(d*(L0+Dl)/D**2)*D*sin(D/d)]
        ])

    R1 = R_MATRIX(Dx1, Dy1, D1)
    R2 = R_MATRIX(Dx2, Dy2, D2)

    t1 = t_vector(Dx1, Dy1, D1, Dl1, L0_1)
    t2 = t_vector(Dx2, Dy2, D2, Dl2, L0_2)
    t3 = t_vector(Dx3, Dy3, D3, Dl3, L0_3)
    t1_global = t1
    t2_global = t1 + R1*t2
    t3_global = t1 + R1*t2 + R1*R2*t3

    T = Matrix.vstack(t1_global, t2_global, t3_global)
    q_vec = Matrix([Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3])
    J = T.jacobian(q_vec)
    return lambdify((Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3), J, modules=['numpy'])

# def q_motor2pose_ee():
def tendon2pose(delta1=_DELTA1, delta2=_DELTA2, delta3=_DELTA3):
    """
    Jacobian d(pose)/d(DELTAL), shape (6,9). Direct end-to-end: tendon lengths -> tip pose.

    delta_i: list of 3 tendon angles [rad] for section i.
    """

    # Tendon length change symbols [mm]
    L11, L21, L31 = symbols("L11 L21 L31")
    L12, L22, L32 = symbols("L12 L22 L32")
    L13, L23, L33 = symbols("L13 L23 L33")

    d    = 30
    L0_1 = L0_2 = L0_3 = 145

    def inv_from_delta(delta):
        """Numeric A^{-1} for a section with given tendon angles."""
        A = np.array([[-np.cos(di), -np.sin(di), 1.0] for di in delta])
        return Matrix(np.linalg.inv(A).tolist())

    # Express q_i = A_i^{-1} @ L_i as linear symbolic functions
    q1 = inv_from_delta(delta1) @ Matrix([L11, L21, L31])
    q2 = inv_from_delta(delta2) @ Matrix([L12, L22, L32])
    q3 = inv_from_delta(delta3) @ Matrix([L13, L23, L33])
    Dx1, Dy1, Dl1 = q1[0], q1[1], q1[2]
    Dx2, Dy2, Dl2 = q2[0], q2[1], q2[2]
    Dx3, Dy3, Dl3 = q3[0], q3[1], q3[2]

    D1 = sqrt(Dx1**2 + Dy1**2 + 1e-12)
    D2 = sqrt(Dx2**2 + Dy2**2 + 1e-12)
    D3 = sqrt(Dx3**2 + Dy3**2 + 1e-12)

    def R_MATRIX(Dx, Dy, D):
        return Matrix([
            [1 + Dx**2/D**2*(cos(D/d)-1),  Dx*Dy/D**2*(cos(D/d)-1),  Dx/D*sin(D/d)],
            [Dx*Dy/D**2*(cos(D/d)-1),  1 + Dy**2/D**2*(cos(D/d)-1),  Dy/D*sin(D/d)],
            [-Dx/D*sin(D/d),               -Dy/D*sin(D/d),            cos(D/d)]
        ])

    def t_vector(Dx, Dy, D, Dl, L0):
        return Matrix([
            [(d*(L0+Dl)/D**2)*Dx*(1-cos(D/d))],
            [(d*(L0+Dl)/D**2)*Dy*(1-cos(D/d))],
            [(d*(L0+Dl)/D**2)*D*sin(D/d)]
        ])

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
    vars_ = Matrix([L11, L21, L31, L12, L22, L32, L13, L23, L33])
    J = T.jacobian(vars_)
    return lambdify((L11, L21, L31, L12, L22, L32, L13, L23, L33),
                    J, modules=['numpy'], cse=True)


if __name__ == "__main__":
    import time

    print("Building Jacobians (symbolic differentiation, takes a few seconds)...")
    t0 = time.time(); J_qt = q2tendon();      print(f"  q2tendon       done  ({time.time()-t0:.1f}s)")
    t0 = time.time(); J_qp = q2coordinates(); print(f"  q2coordinates  done  ({time.time()-t0:.1f}s)")
    t0 = time.time(); J_qx = q2xyz();         print(f"  q2xyz          done  ({time.time()-t0:.1f}s)")
    t0 = time.time(); J_tp = tendon2pose();   print(f"  tendon2pose    done  ({time.time()-t0:.1f}s)")

    # Block-diagonal tendon map: L = A_block @ q  (each section independently)
    def make_A(delta):
        return np.array([[-np.cos(di), -np.sin(di), 1.0] for di in delta])
    A1, A2, A3 = make_A(_DELTA1), make_A(_DELTA2), make_A(_DELTA3)
    A_block = np.block([[A1, np.zeros((3,6))],
                        [np.zeros((3,3)), A2, np.zeros((3,3))],
                        [np.zeros((3,6)), A3]])

    # Validate: tendon2pose(L) == q2coordinates(q) @ inv(q2tendon(q))  at 3 random configs
    print("\nValidation — max |tendon2pose - J_qp @ inv(J_qt)| at 3 random configurations:")
    rng = np.random.default_rng(0)
    for i in range(3):
        q = rng.uniform([-10, -10, -5]*3, [10, 10, 5]*3)   # random q for all 3 sections
        L = A_block @ q                                       # corresponding tendon lengths

        J_direct = np.array(J_tp(*L))                                        # (6,9) from tendon2pose
        J_chain  = np.array(J_qp(*q)) @ np.linalg.inv(np.array(J_qt(*q)))   # (6,9) by chain rule

        err = np.max(np.abs(J_direct - J_chain))
        print(f"  config {i+1}: max error = {err:.2e}")
