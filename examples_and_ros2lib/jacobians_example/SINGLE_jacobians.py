import numpy as np
from sympy import symbols, sqrt, atan2, cos, sin, Matrix, lambdify, asin # type: ignore

# Units: LENGTHS [mm], ANGLES [rad]

def q2tendon(delta=None):
    """
    Jacobian d(DELTAL)/d(q), shape (3,3). q = [Dx, Dy, Dl], DELTAL = [dL1, dL2, dL3].
    delta: tendon angular positions [rad], default [0, 2pi/3, -2pi/3].
    """
    if delta is None:
        delta = [0, np.radians(120), np.radians(-120)]

    Dx, Dy, Dl = symbols("Dx Dy Dl")
    DELTAL = Matrix([Dl - Dx*np.cos(d) - Dy*np.sin(d) for d in delta])

    J = DELTAL.jacobian(Matrix([Dx, Dy, Dl]))
    return lambdify((Dx, Dy, Dl), J, modules=['numpy'])


def q2coordinates():
    """Jacobian d(pose)/d(q), shape (6,3). pose = [x, y, z, yaw, pitch, roll]."""

    Dx, Dy, Dl = symbols("Dx Dy Dl")
    d  = 30    # section radius [mm]  (diameter = 60 mm)
    L0 = 145   # rest length [mm]

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


def tendon2pose(delta=[0, np.radians(120), np.radians(-120)]):
    """
    Jacobian d(pose)/d(DELTAL), shape (6,3). Direct end-to-end: tendon lengths -> pose.

    delta: tendon angular positions [rad], default [0, 2pi/3, -2pi/3].
    """

    L1, L2, L3 = symbols("L1 L2 L3")
    d  = 30
    L0 = 145

    # A[i] = [-cos(delta[i]), -sin(delta[i]), 1]  ->  DELTAL = A @ [Dx, Dy, Dl]
    A_num = np.array([[-np.cos(di), -np.sin(di), 1.0] for di in delta])
    A_inv = Matrix(np.linalg.inv(A_num).tolist())   # numeric inverse as SymPy matrix

    q = A_inv @ Matrix([L1, L2, L3])   # [Dx, Dy, Dl] as linear functions of L1,L2,L3
    Dx, Dy, Dl = q[0], q[1], q[2]

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
    J = T.jacobian(Matrix([L1, L2, L3]))
    return lambdify((L1, L2, L3), J, modules=['numpy'], cse=True)


if __name__ == "__main__":
    import time

    print("Building Jacobians (symbolic differentiation, takes a few seconds)...")
    t0 = time.time(); J_qt = q2tendon();      print(f"  q2tendon      done  ({time.time()-t0:.1f}s)")
    t0 = time.time(); J_qp = q2coordinates(); print(f"  q2coordinates done  ({time.time()-t0:.1f}s)")
    t0 = time.time(); J_tp = tendon2pose();   print(f"  tendon2pose   done  ({time.time()-t0:.1f}s)")

    delta = [0, np.radians(120), np.radians(-120)]
    A = np.array([[-np.cos(di), -np.sin(di), 1.0] for di in delta])  # tendon map: L = A @ q

    # Validate: tendon2pose(L) == q2coordinates(q) @ inv(q2tendon(q))  at 3 random configs
    print("\nValidation — max |tendon2pose - J_qp @ inv(J_qt)| at 3 random configurations:")
    rng = np.random.default_rng(0)
    for i in range(3):
        q = rng.uniform([-10, -10, -5], [10, 10, 5])   # random [Dx, Dy, Dl] in mm
        L = A @ q                                        # corresponding tendon lengths

        J_direct = np.array(J_tp(*L))                               # (6,3) from tendon2pose
        J_chain  = np.array(J_qp(*q)) @ np.linalg.inv(np.array(J_qt(*q)))  # (6,3) by chain rule

        err = np.max(np.abs(J_direct - J_chain))
        print(f"  config {i+1}: q={np.round(q,2)},  max error = {err:.2e}")
