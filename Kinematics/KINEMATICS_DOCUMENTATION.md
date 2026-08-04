# Kinematics Documentation — 3-Segment Soft Continuum Arm

## Quick Start

```python
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'Kinematics'))

from FK_Multi import FK_Multi, FK_Multi_ee
from JacobiansMulti import q_motor2pose, q_motor2pose_ee, pcc2q_motor
```

---

## Motor Order Convention

The arm has **9 Dynamixel motors** (IDs 11–19), three per segment.

| Motor HW IDs | Segment | Tendon angles δᵢ |
|---|---|---|
| 11, 12, 13 | Segment 1 (proximal) | 0°, +120°, −120° |
| 14, 15, 16 | Segment 2 (middle)   | 60°, 180°, −60°  |
| 17, 18, 19 | Segment 3 (distal)   | 150°, 270°, 30°  |

Motor positions are measured in **radians** (encoder counts converted externally).

---

## Piecewise Constant Curvature (PCC) Parametrization

Each segment `i` is described by three scalar parameters:

| Symbol | Meaning | Units |
|---|---|---|
| `Dxᵢ` | lateral curvature in x | rad/m |
| `Dyᵢ` | lateral curvature in y | rad/m |
| `Dlᵢ` | elongation (extra length) | m |

The full PCC state vector is `pcc = [Dx1, Dy1, Dl1, Dx2, Dy2, Dl2, Dx3, Dy3, Dl3] ∈ ℝ⁹`.

### Physical constants

| Constant | Symbol | Value |
|---|---|---|
| Tendon spool radius | `r` | 3 mm |
| Segment cross-section radius | `d` | 30 mm |
| Rest length per segment | `L₀` | 145 mm |

### Tendon-to-PCC map (A matrix)

For segment `i` with motor arrangement angles `δ₁, δ₂, δ₃`:

```
ΔLⱼ = Dlᵢ − Dxᵢ cos(δⱼ) − Dyᵢ sin(δⱼ)    j = 1,2,3

A_i  =  [ −cos δ₁  −sin δ₁  1 ]
         [ −cos δ₂  −sin δ₂  1 ]
         [ −cos δ₃  −sin δ₃  1 ]

pcc_i = A_i⁻¹ · ΔL_i
q_motor_i = ΔL_i / r    (motor angle ↔ tendon elongation via spool radius)
```

The matrices `A_INV1`, `A_INV2`, `A_INV3` are precomputed in `JacobiansMulti.py`.

---

## Rotation matrix and translation (PCC segment)

For a segment with curvature parameters `(Dx, Dy)` and arc length `s ∈ [0,1]`:

```
D     = √(Dx² + Dy²)          (total curvature magnitude)
θ(s)  = s · D / d             (bending angle at arc position s)

R(s) = I  +  (cos θ − 1)/D² · [Dx²   DxDy   0 ] + sin θ / D · [0  0   Dx]
                                 [DxDy  Dy²   0 ]               [0  0   Dy]
                                 [0     0     0 ]               [-Dx -Dy  0]

t(s) = (s · d · (L₀+Dl) / D²) · [Dx(1 − cos θ), Dy(1 − cos θ), D sin θ]ᵀ
```

This gives the homogeneous transform `T(s) = [R(s) | t(s)]` of any point along the segment arc.

---

## Kinematic Chain

The full arm FK chains three segment transforms:

```
T_base  →  T₁(1)  →  T₂(1)  →  T₃(s_tip)  →  tip pose
```

`FK_Multi(q_motor, s_param)` returns the 6-DOF pose `[x, y, z, yaw, pitch, roll]`
at arc-length fraction `s_param ∈ [0, 1]` measured along the **total** 3-segment
backbone (`s_param` is required — 0 = base, 1/3 = end of segment 1, 2/3 = end of
segment 2, 1 = tip). To sample multiple points along the arm for visualization,
call it once per desired `s_param`.

`FK_Multi_ee(q_motor)` is a convenience wrapper equivalent to `FK_Multi(q_motor, 1.0)`
— it also returns the full 6-DOF end-effector pose, not just XYZ.

---

## API Reference

### `JacobiansMulti.py`

| Symbol | Type | Description |
|---|---|---|
| `MOTOR_RADIUS` | float | Spool radius r = 0.003 m |
| `SECTION_RADIUS` | float | Cross-section radius d = 0.030 m |
| `REST_LENGTH` | float | Rest length L₀ = 0.145 m |
| `A_INV1`, `A_INV2`, `A_INV3` | (3,3) ndarray | Precomputed A⁻¹ per segment |
| `pcc2q_motor()` | → callable | Takes no args. Returns `J(Dx1,Dy1,Dl1,...,Dx3,Dy3,Dl3)`, a function of 9 scalars returning the (9,9) Jacobian d(q_motor)/d(pcc) |
| `q_motor2pose(s_param)` | → callable | Takes the arc-length fraction `s_param ∈ [0,1]` (not a configuration). Returns `J(q11,q12,...,q33)`, a function of 9 scalars returning the (6,9) Jacobian d(pose)/d(q_motor) at that point along the arm |
| `q_motor2pose_ee()` | → callable | Takes no args (implicitly `s_param=1`, the tip). Returns `J(q11,...,q33)`, the (6,9) end-effector Jacobian |

All three factory functions build the Jacobian **symbolically with sympy** and
`lambdify` it into a fast numeric function — the factory call itself is slow
(seconds), so compile once and reuse the returned function on every control step.

**Usage pattern:**

```python
from JacobiansMulti import q_motor2pose, q_motor2pose_ee

J_ee_func = q_motor2pose_ee()          # compile once (slow)
J_ee = np.array(J_ee_func(*q_motor))   # evaluate at runtime (fast) -> (6, 9)

J_func = q_motor2pose(0.6667)          # compile once, for a point at s=2/3 (tip of segment 2)
J = np.array(J_func(*q_motor))         # -> (6, 9)
```

### `FK_Multi.py`

```python
from FK_Multi import FK_Multi, FK_Multi_ee
import numpy as np

# Pose [x, y, z, yaw, pitch, roll] at an arbitrary point along the backbone
pose = FK_Multi(q_motor, 0.6667)   # (6,) array, s=2/3 -> tip of segment 2

# End-effector pose (s=1), convenience wrapper
pose_ee = FK_Multi_ee(q_motor)     # (6,) array
```

---

## Testing

```bash
conda activate finger
cd Kinematics
python3 - <<'EOF'
import numpy as np
from FK_Multi import FK_Multi_ee
from JacobiansMulti import REST_LENGTH

q_home = np.zeros(9)
pose = FK_Multi_ee(q_home)
print("EE pose at home:", pose)
print("Expected z ≈", 3 * REST_LENGTH, "m")
EOF
```

Expected output: `EE pose at home: [0.  0.  0.435  0. -0.  0.]` (x, y, z, yaw, pitch, roll)

---

## Important Notes

- **Singularity**: the PCC model has a singularity at `D = 0` (straight segment).
  The code guards against this with an additive epsilon inside the square root:
  `D = √(Dx² + Dy² + 1e-12)`, applied consistently in `FK_Multi.py` and
  `JacobiansMulti.py`.
- **Frame convention**: z-axis points along the arm axis; x-y are lateral.
- **Units**: all internal computations in **meters and radians**. Motor angles from
  the Dynamixel driver are received in radians.

---

## References

- Webster & Jones, "Design and Kinematic Modeling of Constant Curvature Continuum Robots,"
  *Int. J. Robotics Research*, 2010.
- Renda et al., "Discrete Cosserat Approach for Multi-Section Soft Manipulator Dynamics,"
  *IEEE Trans. Robotics*, 2018.
