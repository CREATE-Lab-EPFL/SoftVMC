# Jacobians for Soft Structures

Symbolic Jacobians for tendon-driven soft robots (PCC kinematics). Computed once via SymPy, compiled to NumPy with `lambdify`.

## Model

Each section: **q = [Dx, Dy, Dl]** [mm] — lateral curvatures + elongation.
Tendons at angles `delta`: `ΔLᵢ = Dl - Dx·cos(δᵢ) - Dy·sin(δᵢ)`, i.e. **ΔL = A @ q** (linear).

| File | Sections | Functions |
|------|----------|-----------|
| `SINGLE_jacobians.py` | 1 | `q2tendon`, `q2coordinates`, `tendon2pose` |
| `MULTI_jacobians.py`  | 3 | `q2tendon`, `q2coordinates`, `q2xyz`, `tendon2pose` |

## Functions

- **`q2tendon(delta)`** → `(Dx,Dy,Dl)` → **(3,3)**: ∂(ΔL)/∂q
- **`q2coordinates()`** → `(Dx,Dy,Dl)` → **(6,3)**: ∂(pose)/∂q, pose = [x,y,z,yaw,pitch,roll]
- **`tendon2pose(delta)`** → `(ΔL₁,ΔL₂,ΔL₃)` → **(6,3)**: ∂(pose)/∂(ΔL), end-to-end

`tendon2pose` inverts A numerically from `delta`, substitutes q = A⁻¹ΔL into the kinematics symbolically, then differentiates — no runtime inversion, `cse=True`.

## Usage

```python
from SINGLE_jacobians import q2tendon, q2coordinates, tendon2pose

J_qt = q2tendon()       # d(ΔL)/d(q),    shape (3,3)
J_qp = q2coordinates()  # d(pose)/d(q),  shape (6,3)
J_tp = tendon2pose()    # d(pose)/d(ΔL), shape (6,3)  ← end-to-end

# VMC: virtual tip force -> tendon forces
f_tendon = J_tp(L1, L2, L3).T @ F_tip
```

Run `python3 SINGLE_jacobians.py` or `python3 MULTI_jacobians.py` to build and validate all Jacobians.
