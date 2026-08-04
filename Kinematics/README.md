# Kinematics

Forward kinematics and analytical Jacobians for the 3-segment soft continuum arm using the Piecewise Constant Curvature (PCC) model.

## Files

| File | Contents |
|---|---|
| `JacobiansMulti.py` | Physical constants (`MOTOR_RADIUS`, `SECTION_RADIUS`, `REST_LENGTH`), A and A⁻¹ matrices per segment, Jacobians `pcc2q_motor`, `q_motor2pose`, `q_motor2pose_ee` |
| `FK_Multi.py` | Forward kinematics: `FK_Multi(q_motor, s_param)` (6-DOF pose at any arc-length fraction), `FK_Multi_ee(q_motor)` (end-effector pose, `s_param=1`) |
| `KINEMATICS_DOCUMENTATION.md` | Full mathematical derivation, API reference, testing |

## Quick check

```bash
conda activate finger
cd Kinematics
python3 -c "from FK_Multi import FK_Multi_ee; import numpy as np; print(FK_Multi_ee(np.zeros(9)))"
# Expected: [0.  0.  0.435  0. -0.  0.]  (x, y, z, yaw, pitch, roll)
```
