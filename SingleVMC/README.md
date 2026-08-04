# SingleVMC

Single-segment VMC implementation — kept for reference and simulation testing.

> **Note:** Active experiments use the 3-segment MultiVMC system. SingleVMC is not updated alongside MultiVMC.

## Files

| File | Contents |
|---|---|
| `JacobiansSingle.py` | 1-segment PCC kinematics constants and Jacobians |
| `FK_Single.py` | Forward kinematics for a single segment |
| `SingleVMCTaskSpace.py` | VMC controller for 1-segment arm (3 motors) |
| `SingleController.py` | Motor-space PD controller |
| `SingleGoHome.py` | Homing routine |
| `SingleVMCSimulation.py` | Offline simulation for VMC tuning (no hardware required) |
| `plots/` | Saved simulation output plots |

## Running simulation

```bash
conda activate finger
cd SingleVMC
python3 SingleVMCSimulation.py
```
