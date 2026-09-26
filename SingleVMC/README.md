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
| `SingleVMC.py` | Hardware run: task-space VMC on the real 1-segment arm; on exit, saves a position/torque summary plot to `plots/` if `SAVE_PLOT = True` (it ships `False`) |
| `SingleVMCSimulation.py` | Offline simulation for VMC tuning (no hardware required); only calls `plt.show()`, does not save to disk |
| `plots/` | Per-run snapshots saved by `SingleVMC.py` hardware runs — no analysis notebook combines them |

## Running simulation

```bash
conda activate finger
cd SingleVMC
python3 SingleVMCSimulation.py
```

## Running on hardware

```bash
conda activate finger
cd SingleVMC
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
python3 SingleVMC.py
```
