# LinearSpring Experiments

Experiments with linear spring and linear damper virtual elements.

## Experiments

| Subfolder | Description |
|---|---|
| `outputs/k_saturation/` | Sweep spring stiffness k; records position and torque saturation |
| `outputs/disturbance_rejection/` | Step/impulse disturbance applied to end-effector; measures rejection (peak overshoot, settling time) |

`outputs/k_saturation/` was built up across several runs of `VMC_test_k.py`
with different `K_START`/`K_MAX`/`K_STEP` values edited in the script (the
saved filenames encode the original range, e.g. `k2_18.npz` spans 1.7–18.3 N/m in steps of
0.7, `k20_35.npz`, `k7_100_big.npz`). `plot_k_saturation.ipynb` combines the
saved runs. `outputs/disturbance_rejection/` holds a similar pattern:
`disturbance_k3_100.npz` (main sweep) plus `disturbance_k13_20.npz`
(re-run for two stiffness values).

## Scripts

| File | Purpose |
|---|---|
| `GUI_MultiVMC_Config.py` | VMC config for k-saturation experiment |
| `GUI_MultiVMC_Config_disturbance.py` | VMC config for disturbance-rejection experiment |
| `VMC_test_k.py` | Automated k-sweep runner (publishes reference positions) |

## Notebooks

| Notebook | Generates |
|---|---|
| `plot_k_saturation.ipynb` | Force vs. position curves by k; saves to `outputs/k_saturation/` |
| `plot_disturbance.ipynb` | Disturbance response plot |

## Gravity direction

This folder's config does not set `GRAVITY_DIR`, so the torque node compensates
for an **upright** arm (`-z`) — which is how these runs were recorded. The arm
is currently mounted **on its side** for `ButtonPressing/`. To reproduce this
folder's data, remount it upright. If you run it on the side anyway, pass
`--gravity-dir -y` or the compensation pushes along an axis gravity no longer
acts on — and expect the numbers to differ from the recorded runs either way,
since the arm's own weight loads it differently.

## Running

`GUI_MultiVMC.py`/`GUI_MultiVMC_Torque.py` always import a module literally named
`GUI_MultiVMC_Config` from the current directory — `GUI_MultiVMC_Config.py` (the
k-saturation config, already correctly named) is used as-is; for the disturbance
experiment, copy `GUI_MultiVMC_Config_disturbance.py` into place first:

```bash
conda activate finger
cd LinearSpring

# k-saturation sweep (uses GUI_MultiVMC_Config.py directly):
python3 ../VMC/GUI_MultiVMC_Torque.py &
python3 ../VMC/GUI_MultiVMC.py
# In a third terminal:
python3 VMC_test_k.py   # automated sweep

# Disturbance rejection (separate run) -- every command names the variant:
python3 ../VMC/GUI_MultiVMC_Torque.py --config _disturbance &
python3 ../VMC/GUI_MultiVMC.py --config _disturbance
```
