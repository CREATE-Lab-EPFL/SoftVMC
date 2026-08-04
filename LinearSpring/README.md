# LinearSpring Experiments

Experiments with linear spring and linear damper virtual elements.

## Experiments

| Subfolder | Description | Paper figure |
|---|---|---|
| `outputs/k_saturation/` | Sweep stiffness k over [5, 10, 25, 50, 100, 200] N/m; records position and torque saturation | Fig. 4 |
| `outputs/disturbance_rejection/` | Step disturbance applied to end-effector; measures rejection bandwidth | Fig. 5 |

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

## Running

```bash
conda activate finger
cd LinearSpring
python3 ../VMC/GUI_MultiVMC_Torque.py &
python3 ../VMC/GUI_MultiVMC.py
# In a third terminal:
python3 VMC_test_k.py   # automated sweep
```
