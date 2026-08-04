# GaussianElement Experiments

Experiments with the Gaussian repulsive field virtual element.  The Gaussian field produces a spatially-concentrated force that pushes the attachment point away from a specified center:

```
f(x) = k · exp(−‖x − c‖² / 2σ²) · (x − c) / ‖x − c‖
```

Key properties: zero force at large distance, peak force at distance σ from the center.

## Experiments

| Subfolder | Description |
|---|---|
| `outputs/fixed_gauss/` | Gaussian center is fixed in space; arm reacts to repulsion |
| `outputs/moving_gauss/` | Gaussian center moves along a trajectory (dynamic obstacle avoidance) |

## Scripts

| File | Purpose |
|---|---|
| `GUI_MultiVMC_Config_fixed.py` | VMC config for fixed-obstacle experiment |
| `GUI_MultiVMC_Config_moving.py` | VMC config for moving-obstacle experiment |
| `gauss_test.py` | Automated obstacle motion publisher |
| `moving_gauss_test.py` | Moving-obstacle trajectory publisher |

## Notebooks

| Notebook | Generates |
|---|---|
| `plot_gauss_fixed_gauss.ipynb` | Force field and arm trajectory for fixed obstacle |
| `plot_gauss_moving_gauss.ipynb` | Dynamic avoidance trajectory |

## Running

```bash
conda activate finger
cd GaussianElement

# Fixed obstacle:
python3 ../VMC/GUI_MultiVMC_Torque.py &
python3 ../VMC/GUI_MultiVMC.py

# Moving obstacle (additional terminal):
python3 moving_gauss_test.py
```
