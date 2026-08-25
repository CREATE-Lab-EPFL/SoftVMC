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
| `moving_gauss_test.py` | Moving-obstacle trajectory publisher (no separate runner exists for the fixed-obstacle case — it uses `GUI_MultiVMC.py` directly) |

## Notebooks

| Notebook | Generates |
|---|---|
| `plot_gauss_fixed_gauss.ipynb` | Force field and arm trajectory for fixed obstacle |
| `plot_gauss_moving_gauss.ipynb` | Dynamic avoidance trajectory |

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
`GUI_MultiVMC_Config` from the current directory. Since this folder holds two
named variants (no single unambiguous default), copy the one you want into
place before launching — the same pattern used to switch experiments in the
original data collection:

```bash
conda activate finger
cd GaussianElement

# Fixed obstacle -- every command names the same variant:
python3 ../VMC/GUI_MultiVMC_Torque.py --config _fixed &
python3 ../VMC/GUI_MultiVMC.py --config _fixed

# Moving obstacle (separate run):
python3 ../VMC/GUI_MultiVMC_Torque.py --config _moving &
python3 ../VMC/GUI_MultiVMC.py --config _moving
python3 moving_gauss_test.py _moving   # additional terminal, drives the moving center
```
