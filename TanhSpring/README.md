# TanhSpring Experiments

Experiments with tanh (force-limited) spring and damper virtual elements.

The tanh spring provides **bounded interaction forces** while preserving the restoring behavior of a linear spring for small displacements.  Force saturation occurs at `f_max`.

## Experiments

| Subfolder | Description |
|---|---|
| `outputs/tanh_spring/` | Sequential jump tests: compares tanh spring tracking vs. linear spring |

## Scripts

| File | Purpose |
|---|---|
| `GUI_MultiVMC_Config.py` | VMC config (tanh spring + damper) |
| `Sequential_jump_tests.py` | Runs a sequence of reference position jumps |
| `publish_pos.sh` | Shell helper to publish a fixed reference position |

## Notebook

`plot_tanh_spring.ipynb` — torque saturation and trajectory tracking comparison.

## Running

```bash
conda activate finger
cd TanhSpring
python3 ../VMC/GUI_MultiVMC_Torque.py &
python3 ../VMC/GUI_MultiVMC.py
python3 Sequential_jump_tests.py
```
