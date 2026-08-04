# ConstrainedSpring Experiments

Experiments with the constrained ("cart") linear spring — a virtual spring that acts only along a specified direction (normal to a virtual surface), leaving the orthogonal directions free.

This element implements **sliding contact**: the arm can move freely parallel to the surface while being controlled in the normal direction.

## Experiments

| Subfolder | Description |
|---|---|
| `outputs/constrained_spring/` | End-effector constrained to slide along a virtual plane |

## Scripts

| File | Purpose |
|---|---|
| `GUI_MultiVMC_Config.py` | VMC config (constrained spring + damper) |
| `publish_pos.sh` | Shell helper for publishing reference positions |

## Notebook

`plot_constrained_spring.ipynb` — position traces and normal/tangential force decomposition.

## Running

```bash
conda activate finger
cd ConstrainedSpring
python3 ../VMC/GUI_MultiVMC_Torque.py &
python3 ../VMC/GUI_MultiVMC.py
```
