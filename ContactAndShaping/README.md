# ContactAndShaping Experiments

Experiments involving physical contact and body shaping — multiple virtual elements acting simultaneously on different points along the arm.

## Experiments

| Subfolder | Description |
|---|---|
| `outputs/sponge/` | End-effector pressed against a sponge; sweep contact stiffness k. Only raw `.npz` + auto-generated `GUI_MultiVMC_Torque.py` per-run snapshot PNGs exist here — **no analysis notebook in the repo processes this data into a comparison figure**. |
| `outputs/surface_tracking/` | Arm slides along a rigid surface with a constrained spring |
| `outputs/body_shaping/` | Whole-body shaping: spring elements attached at multiple arc fractions. Same situation as `sponge/` above — only the raw `.npz` + its auto-generated diagnostic snapshot (`whole_body_shaping.png`, position/torque time series) exist, **no analysis notebook**. `old/` holds a discarded earlier run (`whole_body_shaping_bad`). |
| `outputs/three_spring_vs_one/` | Comparison: 1 end-effector spring vs. 3 distributed springs for posture |

## Scripts

| File | Purpose |
|---|---|
| `GUI_MultiVMC_Config_surface.py` | VMC config for surface-tracking experiment |
| `GUI_MultiVMC_Config_three_springs.py` | VMC config for three-spring comparison |

## Notebooks

| Notebook | Generates |
|---|---|
| `plot_three_spring_vs_one.ipynb` | Posture error comparison: 1 vs. 3 springs |
| `plot_surface_tracking.ipynb` | XY/X-position trajectory comparisons, soft-vs-stiff energy comparison, and per-k Y-position tracking across the `outputs/surface_tracking/` k-sweep |

## Running

`GUI_MultiVMC.py`/`GUI_MultiVMC_Torque.py` always import a module literally named
`GUI_MultiVMC_Config` from the current directory. Since this folder holds two
named variants (no single unambiguous default), copy the one you want into
place before launching:

```bash
conda activate finger
cd ContactAndShaping

# Surface tracking:
cp GUI_MultiVMC_Config_surface.py GUI_MultiVMC_Config.py
python3 ../VMC/GUI_MultiVMC_Torque.py &
python3 ../VMC/GUI_MultiVMC.py

# Three-spring vs. one comparison (separate run):
cp GUI_MultiVMC_Config_three_springs.py GUI_MultiVMC_Config.py
python3 ../VMC/GUI_MultiVMC_Torque.py &
python3 ../VMC/GUI_MultiVMC.py
```
