# ContactAndShaping Experiments

Experiments involving physical contact and body shaping — multiple virtual elements acting simultaneously on different points along the arm.

## Experiments

| Subfolder | Description |
|---|---|
| `outputs/sponge/` | End-effector pressed against a sponge; sweep contact stiffness k. Only raw `.npz` + auto-generated `GUI_MultiVMC_Torque.py` per-run snapshot PNGs exist here — **no analysis notebook in the repo processes this data into a comparison figure**. |
| `outputs/surface_tracking/` | Arm slides along a rigid surface with a constrained spring |
| `outputs/body_shaping/` | Whole-body shaping: a Gaussian on segment 1 (`gauss_seg1`) and a linear spring on the tip (`lin_seg3`), both goals/strengths tuned live from the GUI. Two configs, identical except the tip goal's X sign (`_linear_right` / `_linear_left`). Only each side's settled final pose is analyzed (`plot_full_body_shaping.ipynb`), not a time series. `old/whole_body_shaping_bad.*` is an earlier discarded run from before these configs existed. |
| `outputs/three_spring_vs_one/` | Comparison: 1 end-effector spring vs. 3 distributed springs for posture |

## Scripts

| File | Purpose |
|---|---|
| `GUI_MultiVMC_Config_surface.py` | VMC config for surface-tracking experiment |
| `GUI_MultiVMC_Config_three_springs.py` | VMC config for three-spring comparison |
| `GUI_MultiVMC_Config_linear_right.py` | VMC config for whole-body shaping: `gauss_seg1` (segment 1) + `lin_seg3` (tip, goal at +X), both GUI-tunable |
| `GUI_MultiVMC_Config_linear_left.py` | Same as `_linear_right`, tip goal at -X instead |

## Notebooks

| Notebook | Generates |
|---|---|
| `plot_three_spring_vs_one.ipynb` | Posture error comparison: 1 vs. 3 springs |
| `plot_surface_tracking.ipynb` | XY/X-position trajectory comparisons, soft-vs-stiff energy comparison, and per-k Y-position tracking across the `outputs/surface_tracking/` k-sweep |
| `plot_full_body_shaping.ipynb` | One backbone figure (top + side view) overlaying both sides' settled final pose from `outputs/body_shaping/`, from the motor encoders (FK on `q_motor`) — not the virtual model's internal reference |

## Run names

What is recorded here, and the name `--run` needs to write each one:

| run | `--run` names |
|---|---|
| `outputs/surface_tracking/` (`--config _surface`) | `k130` · `k20` · `k20_new` · `k25` · `k25_old` · `k30` · `k30_new` · `k5` · `k70` · `k70_new` · `k8` |
| `outputs/three_spring_vs_one/` (`--config _three_springs`) | `filtered` · `raw` · `raw2` · `raw_with_gauss` |
| `outputs/body_shaping/` (`--config _linear_right`) | `right` — only one run is kept; re-recording overwrites it |
| `outputs/body_shaping/` (`--config _linear_left`) | `left` — only one run is kept; re-recording overwrites it |

`outputs/sponge/` holds earlier data that no current config points at, so
`--run` cannot route there; it is kept as a record. `outputs/body_shaping/`
now has two configs (`_linear_right`/`_linear_left`) and a notebook, but the
pre-existing `old/whole_body_shaping_bad.*` predates both and is likewise
just a kept record.

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --run <name>
```

That writes `outputs/<folder>/<name>.{npz,png}` on Ctrl+C, overwriting the run
of that name — the folder comes from `OUTPUT_FOLDER` in the config. `_surface`
and `_three_springs` ship `COLLECTED_DATA = True`, so nothing is written at
all until you set it `False`; that flag is what keeps their recorded data
safe. `_linear_right`/`_linear_left` ship `COLLECTED_DATA = False` instead,
since there is no data to protect yet — flip each to `True` once its one run
is recorded. Names starting `old_` are discarded runs.

## Gravity direction

`_surface` and `_three_springs` don't set `GRAVITY_DIR`, so the torque node
compensates for an **upright** arm (`-z`) — which is how those runs were
recorded. The arm is currently mounted **on its side** for `ButtonPressing/`.
To reproduce that data, remount it upright; if you run it on the side anyway,
pass `--gravity-dir -y` or the compensation pushes along an axis gravity no
longer acts on, and expect the numbers to differ from the recorded runs
either way, since the arm's own weight loads it differently.

`_linear_right`/`_linear_left` are the exception: both set `GRAVITY_DIR = "-y"`
directly, since they were written for the arm's current side-mounted state —
no `--gravity-dir` flag needed for them.

## Running

`GUI_MultiVMC.py`/`GUI_MultiVMC_Torque.py` always import a module literally named
`GUI_MultiVMC_Config` from the current directory. Since this folder holds two
named variants (no single unambiguous default), copy the one you want into
place before launching:

```bash
conda activate finger
cd ContactAndShaping

# Surface tracking -- every command names the same variant:
python3 ../VMC/GUI_MultiVMC_Torque.py --config _surface &
python3 ../VMC/GUI_MultiVMC.py --config _surface

# Three-spring vs. one comparison (separate run):
python3 ../VMC/GUI_MultiVMC_Torque.py --config _three_springs &
python3 ../VMC/GUI_MultiVMC.py --config _three_springs

# Whole-body shaping -- set COLLECTED_DATA = False in
# GUI_MultiVMC_Config_fullbody.py first, tune the pose live in the GUI,
# then Ctrl+C the torque node once settled (writes outputs/body_shaping/final.*):
python3 ../VMC/GUI_MultiVMC_Torque.py --config _fullbody --run final &
python3 ../VMC/GUI_MultiVMC.py --config _fullbody
```
