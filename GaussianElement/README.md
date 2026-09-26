# GaussianElement Experiments

Experiments with the Gaussian repulsive field virtual element.  The Gaussian field produces a spatially-concentrated force that pushes the attachment point away from a specified center:

```
f(x) = k · exp(−‖x − c‖² / 2σ²) · (x − c)
```

`x − c` is **not** normalised, so `k` is a stiffness in N/m (the config and the GUI call it `strength`; older notebook text calls it a "force amplitude, N" — it is not a force). Key properties: zero force at the center, peak force `k·σ·e^(−1/2)` at distance σ from it, and none to speak of beyond about 3σ. With σ = 0.06 m, a 240 N/m field peaks at 8.7 N.

## Experiments

| Subfolder | Description |
|---|---|
| `outputs/fixed_gauss/` | Gaussian center is fixed in space at x = −17 cm; the arm reaches past it and the field is switched off part-way through, so one run shows both the obstacle-avoiding and the free posture |
| `outputs/moving_gauss/` | Gaussian center circles the tip spring's target (radius 10 cm) while the tip spring holds its reference at the origin; three tip stiffnesses compared |

Each `outputs/<folder>/readme.md` holds that experiment's recording procedure.

## Scripts

| File | Purpose |
|---|---|
| `GUI_MultiVMC_Config_fixed.py` | VMC config for fixed-obstacle experiment |
| `GUI_MultiVMC_Config_moving.py` | VMC config for moving-obstacle experiment |
| `moving_gauss_test.py` | Moving-obstacle trajectory publisher (no separate runner exists for the fixed-obstacle case — it uses `GUI_MultiVMC.py` directly) |

## Notebooks

| Notebook | Generates |
|---|---|
| `plot_gauss_fixed_gauss.ipynb` | `fixed_gauss5_xz.{pdf,svg}` — the backbone posture (X–Z) averaged over the 5 s just before the field is removed against the 5 s just after, from `fixed_gauss5.npz` only |
| `plot_gauss_moving_gauss.ipynb` | `xy_circle_comparison.{pdf,png,svg}` — end-effector path against the moving Gaussian center, colored by time, for k = 2, 4 and 20 N/m; plus its `xy_circle_colorbar` and `xy_circle_legend` as separate files |

## Run names

What is recorded here, and the name `--run` needs to write each one. This folder
has **no plain `GUI_MultiVMC_Config.py`**, so every command needs `--config`:

| run | `--run` names |
|---|---|
| `outputs/fixed_gauss/` (`--config _fixed`) | `fixed_gauss1_no_q` · `fixed_gauss2_no_q` · `fixed_gauss3` · `fixed_gauss4` · `fixed_gauss5` (the notebook uses `fixed_gauss5`) |
| `outputs/moving_gauss/` (`--config _moving`) | `k2_moving_gauss` · `k4_moving_gauss` · `k20_moving_gauss` (all three are in the notebook) |

**What was logged** (`k_params`) differs from the names and from the procedures
in each `readme.md` — trust the log:

| run | logged tip stiffness [N/m] | logged Gaussian strength [N/m] |
|---|---|---|
| `fixed_gauss5` | 10 at first, then 33.3 | `gauss_seg1` 233.3, `gauss_seg2` 200, both then 0 (the procedure says 240 / 200) |
| `k2_moving_gauss` | **1.67** | 133.3 |
| `k4_moving_gauss` | 4.0 | 140 |
| `k20_moving_gauss` | **16.67** | 133.3 |

`k2` and `k20` log 5/6 of the number in their name and predate the run metadata
(14 fields, no `comp_spec` or `config_source`, so the config that made them is
not stored); `k4` has the full metadata and logs its name exactly. The cause of
the 5/6 is not documented. The moving-obstacle notebook labels its curves
"K = 2 / 4 / 20 N/m" from the **file names**, not from the log.

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --config _fixed --run <name>
python3 ../VMC/GUI_MultiVMC_Torque.py --config _moving --run <name>
```

That writes `outputs/<folder>/<name>.{npz,png}` on Ctrl+C, overwriting the run
of that name — the folder comes from `OUTPUT_FOLDER` in the config. **The two
configs differ:** `_fixed` ships `COLLECTED_DATA = True`, so nothing is written at
all until you set it `False`; `_moving` ships `COLLECTED_DATA = False`, so it
**does** write, and re-using `k2_moving_gauss`, `k4_moving_gauss` or
`k20_moving_gauss` overwrites a recorded run — `ls outputs/moving_gauss/` first.
Names starting `old_` are discarded runs.

## Gravity direction

This folder's config does not set `GRAVITY_DIR`, so the torque node compensates
for an **upright** arm (`-z`) — which is how these runs were recorded. The arm
is currently mounted **on its side** for `ButtonPressing/`. To reproduce this
folder's data, remount it upright. If you run it on the side anyway, pass
`--gravity-dir -y` or the compensation pushes along an axis gravity no longer
acts on — and expect the numbers to differ from the recorded runs either way,
since the arm's own weight loads it differently.

## Running

`GUI_MultiVMC.py`/`GUI_MultiVMC_Torque.py` load `GUI_MultiVMC_Config.py` from the
current directory unless told otherwise. This folder holds two named variants and
no plain default, so pass `--config` to pick one — nothing is copied. Name the
**same** variant in every command of a run:

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
