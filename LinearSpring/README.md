# LinearSpring Experiments

Experiments with linear spring and linear damper virtual elements.

## Experiments

| Subfolder | Description |
|---|---|
| `outputs/k_saturation/` | Sweep spring stiffness k; records position and torque saturation |
| `outputs/disturbance_rejection/` | Impulse disturbance applied to the end-effector for each k; the notebook reports each response's peak displacement and its time (settling time and ringing are not computed) |

`outputs/k_saturation/` was built up across several runs of `VMC_test_k.py`
with different `K_START`/`K_MAX`/`K_STEP` values edited in the script (the
saved filenames encode the nominal range). What the logs actually hold
(`k_params`, snap-back value excluded): `k2_18.npz` 26 values from 1.67 to 18.33
in steps of 0.67; `k20_35.npz` 12 values from 16.67 in steps of 1.67;
`k7_100_big.npz` 15 values from 6.67 to 100 in steps of 6.67 — not the script's
current 2.0 / 18.0 / 0.7. `k2_18` and `k20_35` also log a snap-back stiffness of
66.67 (the runner's current `K_RETURN` is 70); in `k7_100_big` it sits on the
grid. `plot_k_saturation.ipynb` combines the saved runs.
`outputs/disturbance_rejection/` holds a similar pattern:
`disturbance_k3_100.npz` (main sweep, 30 values from 3.33 to 100 in steps of
3.33, plus 0) and `disturbance_k13_20.npz` (re-run for two stiffness values,
13.33 and 16.67 — the name's "20" is nominal — plus 0 and a 76.67 snap-back).
All five predate the run metadata: 12–14 fields, no `comp_spec`, `config_source`
or `gravity_dir`. The procedure written in `plot_disturbance.ipynb` is in the GUI
units of the time (k incremented by 30, snap-back at k = 690), which are 9× the
logged values (3.33, 76.67).

## Scripts

| File | Purpose |
|---|---|
| `GUI_MultiVMC_Config.py` | VMC config for k-saturation experiment |
| `GUI_MultiVMC_Config_disturbance.py` | VMC config for disturbance-rejection experiment |
| `VMC_test_k.py` | Automated k-sweep runner (publishes reference positions) |

## Notebooks

| Notebook | Generates |
|---|---|
| `plot_k_saturation.ipynb` | `force_vs_pos_by_k`, `stiffness_vs_pos_by_k` and the two stacked as `force_and_stiffness_by_k` (each `.pdf` + `.svg`) in `outputs/k_saturation/`; a colorbar-by-k sequential palette |
| `plot_disturbance.ipynb` | `disturbance_response_by_k.{pdf,svg}` in `outputs/disturbance_rejection/` — position response to the disturbance for a subset of k (10 of 30 in the current figure) |

## Run names

What is recorded here, and the name `--run` needs to write each one:

| run | `--run` names |
|---|---|
| `outputs/k_saturation/` (default config) | `k20_35` · `k2_18` · `k7_100_big` |
| `outputs/disturbance_rejection/` (`--config _disturbance`) | `disturbance_k13_20` · `disturbance_k3_100` |

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --run <name>                       # k-saturation (plain config)
python3 ../VMC/GUI_MultiVMC_Torque.py --config _disturbance --run <name> # disturbance rejection
```

That writes `outputs/<folder>/<name>.{npz,png}` on Ctrl+C, overwriting the run
of that name — the folder comes from `OUTPUT_FOLDER` in the config. Both configs
ship `COLLECTED_DATA = True`, so nothing is written at all until you set it
`False`; that flag is what keeps the recorded data safe. Names starting `old_`
are discarded runs.

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
current directory unless told otherwise — that is the k-saturation config and is
used as-is. For the disturbance experiment pass `--config _disturbance` (nothing
is copied); name the same variant in every command of a run:

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
