# ConstrainedSpring — The cart element

The constrained ("cart") linear spring, `ConstrainedLinearSpring`, acts only
along a chosen direction `n` (`f = K n nᵀ e`) and leaves the orthogonal
directions free. The tip is guided along `n` and slides freely across it, like
a cart on a rail.

## Experiments

| Subfolder | Description |
|---|---|
| `outputs/constrained_spring/` | The same goal sequence under the cart spring and under a linear spring (both K = 10 N/m) |

## Scripts

| File | Purpose |
|---|---|
| `GUI_MultiVMC_Config.py` | VMC config (constrained spring + damper) |
| `publish_pos.sh` | Shell helper for publishing reference positions |

## Notebook

`plot_constrained_spring.ipynb`: the end-effector's X–Y trajectory under the cart spring against the same goals under a plain linear spring (both K = 10 N/m), saved as `outputs/constrained_spring/xy_trajectory_markers.{pdf,svg}`. The goal is stepped along the 45° diagonal in X–Y, to (20, 20), (40, 40) and (60, 60) cm (`publish_pos.sh`; "Attachment 1–3" in the figure, marked 1 s after each step). The linear spring follows the diagonal, reaching (22.8, 21.0) cm. The cart spring responds only along its direction `n` (x + z, see below), so in X–Y it stays on the 0° line, reaching x = 27.1 cm with |y| ≤ 0.4 cm. The notebook plots positions only, not forces.

The shipped config uses `ConstrainedLinearSpring(10.0, n=[1, 0, 1])` (normalised): force acts only along the x + z direction. (`ConnectivityTest/` uses `n = [1, 0, 0]`.)

## Run names

What is recorded here, and the name `--run` needs to write each one:

| run | `--run` names |
|---|---|
| `outputs/constrained_spring/` (default config) | `constrained_k10` · `linear_k10` · `old_constrained_k10` · `old_linear_k10` |

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --run <name>
```

That writes `outputs/<folder>/<name>.{npz,png}` on Ctrl+C, overwriting the run
of that name — the folder comes from `OUTPUT_FOLDER` in the config. These configs
ship `COLLECTED_DATA = True`, so nothing is written at all until you set it
`False`; that flag is what keeps the recorded data safe. Names starting `old_`
are discarded runs.

## Gravity direction

These runs were recorded with the arm **upright**. The config does not set
`GRAVITY_DIR`, so the torque node compensates for an upright arm (`-z`). To
reproduce the data, mount the arm upright. On an arm mounted on its side, pass
`--gravity-dir -y`. Otherwise the compensation pushes along an axis gravity no
longer acts on. Expect the numbers to differ from the recorded runs either way,
since the arm's own weight loads it differently.

## Running

```bash
cd ConstrainedSpring
python3 ../VMC/GUI_MultiVMC_Torque.py &
python3 ../VMC/GUI_MultiVMC.py
```
