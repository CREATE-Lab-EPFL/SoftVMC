# TanhSpring — Force-bounded interaction

The tanh spring, `f = F_max tanh(K e / F_max)`, behaves like a linear spring of
stiffness `K` for small displacements and **bounds the interaction force** at
`F_max` for large ones. Here it is compared on the hardware against a linear
spring of the same `K`.

## Experiments

| Subfolder | Description |
|---|---|
| `outputs/tanh_spring/` | Sequential jump tests: the tip's reference is stepped out to targets from −5 to −50 cm (10 test windows), once with a tanh spring and once with a linear spring of the same K, while a load cell measures the shear force on a string wrapped round the tip. Needs `LoadCell/arduino_force.py` running |

## Scripts

| File | Purpose |
|---|---|
| `GUI_MultiVMC_Config.py` | VMC config (tanh spring + damper) |
| `Sequential_jump_tests.py` | Runs a sequence of reference position jumps |
| `publish_pos.sh` | Shell helper to publish a fixed reference position |

## Notebook

`plot_tanh_spring.ipynb` — the tanh spring's force law and the measured evidence that it saturates. It writes, in `outputs/tanh_spring/` (each `.pdf` + `.svg`, and `.png`):

- `virtual_vs_real_tanh_fmax133`: the controller's virtual force against the load cell's measured shear force for the tanh run, on one axis. The measured force is also drawn divided by η = 0.55, a fixed factor set in the notebook, so the two shapes can be compared;
- `measured_force_vs_displacement` — measured shear force vs. commanded displacement, tanh against linear on one axis. At −50 cm the tanh spring's force has plateaued at ≈ 0.71 N while the linear spring's is still rising (≈ 0.94 N).

Unlike the rest of the repo's figures, the notebook uses **mathtext with `cmr10`** (`usetex` off), so it needs no LaTeX install.

**Config vs. recorded runs.** The shipped `GUI_MultiVMC_Config.py` has `TanhSpring(15.0, 1.5)` (K = 15 N/m, F_max = 1.5 N); the notebook describes the recorded runs as K = 10 N/m, F_max = 1.33 N (hence `fmax133` in the file name). Set those in the config, not the shipped values, to reproduce them.

## Run names

What is recorded here, and the name `--run` needs to write each one:

| run | `--run` names |
|---|---|
| `outputs/tanh_spring/` (default config) | `linear_k10` · `tanh_k10` |

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
cd TanhSpring
python3 ../VMC/GUI_MultiVMC_Torque.py &
python3 ../VMC/GUI_MultiVMC.py
python3 Sequential_jump_tests.py
```
