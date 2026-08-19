# ButtonPressing Experiments

Covers both remaining planned experiments. Neither has been run yet — no data exists in this folder.

## Fig 9 — Exploiting Contact for Button Pressing

Two virtual components summed at the arm tip (`s_param=1.0`), both targeting the
same `button_pos`, `tau = J1^T@F1 + J2^T@F2`:

| Component | Element | Role |
|---|---|---|
| `press` | `TanhSpring(stiffness, max_force)` | Drives the tip into the button. Saturates at `max_force`, so that value *is* the calibrated full-contact force target — bounded/safe regardless of the component below. |
| `gauss_repel` | `GaussianSpring(strength, sigma)` | Repels the tip away from `button_pos`. Sweeping `strength` from 0 up to a value that fully cancels the approach trades off against `press`, producing the force family for the figure. |

Sweeping `gauss_repel`'s strength live (0 → high) produces the run family:
**no-contact → middle → middle → full-contact**, i.e. one continuous knob
instead of separate scenario types. Contact force is read from the load cell's
**shear** channel (`/force_shear`) — the arm is mounted horizontally against the
load-cell structure, so shear, not normal force, is the meaningful signal here.
Both components' goal markers must stay coincident at `button_pos` in the GUI —
only the `gauss_repel` strength slider should move between runs.

The visible "button" itself is cosmetic (for photos) — the actual measurement
structure is the external load cell the arm moves onto and presses against.

## Fig 10 — Demo with Person

Qualitative photo/video demo only (paper's softer "C. Demo" section, medical
framing) — arm avoids or gently touches a person. No quantitative logging
planned; conservative `TAU_LIMIT` applies as the safety bound.

## Files

| File | Purpose |
|---|---|
| `GUI_MultiVMC_Config.py` | VMC config: `press` + `gauss_repel` components described above |
| `button_pressing.py` | ROS 2 scenario driver stub (`--scenario avoid\|exploit\|minimize`); its `_tick()` is unimplemented — **not currently used**. The GUI workflow below is the actual/authoritative path, matching every other experiment folder in this repo. |

## Running

```bash
conda activate finger
cd ButtonPressing

# Terminal 1 -- hardware
bash ../start_arm.sh

# Terminal 2 -- torque node
python3 ../VMC/GUI_MultiVMC_Torque.py &

# Terminal 3 -- GUI
python3 ../VMC/GUI_MultiVMC.py
```

Both spring parameters start conservative/untested (`press` stiffness=20, max_force=1.0;
`gauss_repel` strength=0.0) — live-tune via the GUI sliders while watching
`/force_shear` before trusting any value. `GUI_MultiVMC_Torque.py` only
auto-saves once, on exit, so each of the 4 sweep conditions needs its own
process run (restart the torque node between conditions) rather than one
continuous session.

## Data

Saved `.npz` files go to `outputs/button_pressing/` (create this directory
before running — it does not exist yet). The `force` field is the raw load-cell
reading in **grams**; multiply by `9.80665e-3` to get Newtons (see
`LinearSpring/plot_k_saturation.ipynb` for the verified precedent).
