# ConnectivityTest

Pure sanity check: connect to the real arm and move it, using the same VMC
code as every real experiment in this repo, to confirm the hardware/software
chain actually works — **before** trusting it for a real experiment or a
demo. Nothing is ever saved here (`SAVE_PLOT`/`SAVE_DATA` are `False` in
`GUI_MultiVMC_Config.py`), so there's no `COLLECTED_DATA` flag to manage in
this folder.

`GUI_MultiVMC_Config.py` defines a single X-constrained spring at the arm tip
with intentionally conservative gains (`k=10`, `c=1`) and a low `TAU_LIMIT` —
enough authority to confirm the arm responds, without needing much. Raise
them live via the GUI sliders if you want more, and keep `TAU_LIMIT` low
while doing so.

## Gravity direction

`GRAVITY_DIR = "-y"` in `GUI_MultiVMC_Config.py`: the default here assumes the
arm rotated onto its side, the mounting used for the `ButtonPressing` runs, so
gravity pulls along **-y** in the robot frame rather than the upright **-z**.
The torque node reads it and orients its gravity compensation to match.

Change it for a single run straight from the terminal — no edit needed:

```bash
python3 ../VMC/GUI_MultiVMC_Torque.py --gravity-dir -z    # arm standing upright
```

Accepts `+x/-x/+y/-y/+z/-z`, and `--gravity-dir=-z` works too. The node logs the
direction it started with, so check that line before trusting a run. If the arm
sags or drifts consistently to one side, this is the first thing to check.

## Running

Primary path — interactive GUI:

```bash
conda activate finger
cd ConnectivityTest

# Terminal 1 -- hardware
bash ../start_arm.sh

# Terminal 2 -- torque node
python3 ../VMC/GUI_MultiVMC_Torque.py

# Terminal 3 -- GUI
python3 ../VMC/GUI_MultiVMC.py
```

Drag the goal around in the GUI and confirm the arm tracks it smoothly with
no errors in either terminal. Ctrl+C both terminals when done — nothing gets
written to disk either way.

Alternative — scripted motion instead of dragging by hand (same mechanism
used elsewhere in this repo, e.g. `GaussianElement/moving_gauss_test.py`,
`ButtonPressing`'s driver scripts): replace Terminal 3 above with
`python3 ../VMC/MultiVMC_MovingGoal_Pub.py`, which ramps the reference from
`DEFAULT_START` to the goal and back (`LOOP_BACK = True`) continuously, no
dragging required.
