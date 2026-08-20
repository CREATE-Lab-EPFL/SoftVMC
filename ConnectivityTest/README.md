# ConnectivityTest

Pure sanity check: connect to the real arm and move it, using the same VMC
code as every real experiment in this repo, to confirm the hardware/software
chain actually works — **before** trusting it for a real experiment or a
demo. Nothing is ever saved here (`SAVE_PLOT`/`SAVE_DATA` are `False` in
`GUI_MultiVMC_Config.py`), so there's no `COLLECTED_DATA` flag to manage in
this folder.

The single component, its goal, `TAU_LIMIT`, and the slider ranges in
`GUI_MultiVMC_Config.py` are copied **exactly** from `VMC/GUI_MultiVMC_Config.py`'s
own `lin_seg3` component — the student's own proven-working reference setup,
not reinvented numbers. Those values (`k=90`, `c=15`, `TAU_LIMIT=2.1`) are
confirmed to already be correctly adapted for the current `kt=0.000354`
motor calibration (see the comment block at the top of the config file for
the git-history trail proving this). An earlier version of this file used
untested placeholder values and visibly oscillated on real hardware — that's
why it's pinned to the verified values now instead.

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
