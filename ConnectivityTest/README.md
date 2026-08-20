# ConnectivityTest

Pure sanity check: connect to the real arm and move it, using the same VMC
code and the same "moving target" mechanism as every real experiment in this
repo, to confirm the hardware/software chain actually works — **before**
trusting it for a real experiment or a demo. Nothing is ever saved here
(`SAVE_PLOT`/`SAVE_DATA` are `False` in `GUI_MultiVMC_Config.py`), so there's
no `COLLECTED_DATA` flag to manage in this folder.

This doesn't reimplement anything — it's the standard config-swap pattern
every experiment folder uses (`GUI_MultiVMC_Config.py` supplies one
`LinearSpring` component with a small, safe goal), pointed at the existing
`VMC/MultiVMC_MovingGoal_Pub.py`, which ramps the reference from
`DEFAULT_START` to the goal and back (`LOOP_BACK = True`) so you can just
watch it track continuously.

## Running

```bash
conda activate finger
cd ConnectivityTest

# Terminal 1 -- hardware
bash ../start_arm.sh

# Terminal 2 -- torque node
python3 ../VMC/GUI_MultiVMC_Torque.py

# Terminal 3 -- moving-target publisher (same mechanism used everywhere else,
# e.g. GaussianElement/moving_gauss_test.py, ButtonPressing's driver scripts)
python3 ../VMC/MultiVMC_MovingGoal_Pub.py
```

Watch the arm oscillate a few centimeters toward `TEST_GOAL` and back. If it
tracks smoothly with no errors in either terminal, the connection, kinematics,
and control loop are working. Ctrl+C both terminals when done — nothing gets
written to disk either way.

Prefer to drive it by hand instead? `python3 ../VMC/GUI_MultiVMC.py` works
too, in place of Terminal 3 — same config, same underlying controller, just
an interactive drag instead of a scripted motion.

`TEST_GOAL` and the spring/damper values in `GUI_MultiVMC_Config.py` are
arbitrary and safe by default — edit them freely, they aren't measured
against anything physical.
