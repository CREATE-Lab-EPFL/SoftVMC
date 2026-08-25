# VMC — Virtual Model Control

Core VMC implementation and ROS 2 interface for the soft continuum arm.

See [VMC_DOCUMENTATION.md](VMC_DOCUMENTATION.md) for the full architecture description, running instructions, and data format.

## Key files

| File | Role |
|---|---|
| `MultiVMCTaskSpace.py` | `VirtualComponent` + `VMC_Multi_Modular` controller |
| `GUI_MultiVMC_Torque.py` | Main torque computation ROS 2 node |
| `GUI_MultiVMC.py` | tkinter GUI + publisher |
| `MultiGoHome.py` | Homing routine (run first). Also: `python3 MultiGoHome.py --read` prints the current absolute position (encoder ticks) of all 9 motors and exits — no ROS needed, talks to the Dynamixel bus directly. Use this to read raw motor state on demand, or to capture a new `HOME_POSITION`. |
| `MultiController.py` | Low-level PD motor controller |
| `MultiVMC_MovingGoal_Pub.py` | Scripted alternative to `GUI_MultiVMC.py`: ramps `/VC_ref_pos` from `DEFAULT_START` to `DEFAULT_GOALS` (and back, if `LOOP_BACK=True`) using whichever config is active in the caller's directory — no dragging required. Used directly by `ConnectivityTest/`. |
| `MultiVMC_publish_0.py` | Publishes a fixed `DEFAULT_START` so the arm holds home pose while testing external perturbations |

## Common flags

Both apply to every script that loads an experiment config
(`GUI_MultiVMC_Torque.py`, `GUI_MultiVMC.py`, `MultiVMC_MovingGoal_Pub.py`,
`MultiVMC_publish_0.py`; `--gravity-dir` to the torque nodes only):

| Flag | Effect |
|---|---|
| `--config NAME` | load `GUI_MultiVMC_Config_NAME.py` from the launch directory instead of `GUI_MultiVMC_Config.py`. `_moving` and `moving` are equivalent; a bad name lists what is available. Replaces copying a variant into place. |
| `--gravity-dir DIR` | direction gravity pulls in the robot frame — `+x/-x/+y/-y/+z/-z`. Defaults to the config's `GRAVITY_DIR`, or `-z` (upright) if it defines none. `--gravity-dir=-y` and `--gravity-dir -y` both parse. |

The torque node logs both on startup; check those two lines before trusting a
run:

```
[INFO] config: GUI_MultiVMC_Config_button
[INFO] gravity compensation: gravity pulls along -y in the robot frame
```

## Running

```bash
# From the experiment directory (so GUI_MultiVMC_Config.py is found):
cd LinearSpring        # or TanhSpring, etc.
python3 ../VMC/GUI_MultiVMC_Torque.py &
python3 ../VMC/GUI_MultiVMC.py
```

See [../start_arm.sh](../start_arm.sh) for the full startup sequence.
