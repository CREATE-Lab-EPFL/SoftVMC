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
| `MultiCheckBus.py` | Read-only Dynamixel bus health check (no ROS, no motion): run it when the driver logs `SyncRead failed` or homing times out |
| `MultiGravityComp.py` | `MultiGravComp`: gravity compensation for the 3 segments (lumped 40 g each), oriented by `GRAVITY_DIR`; used by the torque node |
| `MultiController.py` | Low-level PD motor controller, and the `CONTROL_FREQUENCY` constant (330 Hz) |
| `GUI_MultiVMC_Config.py` | Default/template config (`TAU_LIMIT = 0.25`); each experiment folder has its own |
| `read_npz.py` | CLI viewer for a saved `.npz`: re-plot it, crop a time window, or write a cropped `.npz` |
| `MultiVMC_MovingGoal_Pub.py` | Scripted alternative to `GUI_MultiVMC.py`: ramps `/VC_ref_pos` from `DEFAULT_START` to `DEFAULT_GOALS` (and back, if `LOOP_BACK=True`) using whichever config is active in the caller's directory — no dragging required. Used directly by `ConnectivityTest/`. |
| `MultiVMC_publish_0.py` | Publishes a fixed `DEFAULT_START` so the arm holds home pose while testing external perturbations |

## Common flags

Both apply to every script that loads an experiment config
(`GUI_MultiVMC_Torque.py`, `GUI_MultiVMC.py`, `MultiVMC_MovingGoal_Pub.py`,
`MultiVMC_publish_0.py`; `--gravity-dir` to the torque nodes only):

| Flag | Effect |
|---|---|
| `--config NAME` | load `GUI_MultiVMC_Config_NAME.py` from the launch directory instead of `GUI_MultiVMC_Config.py`. `_moving` and `moving` are equivalent; a bad name lists what is available. Replaces copying a variant into place. |
| `--run NAME` | (torque node) write the run straight to `outputs/<OUTPUT_FOLDER>/NAME.{npz,png}`, overwriting a previous run of that name. Without it, output is a timestamped `experiments/z_<stamp>` pair. The GUI also reads it, and refuses to start without it when the config sets `REQUIRE_RUN_NAME`. |
| `--gravity-dir DIR` | direction gravity pulls in the robot frame — `+x/-x/+y/-y/+z/-z`. Defaults to the config's `GRAVITY_DIR`, or `-z` (upright) if it defines none. `--gravity-dir=-y` and `--gravity-dir -y` both parse. |

The torque node logs both on startup; check those two lines before trusting a
run:

```
[INFO] config: GUI_MultiVMC_Config_button
[INFO] gravity compensation: gravity pulls along -y in the robot frame
```

## Recorded data and gravity

- **`COLLECTED_DATA`**: every experiment config sets it. `True` means that
  config's data is already recorded, so a rerun (demo, sanity check) never
  writes anything. It is `True` everywhere except three variants that ship
  `False`, where re-using a run name overwrites the file (`ls outputs/<folder>/`
  first): `ButtonPressing/`'s `_card_castle`, `ContactAndShaping/`'s
  `_bending` (a new name per case) and `GaussianElement/`'s `_moving`.
  `ConnectivityTest/` never saves anything.
- **`GRAVITY_DIR`**: the direction gravity pulls in the robot frame, read from
  each config (`"-z"`, upright, if a config omits it). `ButtonPressing/`,
  `ConnectivityTest/` and `ContactAndShaping`'s `_bending` set `"-y"` (arm
  mounted on its side); every other config was recorded upright. If the
  mounting does not match the config, pass `--gravity-dir`. Otherwise gravity
  compensation pushes along an axis gravity no longer acts on.

## Hardware notes

- `start_arm.sh` homes the motors, sets the USB latency timer to 1 ms, and
  starts `dynamixel_node` in torque mode with the current motor calibration
  (`use_kt:=true`, `kt:=0.000354`).
- `python3 VMC/MultiGoHome.py --read` prints the absolute encoder position of
  the 9 motors and exits. It talks to the Dynamixel bus directly, without ROS.

## Running

```bash
# From the experiment directory (so GUI_MultiVMC_Config.py is found):
cd LinearSpring        # or TanhSpring, etc.
python3 ../VMC/GUI_MultiVMC_Torque.py &
python3 ../VMC/GUI_MultiVMC.py
```

See [../start_arm.sh](../start_arm.sh) for the full startup sequence.
