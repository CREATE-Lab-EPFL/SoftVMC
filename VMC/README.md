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

## Running

```bash
# From the experiment directory (so GUI_MultiVMC_Config.py is found):
cd LinearSpring        # or TanhSpring, etc.
python3 ../VMC/GUI_MultiVMC_Torque.py &
python3 ../VMC/GUI_MultiVMC.py
```

See [../start_arm.sh](../start_arm.sh) for the full startup sequence.
