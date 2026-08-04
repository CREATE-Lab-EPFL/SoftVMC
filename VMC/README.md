# VMC — Virtual Model Control

Core VMC implementation and ROS 2 interface for the soft continuum arm.

See [VMC_DOCUMENTATION.md](VMC_DOCUMENTATION.md) for the full architecture description, running instructions, and data format.

## Key files

| File | Role |
|---|---|
| `MultiVMCTaskSpace.py` | `VirtualComponent` + `VMC_Multi_Modular` controller |
| `GUI_MultiVMC_Torque.py` | Main torque computation ROS 2 node |
| `GUI_MultiVMC.py` | tkinter GUI + publisher |
| `MultiGoHome.py` | Homing routine (run first) |
| `MultiController.py` | Low-level PD motor controller |

## Running

```bash
# From the experiment directory (so GUI_MultiVMC_Config.py is found):
cd LinearSpring        # or TanhSpring, etc.
python3 ../VMC/GUI_MultiVMC_Torque.py &
python3 ../VMC/GUI_MultiVMC.py
```

See [../start_arm.sh](../start_arm.sh) for the full startup sequence.
