# ButtonPressing Experiments

Button-pressing experiments demonstrating three distinct interaction strategies:

| Scenario | VMC element | Behavior |
|---|---|---|
| **avoid** | Gaussian repulsion | Arm stays away from button; obstacle avoidance |
| **exploit** | Linear spring | Arm presses into button with calibrated force |
| **minimize** | Tanh spring | Light contact with bounded interaction force |

## Files

| File | Purpose |
|---|---|
| `GUI_MultiVMC_Config.py` | VMC config (currently configured for *avoid*; edit to switch scenario) |
| `button_pressing.py` | ROS 2 scenario driver; `--scenario avoid|exploit|minimize` |

## Running

```bash
conda activate finger
cd ButtonPressing

# Terminal 1 — hardware
bash ../start_arm.sh

# Terminal 2 — torque node
python3 ../VMC/GUI_MultiVMC_Torque.py

# Terminal 3 — GUI
python3 ../VMC/GUI_MultiVMC.py

# Optional: scenario driver instead of GUI
python3 button_pressing.py --scenario exploit
```

## Data

Saved `.npz` files go to `outputs/button_pressing/` (create this directory before running if using automated saves).
