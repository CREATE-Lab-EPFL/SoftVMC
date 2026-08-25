# SoftVMC

Virtual Model Control (VMC) of a 3-segment soft continuum manipulator (Helyx).  Kinematics are modelled with **Piecewise Constant Curvature (PCC)**; control is achieved by attaching virtual mechanical elements (springs, dampers) to points along the arm body and computing the resulting motor torques via the Jacobian transpose.

> **Philosophy:** the robot receives *suggestions* rather than rigid commands, exploiting its inherent compliance to interact safely with the environment and adapt its posture through contact.

## ROS 2 environment

Uses the external ROS 2 workspace that contains the Dynamixel package. Clone it once, alongside this repo:

```bash
cd "$HOME/github"
git clone git@github.com:CREATE-Lab-EPFL/ros2_dynamixel.git
```

Then, for every session:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/github/ros2_dynamixel/install/setup.bash"
```

If the external package changed and needs rebuilding, do it there, not inside this repo:

```bash
cd "$HOME/github/ros2_dynamixel"
colcon build --packages-select dynamixel_interface
source install/setup.bash
```

---

## Repository Structure

```
SoftVMC/
├── Kinematics/          FK and analytical Jacobians (PCC, 3 segments, 9 motors)
├── VMC/                 Core VMC controller + ROS 2 nodes (GUI, torque, go-home)
├── VMC_utils/           Shared building blocks (virtual models, gravity comp)
├── ConnectivityTest/    Run this first: connect + move sanity check, nothing saved
│
├── LinearSpring/        Experiments: k-saturation, disturbance rejection
├── TanhSpring/          Experiments: force-limited spring (sequential jump tests)
├── ConstrainedSpring/   Experiments: surface-sliding (constrained/cart spring)
├── GaussianElement/     Experiments: obstacle avoidance (fixed & moving Gaussian)
├── ContactAndShaping/   Experiments: contact, whole-body shaping, surface tracking
├── ButtonPressing/      Experiments: tip reaching spring vs. two body Gaussians (button + person, in progress)
│
├── SingleVMC/           1-segment VMC (reference / simulation only)
├── LoadCell/            Load cell interface utilities
├── AdditionalMedia/     Raw setup photos/videos, organized by sub-experiment
│                        (per-experiment media/ folders hold setup imagery only;
│                        generated graphs always live in that experiment's outputs/)
│
├── plot_config.mplstyle  Matplotlib style (LaTeX, NPG colors, 18 pt)
├── plot_config.py        Plotting helpers (COLORS, FIG_W_*, set_font_size)
├── start_arm.sh          Full startup script (home → latency → dynamixel_node)
└── README.md            Project overview and setup
```

Each experiment directory follows the same internal layout: `GUI_MultiVMC_Config*.py` (VMC config, one per sub-experiment variant), `outputs/<name>/` (recorded `.npz` runs + generated plots), `media/` (setup photos and screenshots only — **never** generated graphs, which belong in `outputs/`), and `plot_*.ipynb` (the analysis notebook(s) that turn `outputs/` into figures). See each experiment's own `README.md` for which notebook produces which output, and which config file is active by default.

The ROS 2 Dynamixel driver is maintained in the sibling workspace:

```bash
$HOME/github/ros2_dynamixel
```

---

## Setup

**Conda environment:** `finger` (Python 3.10, ROS 2 Humble)

```bash
conda activate finger
```

**ROS 2 driver** (clone once — see "ROS 2 environment" above — then build once in the external workspace):

```bash
cd "$HOME/github/ros2_dynamixel"
colcon build --packages-select dynamixel_interface
source install/setup.bash
```

The startup script launches the node with the current motor calibration:

```bash
-p use_kt:=true
-p kt:=0.000354
```

---

## Quick Start

```bash
# 1. Start the arm (homes motors + starts ROS 2 driver)
bash start_arm.sh

# 2. First time in a session, or after any change to the hardware/software
#    chain: verify it actually works before trusting it with an experiment.
#    Nothing is saved here.
cd ConnectivityTest
python3 ../VMC/GUI_MultiVMC_Torque.py &
python3 ../VMC/MultiVMC_MovingGoal_Pub.py

# 3. From an experiment directory — pick up its VMC config automatically
cd LinearSpring
python3 ../VMC/GUI_MultiVMC_Torque.py &   # torque computation node
python3 ../VMC/GUI_MultiVMC.py            # visualization + control GUI

# 3b. A folder holding several config variants: name the one you want, and
#     name the SAME one in every command of the run. Nothing is copied.
cd ../GaussianElement
python3 ../VMC/GUI_MultiVMC_Torque.py --config _moving &
python3 ../VMC/GUI_MultiVMC.py --config _moving
```

---

## Kinematics

Each segment is parametrized by `(Dx, Dy, Dl)` — lateral curvatures and elongation.  Nine Dynamixel motors (IDs 11–19, three per segment) control tendon lengths.  The mapping `q_motor ↔ PCC ↔ Cartesian pose` is implemented in `Kinematics/`.

See [Kinematics/KINEMATICS_DOCUMENTATION.md](Kinematics/KINEMATICS_DOCUMENTATION.md) for the full derivation.

---

## Virtual Element Library

| Element | Force law | Use case |
|---|---|---|
| `LinearSpring` | f = k Δx | Position control, stiffness characterization |
| `LinearDamper` | f = c ẋ | Velocity damping |
| `TanhSpring` | f = f_max tanh(k Δx / f_max) | Bounded interaction force |
| `ConstrainedSpring` | linear spring on normal direction | Sliding contact |
| `GaussianSpring` | f ∝ exp(−‖Δx‖²/2σ²) | Obstacle avoidance |
| `DeadzoneLimitSpring` | zero inside deadzone | Soft limits |

All elements are in `VMC_utils/VirtualModels.py`.  See [VMC/VMC_DOCUMENTATION.md](VMC/VMC_DOCUMENTATION.md) for the node architecture.

---

## Plotting Style

All notebooks use the shared NPG color palette and LaTeX rendering, no grid lines by default, and every figure saves as both PDF and SVG automatically (patched into `Figure.savefig` by `plot_config.py`). Axes for magnitude-vs-parameter curves (force, energy, displacement — anywhere zero is a meaningful reference) are explicitly anchored at 0 rather than left to matplotlib's auto-padding; spatial/XY-trajectory and signed plots are not forced through zero. Import at the top of each notebook:

```python
import sys; sys.path.insert(0, '..')
from plot_config import COLORS, FIG_W_SINGLE, FIG_W_DOUBLE, set_font_size
GRID = False
FONT_SIZE = 18
set_font_size(FONT_SIZE)
```

---

## Notes

- **ROS 2 version:** ROS 2 Humble (check `ros2_dynamixel` build if switching to Jazzy).
- **Motor kT:** the node launches with `use_kt:=true` and `kt:=0.000354` by default. This is the current motor calibration used by the external Dynamixel package.
- **Latency timer:** set to 1 ms for real-time performance (`start_arm.sh` does this automatically).
- **Reading the 9 motors:** `python3 VMC/MultiGoHome.py --read` prints the current absolute position (encoder ticks) of all 9 motors and exits — no ROS needed, talks to the Dynamixel bus directly. Use it to check raw motor state on demand, or to capture a new `HOME_POSITION`.
- **`--config`:** every script that reads an experiment config — `GUI_MultiVMC_Torque.py`, `GUI_MultiVMC.py`, `MultiVMC_MovingGoal_Pub.py`, `MultiVMC_publish_0.py` — accepts `--config NAME`, loading `GUI_MultiVMC_Config_NAME.py` from the launch directory instead of the plain `GUI_MultiVMC_Config.py`. `_moving` and `moving` are equivalent. This replaces copying a variant over `GUI_MultiVMC_Config.py`; the copy still works, but every command in a run must name the same variant, and a wrong name lists what is available. The folder runners (`GaussianElement/moving_gauss_test.py`, `LinearSpring/VMC_test_k.py`, `TanhSpring/Sequential_jump_tests.py`) take the variant as their first argument instead: `python3 moving_gauss_test.py _moving`.
- **`--run NAME`:** name a run and `GUI_MultiVMC_Torque.py` writes it straight to `outputs/<OUTPUT_FOLDER>/NAME.{npz,png}` on exit, **overwriting** a previous run of that name — so repeating a run replaces it instead of piling up files. Without it, output goes to a timestamped `experiments/z_<stamp>` pair as before, to be moved later with the folder's `collect_run.py`. The `.npz` is written before the plot, so an interrupt during the save costs the figure, never the data.
- **`GRAVITY_DIR`:** direction gravity pulls **in the robot frame**, read from each experiment's config by `GUI_MultiVMC_Torque.py` (`"-z"` if a config omits it). `"-z"` is the arm standing straight up; `"-y"` is the arm rotated onto its side, which is how `ButtonPressing/` and `ConnectivityTest/` are set up. Override per run without editing the config: `python3 ../VMC/GUI_MultiVMC_Torque.py --gravity-dir -z`. Get it wrong and gravity compensation pushes along an axis gravity no longer acts on. **The arm is currently mounted on its side**, so every other folder — whose configs assume upright — needs either a remount or `--gravity-dir -y` to run at all, and only a remount to reproduce its recorded data.
- **`COLLECTED_DATA`:** every experiment config sets this flag (see `GUI_MultiVMC_Torque.py`). `True` means real data is already collected for that config — reruns (demos, sanity checks) will never overwrite it. Currently `True` everywhere except `ButtonPressing/`, which is still being recorded. `ConnectivityTest/` has no such flag since it never saves anything regardless.
