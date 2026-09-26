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
├── ButtonPressing/      Experiments: tip reaching spring vs. body Gaussians (button + person locked; card castle recorded once, not final)
│
├── SingleVMC/           1-segment VMC (reference / simulation only)
├── LoadCell/            Load cell interface utilities (Arduino sketch + ROS 2 publisher)
│
├── plot_config.mplstyle  Matplotlib style (LaTeX, pastel palette, 18 pt)
├── plot_config.py        Plotting helpers (COLORS, PASTEL_CMAP*, FIG_W_*, set_font_size, pick_spread, make_square_panel_row)
├── start_arm.sh          Full startup script (home → latency → dynamixel_node)
└── README.md            Project overview and setup
```

Each experiment directory follows the same internal layout: `GUI_MultiVMC_Config*.py` (VMC config, one per sub-experiment variant), `outputs/<name>/` (recorded `.npz` runs + generated plots), `plot_*.ipynb` (the analysis notebook(s) that turn `outputs/` into figures), and — only in `ButtonPressing/` and `ContactAndShaping/` — `media/` (setup photos and screenshots only, **never** generated graphs, which belong in `outputs/`). The smaller experiments keep a `setup*.png` inside their `outputs/` folder instead. See each experiment's own `README.md` for which notebook produces which output, and which config file is active by default.

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
| `GaussianSpring` | f = k · exp(−‖Δx‖²/2σ²) · Δx | Obstacle avoidance |
| `DeadzoneLimitSpring` | zero inside deadzone | Soft limits |

For `GaussianSpring`, `Δx` = attachment point − centre, **not** a unit vector, so `k` is a stiffness in **N/m**. (The constructor argument is called `strength`, and its docstring in `VirtualModels.py`, some notebook text and a few plot legends still say "force amplitude, N" — it is not a force.) The force is zero at the centre, peaks at `‖Δx‖ = σ` with magnitude `k·σ·e^(−1/2)` ≈ 0.607·k·σ newtons, and dies off beyond about 3σ. With `σ = 0.06 m`, 500 N/m peaks at 18 N.

All elements are in `VMC_utils/VirtualModels.py` — that file also has `SigmoidSpring`, `PolynomialSpring`, the tanh and constrained variants of the dampers, and the constrained Gaussian; see [VMC/VMC_DOCUMENTATION.md](VMC/VMC_DOCUMENTATION.md) for the full table and the node architecture.

---

## Plotting Style

Every notebook imports `plot_config.py`, which applies `plot_config.mplstyle`: LaTeX text (`text.usetex`, Computer Modern), a **pastel** palette (`COLORS`, plus `PASTEL_CMAP*` for swept parameters), no grid lines, 18 pt text. Saving a figure as PDF through `Figure.savefig` also writes an SVG next to it (patched in by `plot_config.py`; PNG saves get no twin, and an SVG that fails to write is skipped silently). Axes for magnitude-vs-parameter curves (force, energy, displacement — anywhere zero is a meaningful reference) are anchored at 0 rather than left to matplotlib's auto-padding, except where a marker or error-bar cap sits at 0 and would be clipped, so the axis is padded slightly below it (`ButtonPressing` `force_vs_strength`, `TanhSpring` measured force); spatial/XY-trajectory and signed plots are not forced through zero. Import at the top of each notebook:

```python
import sys; sys.path.insert(0, '..')
from plot_config import COLORS, FIG_W_SINGLE, FIG_W_DOUBLE, set_font_size
GRID = False
FONT_SIZE = 18
set_font_size(FONT_SIZE)
```

Deliberate exceptions:

- `TanhSpring/plot_tanh_spring.ipynb` turns `usetex` off (mathtext with `cmr10`), so it needs no TeX and is not affected by the issue below.
- The `ButtonPressing/` notebooks pin their own colours (NPG in `plot_button`, Dark2 in `plot_person` and `plot_card_castle`); see that folder's README.
- `FONT_SIZE` is 18 unless a notebook says otherwise: 24 in `plot_gauss_fixed_gauss`, 28 in the LinearSpring figures that stack into one panel (`ALIGNED_*` in `plot_config.py`).

**Requirements.** A LaTeX install (MiKTeX or TeX Live), plus numpy, matplotlib, scipy and sympy (the notebooks that draw the arm import `Kinematics/FK_Multi`, which builds its Jacobians with sympy). There is no requirements file; the committed figures were made with four different matplotlib versions (3.10.0, 3.10.8, 3.11.0, 3.11.2), and layouts shift slightly between them (the `body_shaping` legends do), so pin one before regenerating a whole set.

> ⚠ **Known issue — matplotlib 3.11.0 drops minus signs from LaTeX PDFs.** Its Type-1 font subsetter overwrites glyph code 0, which in CMSY10 is the minus, so a tick labelled −20 prints as 20. The SVG twin is unaffected. Use 3.10.x or ≥ 3.11.2 (checked: 3.11.2 needs no workaround). Six committed PDFs were made with 3.11.0 and still carry the defect — regenerate them by re-running their notebooks on a fixed matplotlib (the five `ButtonPressing/outputs/button_pressing/backbone*.pdf` were regenerated on 3.11.2 and are fine):
>
> - `ButtonPressing/outputs/person_avoiding/backbone.pdf`
> - `LinearSpring/outputs/k_saturation/{force_vs_pos_by_k,stiffness_vs_pos_by_k,force_and_stiffness_by_k}.pdf` and `LinearSpring/outputs/disturbance_rejection/disturbance_response_by_k.pdf`
> - `GaussianElement/outputs/fixed_gauss/fixed_gauss5_xz.pdf`
>
> To check a PDF: `fitz.open(path).metadata['producer']` names the matplotlib that wrote it, and rendering it (not extracting its text, which still reports the minus) shows whether the sign is drawn. Delete this note once they are regenerated.

---

## Notes

- **ROS 2 version:** ROS 2 Humble (check `ros2_dynamixel` build if switching to Jazzy).
- **Motor kT:** the node launches with `use_kt:=true` and `kt:=0.000354` by default. This is the current motor calibration used by the external Dynamixel package.
- **Latency timer:** set to 1 ms for real-time performance (`start_arm.sh` does this automatically).
- **Reading the 9 motors:** `python3 VMC/MultiGoHome.py --read` prints the current absolute position (encoder ticks) of all 9 motors and exits — no ROS needed, talks to the Dynamixel bus directly. Use it to check raw motor state on demand, or to capture a new `HOME_POSITION`.
- **`--config`:** every script that reads an experiment config — `GUI_MultiVMC_Torque.py`, `GUI_MultiVMC.py`, `MultiVMC_MovingGoal_Pub.py`, `MultiVMC_publish_0.py` — accepts `--config NAME`, loading `GUI_MultiVMC_Config_NAME.py` from the launch directory instead of the plain `GUI_MultiVMC_Config.py`. `_moving` and `moving` are equivalent. This replaces copying a variant over `GUI_MultiVMC_Config.py`; the copy still works, but every command in a run must name the same variant, and a wrong name lists what is available. The folder runners (`GaussianElement/moving_gauss_test.py`, `LinearSpring/VMC_test_k.py`, `TanhSpring/Sequential_jump_tests.py`) take the variant as their first argument instead: `python3 moving_gauss_test.py _moving`.
- **`--run NAME`:** name a run and `GUI_MultiVMC_Torque.py` writes it straight to `outputs/<OUTPUT_FOLDER>/NAME.{npz,png}` on exit, **overwriting** a previous run of that name — so repeating a run replaces it instead of piling up files. Without it, output goes to a timestamped `experiments/z_<stamp>` pair as before, to be moved later with the folder's `collect_run.py`. The `.npz` is written before the plot, so an interrupt during the save costs the figure, never the data.
- **`GRAVITY_DIR`:** direction gravity pulls **in the robot frame**, read from each experiment's config by `GUI_MultiVMC_Torque.py` (`"-z"` if a config omits it). `"-z"` is the arm standing straight up; `"-y"` is the arm rotated onto its side, which is how `ButtonPressing/`, `ConnectivityTest/` and `ContactAndShaping`'s `_bending` variant are set up. Override per run without editing the config: `python3 ../VMC/GUI_MultiVMC_Torque.py --gravity-dir -z`. Get it wrong and gravity compensation pushes along an axis gravity no longer acts on. **The arm is currently mounted on its side**, so every other config — which assumes upright — needs either a remount or `--gravity-dir -y` to run at all, and only a remount to reproduce its recorded data.
- **`COLLECTED_DATA`:** every experiment config sets this flag (see `GUI_MultiVMC_Torque.py`). `True` means real data is already collected for that config — reruns (demos, sanity checks) will never overwrite it. Currently `True` everywhere **except three variants that ship `False`**, where re-using a run name overwrites the file (`ls outputs/<folder>/` first): `ButtonPressing/`'s `_card_castle` (recorded once, not final; its `_button` and `_person` variants are finished and locked), `ContactAndShaping/`'s `_bending` (a new name per case, see its README), and `GaussianElement/`'s `_moving` (which already holds three recorded runs). `ConnectivityTest/` has no such flag since it never saves anything regardless.
