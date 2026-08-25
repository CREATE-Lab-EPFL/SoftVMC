# VMC_utils — Shared VMC Utilities

Reusable building blocks shared across all VMC experiments.

## Files

| File | Contents |
|---|---|
| `VirtualModels.py` | Spring classes: `LinearSpring`, `TanhSpring`, `SigmoidSpring`, `PolynomialSpring`, `GaussianSpring`, `DeadzoneLimitSpring`, plus plane-constrained variants `ConstrainedLinearSpring`, `ConstrainedTanhSpring`, `ConstrainedGaussianSpring`. Damper classes: `LinearDamper`, `TanhDamper`, `ConstrainedLinearDamper`, `ConstrainedTanhDamper` |
| `GravityCompensation.py` | Gravity torque estimate for the arm, for any mounting orientation: `rotation_for_gravity_dir("-y")` turns the direction gravity pulls in the robot frame into the rotation matrix `GravityCompensation` expects |
| `ConfigHelpers.py` | Boilerplate shared by the experiment configs: `rest_pose(s)`, `default_start(components)`, `param_info(components)`, and `REST_LENGTH` imported from `Kinematics/JacobiansMulti.py` rather than re-typed. Also the config loader every script shares: `load_config(variant)`, `config_variants(dir)`, `config_from_argv()` — which is what makes `--config _moving` work instead of copying a variant over `GUI_MultiVMC_Config.py` |

## Import pattern

```python
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from VMC_utils.VirtualModels import LinearSpring, LinearDamper, TanhSpring
```
