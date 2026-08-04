# VMC_utils — Shared VMC Utilities

Reusable building blocks shared across all VMC experiments.

## Files

| File | Contents |
|---|---|
| `VirtualModels.py` | Spring classes: `LinearSpring`, `TanhSpring`, `SigmoidSpring`, `PolynomialSpring`, `GaussianSpring`, `DeadzoneLimitSpring`, plus plane-constrained variants `ConstrainedLinearSpring`, `ConstrainedTanhSpring`, `ConstrainedGaussianSpring`. Damper classes: `LinearDamper`, `TanhDamper`, `ConstrainedLinearDamper`, `ConstrainedTanhDamper` |
| `GravityCompensation.py` | Gravity torque estimate for the arm |

## Import pattern

```python
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from VMC_utils.VirtualModels import LinearSpring, LinearDamper, TanhSpring
```
