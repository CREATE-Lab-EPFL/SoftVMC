# fixed_gauss — recording procedure

Both Gaussians (`gauss_seg1`, `gauss_seg2`) sit at a fixed obstacle position;
the arm reaches past it and the field is switched off partway through, so the
run shows both the obstacle-avoiding posture and the free one.

## Procedure

0. From `GaussianElement/`: `bash ../start_arm.sh`, then
   `python3 ../VMC/GUI_MultiVMC_Torque.py --config _fixed --run <name>`, then
   `python3 ../VMC/GUI_MultiVMC.py --config _fixed`.
1. Move `gauss_seg1` and `gauss_seg2` to x = −17 cm.
2. Set `lin_seg3` stiffness = 33, `gauss_seg1` strength = 240, `gauss_seg2`
   strength = 200.
3. Select `lin_seg3` and move it to x = −50 cm.
4. Once the arm reaches steady state, set `gauss_seg1` and `gauss_seg2`
   strength back to 0.

What `fixed_gauss5` actually logged (`k_params`, `goal`): `lin_seg3` 10 → 33.3
N/m, `gauss_seg1` 233.3 and `gauss_seg2` 200 N/m (both then 0), the Gaussian
goals at x = −16.2 and −16.8 cm, and the tip goal at x = −49.6 cm. The strengths
are stiffnesses in N/m, not forces.

A Gaussian spring's force peaks when its attachment point is exactly `sigma`
from the centre, so both start at strength 0 in the config — raising them
before the reference is moved away from the robot risks a very large force at
start-up.

## Component layout

| Component | s | Start (x, y, z) [m] |
|---|---|---|
| `lin_seg3` | 1.0 | (0, 0, 0.435) |
| `gauss_seg1` | 1/3 | (0, 0, 0.2175) |
| `gauss_seg2` | 2/3 | (0, 0, 0.2175) |

Both Gaussians' `goal` in the config is a placeholder — the GUI overrides it
once dragged.
