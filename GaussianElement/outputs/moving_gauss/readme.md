# moving_gauss — recording procedure

A single Gaussian (`gauss_seg3`, at the tip) circles a moving obstacle while
the arm reaches; recorded at two linear-spring stiffnesses for comparison.

## Procedure

0. `bash ../../start_arm.sh`, then `python3 ../../VMC/GUI_MultiVMC_Torque.py`
   (from `GaussianElement/`, `--config _moving`).
1. `python3 moving_gauss_test.py` with `lin_seg3` k = 2.
2. `python3 moving_gauss_test.py` with `lin_seg3` k = 20.

`plot_gauss_moving_gauss.ipynb` turns the two recordings into plots.

A Gaussian spring's force peaks when its attachment point is exactly `sigma`
from the centre, so it starts at strength 0 in the config — raising it before
the reference is moved away from the robot risks a very large force at
start-up.

`DEFAULT_START` here is derived per component from its `s_param`, rather than
the hardcoded start pose `fixed_gauss` uses.
