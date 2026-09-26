# moving_gauss — recording procedure

A single Gaussian (`gauss_seg3`, at the tip) circles a moving obstacle while
the arm reaches; recorded at three linear-spring stiffnesses for comparison.

## Procedure

0. From `GaussianElement/`: `bash ../start_arm.sh`, then
   `python3 ../VMC/GUI_MultiVMC_Torque.py --config _moving --run <name>`.
1. `python3 moving_gauss_test.py _moving` with `lin_seg3` k = 2 → `k2_moving_gauss`.
2. The same with k = 4 → `k4_moving_gauss`.
3. The same with k = 20 → `k20_moving_gauss`.

`plot_gauss_moving_gauss.ipynb` turns the three recordings into plots. Note that
`_moving` ships `COLLECTED_DATA = False`, so re-using one of those names
overwrites the recording, and that the logged tip stiffness is 1.67 / 4.0 /
16.67 N/m for the three names (see the folder's README).

A Gaussian spring's force peaks when its attachment point is exactly `sigma`
from the centre, so it starts at strength 0 in the config — raising it before
the reference is moved away from the robot risks a very large force at
start-up.

`DEFAULT_START` here is derived per component from its `s_param`, rather than
the hardcoded start pose `fixed_gauss` uses.
