"""
person_avoiding.py
────────────────────
Headless driver for the Fig 10 person-interaction demo (qualitative
photo/video only, per the paper's softer "C. Demo" section -- not a
data-analysis target). Same press+gauss_repel mechanism as
button_pressing.py -- see _press_repel_driver.py for the actual
approach/ramp/hold state machine -- pointed at a person instead of the
load-cell button:

  --gauss-strength HIGH  -> arm reaches toward the person but is repelled
                            before contact ("avoid")
  --gauss-strength LOW   -> arm makes gentle, bounded contact ("gentle touch"),
                            still capped by press's max_force in
                            GUI_MultiVMC_Config.py and by TAU_LIMIT

SAFETY -- read before running:
  - PERSON_TARGET below is a PLACEHOLDER, not a measurement. Measure the
    real target position (or pass --target X Y Z) before running anything
    near an actual person.
  - Always run --gauss-strength HIGH ("avoid") first and confirm the arm
    stops well clear of the person before ever trying a lower value.
  - Have a second person at the e-stop / ready to Ctrl+C the torque node
    for every run, not just the "gentle touch" ones.
  - No numeric default is provided for --gauss-strength -- choose it
    deliberately every run.

TERMINAL 1
bash ../start_arm.sh

TERMINAL 2 (from ButtonPressing/)
python3 ../VMC/GUI_MultiVMC_Torque.py

TERMINAL 3 (from ButtonPressing/, replaces GUI_MultiVMC.py)
python3 person_avoiding.py --gauss-strength 1500 --target 0.10 0.0 0.35   # avoid -- run this first
python3 person_avoiding.py --gauss-strength 200   --target 0.10 0.0 0.35  # gentle touch -- UNVERIFIED strength
"""

import argparse

import numpy as np

from _press_repel_driver import run_press_repel

PERSON_TARGET = np.array([0.10, 0.0, 0.35], dtype=float)  # PLACEHOLDER -- measure on the real setup


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--gauss-strength", type=float, required=True,
                    help="gauss_repel strength for this run. High = avoid (no contact), "
                         "low = gentle touch. No default -- choose deliberately, and run "
                         "a high (avoid) value first.")
    p.add_argument("--target", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                    help="override PERSON_TARGET, e.g. after live-measuring the person's position")
    p.add_argument("--press-stiffness", type=float, default=None,
                    help="override press.stiffness (default: GUI_MultiVMC_Config's initial value)")
    p.add_argument("--hold-time", type=float, default=15.0,
                    help="seconds to hold at final strength before signalling done (default: 15)")
    return p.parse_args()


def main():
    args = parse_args()
    target = np.array(args.target, dtype=float) if args.target is not None else PERSON_TARGET
    run_press_repel(
        target_pos=target,
        gauss_strength=args.gauss_strength,
        press_stiffness=args.press_stiffness,
        hold_time=args.hold_time,
        label="person_avoiding",
    )


if __name__ == "__main__":
    main()
