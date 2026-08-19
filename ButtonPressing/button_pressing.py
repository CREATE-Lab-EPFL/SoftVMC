"""
button_pressing.py
───────────────────
Headless driver for the Fig 9 button-pressing contact-force sweep. Drives the
tip to the hardcoded button_pos (from GUI_MultiVMC_Config.py) and holds a
hardcoded gauss_repel strength for the whole run -- see
_press_repel_driver.py for the actual approach/ramp/hold state machine.

Run once per condition; GUI_MultiVMC_Torque.py only auto-saves once, on
exit, so restart both this script and the torque node between conditions.
Rename each auto-saved .npz into outputs/button_pressing/ right after --
e.g. no_contact.npz / middle_1.npz / middle_2.npz / full_contact.npz -- so
plot_button_and_person.ipynb can find them by name.

TERMINAL 1
bash ../start_arm.sh

TERMINAL 2 (from ButtonPressing/)
python3 ../VMC/GUI_MultiVMC_Torque.py

TERMINAL 3 (from ButtonPressing/, replaces GUI_MultiVMC.py)
python3 button_pressing.py --gauss-strength 0        # full contact
python3 button_pressing.py --gauss-strength 300      # middle -- UNVERIFIED, tune live first
python3 button_pressing.py --gauss-strength 700      # middle -- UNVERIFIED, tune live first
python3 button_pressing.py --gauss-strength 1500     # no contact -- UNVERIFIED, tune live first

The 4 strength values above are placeholders, not a calibration. Find real
values live via the GUI (GUI_MultiVMC.py) first -- watch /force_shear, sweep
the slider, note what strength kills contact vs. halves the force -- THEN
lock those numbers into the commands above for the recorded/reported runs.
"""

import argparse
import sys, os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from GUI_MultiVMC_Config import button_pos
from _press_repel_driver import run_press_repel


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--gauss-strength", type=float, required=True,
                    help="gauss_repel strength for this run. 0 = full contact "
                         "(press's max_force is felt directly); higher values "
                         "reduce contact force. No default -- choose deliberately.")
    p.add_argument("--press-stiffness", type=float, default=None,
                    help="override press.stiffness (default: GUI_MultiVMC_Config's initial value)")
    p.add_argument("--hold-time", type=float, default=15.0,
                    help="seconds to hold at final strength before signalling done (default: 15)")
    return p.parse_args()


def main():
    args = parse_args()
    run_press_repel(
        target_pos=button_pos,
        gauss_strength=args.gauss_strength,
        press_stiffness=args.press_stiffness,
        hold_time=args.hold_time,
        label="button_pressing",
    )


if __name__ == "__main__":
    main()
