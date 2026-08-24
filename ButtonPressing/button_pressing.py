"""
Headless runner for GUI_MultiVMC_Config_button.py.

    cp GUI_MultiVMC_Config_button.py GUI_MultiVMC_Config.py

TERMINAL 1  bash ../start_arm.sh
TERMINAL 2  python3 ../LoadCell/arduino_force.py       # /force_shear
TERMINAL 3  python3 ../VMC/GUI_MultiVMC_Torque.py
TERMINAL 4  python3 button_pressing.py --condition no_gauss

One condition per run -- the torque node saves once, on exit. After each run
Ctrl+C the torque node, then:

    python3 collect_run.py button_pressing no_gauss

and restart the torque node for the next condition.
"""

import argparse
import sys

try:
    import GUI_MultiVMC_Config as cfg
except ModuleNotFoundError:
    sys.exit('no GUI_MultiVMC_Config.py in this directory -- run:\n'
             '  cp GUI_MultiVMC_Config_button.py GUI_MultiVMC_Config.py')


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--condition", choices=sorted(cfg.GAUSS_CONDITIONS),
                   help="hardcoded Gaussian strength to use for this run")
    p.add_argument("--gauss-strength", type=float, default=None,
                   help="override the condition with an explicit strength (tuning only)")
    p.add_argument("--hold-time", type=float, default=15.0,
                   help="seconds to hold at steady state (default: 15)")
    p.add_argument("--dry-run", action="store_true",
                   help="print the offline force/torque check and exit, no ROS")
    return p.parse_args()


def main():
    args = parse_args()

    if cfg.CONFIG_NAME != "button":
        sys.exit(f"active GUI_MultiVMC_Config.py is '{cfg.CONFIG_NAME}', expected 'button' -- "
                 f"run: cp GUI_MultiVMC_Config_button.py GUI_MultiVMC_Config.py")

    if args.dry_run:
        from _geometry_check import report
        report()
        return

    if args.gauss_strength is not None:
        strength, label = args.gauss_strength, "tuning"
    elif args.condition is not None:
        strength, label = cfg.GAUSS_CONDITIONS[args.condition], args.condition
    else:
        sys.exit("give --condition {" + ",".join(sorted(cfg.GAUSS_CONDITIONS)) + "} or --gauss-strength")

    print(f"condition '{label}': gauss strength {strength}")
    from _reach_avoid_driver import run_reach_avoid
    run_reach_avoid(strength, hold_time=args.hold_time, label="button_pressing")
    print(f"\nnext: Ctrl+C the torque node, then\n  python3 collect_run.py button_pressing {label}")


if __name__ == "__main__":
    main()
