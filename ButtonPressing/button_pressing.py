"""
Headless runner for the reach-and-avoid experiments. The config variant is the
first argument -- nothing is copied into place:

    python3 button_pressing.py _button    # press the button, load cell in the loop
    python3 button_pressing.py _person    # same button, a person in the way

TERMINAL 1  bash ../start_arm.sh
TERMINAL 2  python3 ../LoadCell/arduino_force.py            # /force_shear (_button only)
TERMINAL 3  python3 ../VMC/GUI_MultiVMC_Torque.py --config _button
TERMINAL 4  python3 button_pressing.py _button --condition no_gauss

Both terminals must name the same variant; the driver aborts if they disagree.
One condition per run -- the torque node saves once, on exit. After each run
Ctrl+C the torque node, then:

    python3 collect_run.py button_pressing no_gauss

and restart the torque node for the next condition.

Before running _person, anywhere near a person:
  - PERSON_POS in the config -- where the person's arm is -- is a placeholder.
    Measure it and edit it. The target is the button, same as _button.
  - --dry-run first, then run --condition strong and confirm the arm stops
    clear before trying anything lower.
  - Someone on the e-stop, ready to Ctrl+C the torque node, every run.
"""

import argparse
import glob
import importlib
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_BASE = "GUI_MultiVMC_Config"

# Variants are whatever config files sit next to this script, so adding a new
# GUI_MultiVMC_Config_<name>.py is enough to make `<name>` runnable.
VARIANTS = sorted("_" + os.path.basename(f)[len(_BASE) + 1:-3]
                  for f in glob.glob(os.path.join(_HERE, f"{_BASE}_*.py")))
DEFAULT_VARIANT = "_button" if "_button" in VARIANTS else (VARIANTS[0] if VARIANTS else None)


def load_config(variant):
    """Import GUI_MultiVMC_Config_<variant>; '_button' and 'button' both work."""
    return importlib.import_module(f"{_BASE}_{variant.lstrip('_')}")


def parse_args():
    # The variant decides which conditions exist, so it is resolved before the
    # real parser is built.
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("variant", nargs="?", default=DEFAULT_VARIANT, choices=VARIANTS)
    cfg = load_config(pre.parse_known_args()[0].variant)

    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("variant", nargs="?", default=DEFAULT_VARIANT, choices=VARIANTS,
                   help=f"config variant to run (default: {DEFAULT_VARIANT})")
    p.add_argument("--condition", choices=sorted(cfg.GAUSS_CONDITIONS),
                   help="hardcoded Gaussian strength to use for this run")
    p.add_argument("--gauss-strength", type=float, default=None,
                   help="override the condition with an explicit strength (tuning only)")
    p.add_argument("--hold-time", type=float, default=cfg.HOLD_TIME,
                   help=f"seconds to hold at steady state (default: {cfg.HOLD_TIME:.0f})")
    p.add_argument("--dry-run", action="store_true",
                   help="print the offline force/torque check and exit, no ROS")
    return p.parse_args(), cfg


def main():
    args, cfg = parse_args()

    if args.dry_run:
        from _geometry_check import report
        report(cfg)
        return

    if args.gauss_strength is not None:
        strength, label = args.gauss_strength, "tuning"
    elif args.condition is not None:
        strength, label = cfg.GAUSS_CONDITIONS[args.condition], args.condition
    else:
        raise SystemExit("give --condition {" + ",".join(sorted(cfg.GAUSS_CONDITIONS))
                         + "} or --gauss-strength")

    safe_first = getattr(cfg, "SAFE_FIRST_CONDITION", None)
    if safe_first is not None and label != safe_first:
        print(f"NOTE: '{safe_first}' should have been recorded and verified first.")

    print(f"{cfg.CONFIG_NAME} / condition '{label}': gauss strength {strength}")
    from _reach_avoid_driver import run_reach_avoid
    run_reach_avoid(cfg, strength, hold_time=args.hold_time, label=cfg.OUTPUT_FOLDER)
    print(f"\nnext: Ctrl+C the torque node, then\n"
          f"  python3 collect_run.py {cfg.OUTPUT_FOLDER} {label}")


if __name__ == "__main__":
    main()
