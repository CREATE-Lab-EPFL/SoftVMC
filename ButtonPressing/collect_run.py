"""
Move the newest auto-saved run out of experiments/ and into outputs/, renamed.

    python3 collect_run.py button_pressing mid_1

GUI_MultiVMC_Torque.py writes experiments/z_<stamp>.npz and .png on exit; this
renames both to outputs/<folder>/<name>.{npz,png} so the notebook finds them.
Refuses to overwrite unless --force.
"""

import argparse
import glob
import os
import shutil


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("folder", help="outputs/ subfolder, e.g. button_pressing")
    p.add_argument("name", help="condition name, e.g. mid_1")
    p.add_argument("--force", action="store_true", help="overwrite an existing file")
    return p.parse_args()


def main():
    args = parse_args()
    here = os.path.dirname(os.path.abspath(__file__))

    runs = glob.glob(os.path.join(here, "experiments", "z_*.npz"))
    if not runs:
        raise SystemExit(
            f"no z_*.npz in {os.path.join(here, 'experiments')}\n"
            "the torque node writes there only if it was launched from this\n"
            "directory and exited cleanly (Ctrl+C), with COLLECTED_DATA = False")
    newest = max(runs, key=os.path.getmtime)
    stem   = os.path.splitext(newest)[0]

    dest_dir = os.path.join(here, "outputs", args.folder)
    os.makedirs(dest_dir, exist_ok=True)

    for ext in (".npz", ".png"):
        src = stem + ext
        if not os.path.exists(src):
            continue
        dst = os.path.join(dest_dir, args.name + ext)
        if os.path.exists(dst) and not args.force:
            raise SystemExit(f"{dst} exists -- pass --force to overwrite")
        shutil.move(src, dst)
        print(f"{os.path.relpath(src, here)} -> {os.path.relpath(dst, here)}")


if __name__ == "__main__":
    main()
