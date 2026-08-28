#!/usr/bin/env python3
"""
Replay a recorded run's goal trajectory into a new run, with no GUI.

    python3 replay_goal.py                                  # no_gauss -> gauss, _person
    python3 replay_goal.py --config _card_castle
    python3 replay_goal.py --from gauss --run no_gauss      # the other way round
    python3 replay_goal.py --dry-run                        # print the plan, publish nothing

The point is the comparison. Driven by hand, the two cases differ in the field
AND in wherever the mouse happened to go, and the second difference is the
bigger one -- in the button sweep, two hand-driven runs landed 7.4 cm apart in x
and 5.4 cm apart in y. Here the first run is driven from the GUI and its goal
trajectory is recorded; this script publishes that same trajectory back into the
second run, sample for sample. What is left between them is the Gaussian field.

It replaces the GUI, not the torque node. Terminal 3 runs this instead:

    terminal 1  bash ../start_arm.sh
    terminal 2  python3 ../VMC/GUI_MultiVMC_Torque.py --config _person --run gauss
    terminal 3  python3 replay_goal.py --config _person --from no_gauss --run gauss

Goals are matched by component NAME, not by index, so a source recorded with a
different set of elements still replays: a component the source does not have
takes the goal of whichever component it shares a GOAL_LINK group with (that is
how gauss_seg3 follows gauss_seg1 when replaying a two-Gaussian recording), and
failing that its own DEFAULT_START.

Marks are replayed at their recorded times over /vmc_mark, and the run is ended
and written with /vmc_end_run -- so the measurement window lands in the same
place in both runs and nothing has to be pressed by hand.
"""

import argparse
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_HERE, _ROOT, os.path.join(_ROOT, "Kinematics")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import rclpy                                            # noqa: E402
from rclpy.node import Node                             # noqa: E402
from std_msgs.msg import Bool, Float64MultiArray, String  # noqa: E402

from VMC_utils.ConfigHelpers import load_config, component_names  # noqa: E402


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default="_person", help="config variant (default: _person)")
    p.add_argument("--from", dest="src", default="no_gauss", metavar="RUN",
                   help="run whose goal trajectory is replayed (default: no_gauss)")
    p.add_argument("--run", default="gauss", metavar="NAME",
                   help="the run being recorded now -- must match the torque node's "
                        "--run (default: gauss). Used only to check you have not "
                        "pointed source and destination at the same file.")
    p.add_argument("--rate", type=float, default=50.0, metavar="HZ",
                   help="publish rate (default: 50). The log is at control rate; "
                        "the GUI itself only published about 10 Hz while dragging.")
    p.add_argument("--settle", type=float, default=8.0, metavar="S",
                   help="wait this long before starting, so the field has finished "
                        "ramping in and the arm has settled (default: 8)")
    p.add_argument("--speed", type=float, default=1.0, metavar="X",
                   help="replay faster or slower than recorded (default: 1.0)")
    p.add_argument("--dry-run", action="store_true",
                   help="print the plan and publish nothing")
    return p.parse_args()


def load_source(cfg, src_path):
    """(t, goal_xyz) resampled onto THIS config's components, by name."""
    d = np.load(src_path, allow_pickle=True)
    t = np.asarray(d["t"], dtype=float)
    goal = np.asarray(d["goal"], dtype=float)              # (n, N_src, 3)
    src_names = [str(x) for x in d["comp_names"]]

    keep = t >= 0.0                                        # t<0 is before the GUI connected
    if not keep.any():
        raise SystemExit(f"[ERROR] {src_path} has no samples at t >= 0 -- the GUI never "
                         "published a reference, so there is no trajectory to replay.")
    t, goal = t[keep] - t[keep][0], goal[keep]

    # who follows whom when the source has fewer elements than this config
    link = {}
    for group in getattr(cfg, "GOAL_LINK", []):
        for name in group:
            for other in group:
                if other in src_names:
                    link[name] = other
                    break

    out = np.zeros((len(t), cfg.N_VC, 3), dtype=float)
    how = []
    for i, name in enumerate(component_names(cfg)):
        if name in src_names:
            out[:, i, :] = goal[:, src_names.index(name), :]
            how.append(f"{name} <- {name}")
        elif name in link:
            out[:, i, :] = goal[:, src_names.index(link[name]), :]
            how.append(f"{name} <- {link[name]} (shared goal)")
        else:
            out[:, i, :] = cfg.DEFAULT_START[i]
            how.append(f"{name} <- DEFAULT_START (not in the source)")
    marks = [(float(mt), str(ml)) for mt, ml in
             zip(np.asarray(d.get("marks_t", []), dtype=float),
                 [str(x) for x in d.get("marks_label", [])])
             if np.isfinite(mt) and mt >= 0.0]
    return t, out, how, marks


class Replayer(Node):
    def __init__(self, cfg):
        super().__init__("vmc_goal_replay")
        self.pub_goal = self.create_publisher(Float64MultiArray, "/VC_ref_pos", 10)
        self.pub_mark = self.create_publisher(String, "/vmc_mark", 10)
        self.pub_end = self.create_publisher(Bool, "/vmc_end_run", 10)
        self.cfg = cfg

    def send(self, xyz):
        self.pub_goal.publish(Float64MultiArray(data=xyz.reshape(-1).tolist()))


def main():
    a = parse_args()
    cfg = load_config(a.config, _HERE)
    folder = getattr(cfg, "OUTPUT_FOLDER", "")
    src_path = os.path.join(_HERE, "outputs", folder, a.src + ".npz")
    if not os.path.exists(src_path):
        sys.exit(f"[ERROR] no {src_path}\n"
                 f"        record {a.src!r} from the GUI first -- that is the run this "
                 f"one replays.")
    if a.src == a.run:
        sys.exit(f"[ERROR] --from and --run are both {a.run!r}: that would replay a run "
                 f"into itself and overwrite it.")

    t, goal, how, marks = load_source(cfg, src_path)
    dur = t[-1] / a.speed
    n_out = max(2, int(dur * a.rate))
    t_out = np.linspace(0.0, t[-1], n_out)
    goal_out = np.stack([np.stack([np.interp(t_out, t, goal[:, i, c])
                                   for c in range(3)], axis=1)
                         for i in range(cfg.N_VC)], axis=1)      # (n_out, N_VC, 3)

    tip = goal_out[:, 0, :2]
    print(f"[replay] source   {src_path}")
    print(f"[replay] into     outputs/{folder}/{a.run}.npz  (the torque node writes it)")
    print(f"[replay] {t[-1]:.1f} s of trajectory -> {dur:.1f} s at speed {a.speed:g}, "
          f"{n_out} samples at {a.rate:g} Hz")
    for line in how:
        print(f"[replay]   {line}")
    print(f"[replay] tip goal {tip[0]} -> {tip[-1]}, "
          f"total travel {np.abs(np.diff(tip, axis=0)).sum() * 100:.0f} cm")
    if marks:
        print(f"[replay] marks    {marks}")
    else:
        print("[replay] marks    none in the source -- the measurement window will be "
              "inferred in BOTH runs")
    if a.dry_run:
        print("[replay] --dry-run: nothing published")
        return 0

    rclpy.init()
    node = Replayer(cfg)
    try:
        print(f"[replay] settling {a.settle:.0f} s (the field ramps in over "
              f"{getattr(cfg, 'RAMP_S', 0):g} s) -- hold still", flush=True)
        node.send(goal_out[0])                    # hold the start pose while settling
        t_end = time.time() + a.settle
        while time.time() < t_end:
            node.send(goal_out[0])
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)

        print("[replay] replaying", flush=True)
        pending = list(marks)
        t0 = time.time()
        for k in range(n_out):
            due = t0 + t_out[k] / a.speed
            while time.time() < due:
                rclpy.spin_once(node, timeout_sec=0.002)
                time.sleep(0.001)
            node.send(goal_out[k])
            while pending and t_out[k] >= pending[0][0]:
                mt, ml = pending.pop(0)
                node.pub_mark.publish(String(data=ml))
                print(f"[replay]   mark {ml!r} at t = {t_out[k]:.1f} s", flush=True)
            if k % int(max(1, a.rate * 10)) == 0:
                print(f"[replay]   t = {t_out[k]:6.1f} / {t[-1]:.1f} s", flush=True)

        print("[replay] done -- ending the run so the torque node writes it", flush=True)
        for _ in range(5):
            node.pub_end.publish(Bool(data=True))
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[replay] interrupted -- the torque node still holds the run; "
              "Ctrl+C there to write it")
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
