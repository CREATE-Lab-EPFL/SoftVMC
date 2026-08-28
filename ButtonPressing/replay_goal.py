#!/usr/bin/env python3
"""
Replay a recorded run's goal trajectory into a new run, with no GUI.

    python3 replay_goal.py                                  # no_gauss -> gauss, _person
    python3 replay_goal.py --config _card_castle
    python3 replay_goal.py --from gauss --run no_gauss      # the other way round
    python3 replay_goal.py --dry-run                        # print the plan, publish nothing

ONLY THE REFERENCE IS REPLAYED -- the goals on /VC_ref_pos, nothing else. The
torque is still computed live by the torque node, from the arm's actual state
and whatever field this run has. That is the whole idea: replaying the torque
would replay the no_gauss motion verbatim and the Gaussians could not change
anything, which is the opposite of the experiment. Replaying the reference makes
the operator's input identical and lets the field do what it does.

    replayed:  the goal trajectory -- what the hand asked for
    NOT:       torque, joint angles, or any part of the response

So the two runs share an input and differ in a controller. Driven by hand
instead, they would differ in the field AND in wherever the mouse happened to
go, and the second difference is the bigger one -- in the button sweep, two
hand-driven runs landed 7.4 cm apart in x and 5.4 cm apart in y.

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

It then finishes the session exactly as the GUI's "Home & Quit" does: stop
dynamixel_node (which closes start_arm.sh in terminal 1), home the arm in
extended position control, hand back in current mode. --no-home skips that and
leaves everything up.
"""

import argparse
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_HERE, _ROOT, os.path.join(_ROOT, "VMC"), os.path.join(_ROOT, "Kinematics")):
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
    p.add_argument("--rate", type=float, default=100.0, metavar="HZ",
                   help="publish rate (default: 100). The log is at control rate and "
                        "the GUI published about 10 Hz while dragging, so this is "
                        "already finer than the source. A click JUMPS the goal, and a "
                        "step is smeared over one interval, so raise it (or drop "
                        "--speed) if the source has big jumps.")
    p.add_argument("--settle", type=float, default=8.0, metavar="S",
                   help="wait this long before starting, so the field has finished "
                        "ramping in and the arm has settled (default: 8)")
    p.add_argument("--speed", type=float, default=1.0, metavar="X",
                   help="replay faster or slower than recorded (default: 1.0)")
    p.add_argument("--no-home", dest="home", action="store_false",
                   help="do NOT end the session: leave the driver up and the arm "
                        "where it is. By default the replay finishes like the GUI's "
                        "Home & Quit -- stop the driver, home, hand back.")
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
        self._tau_stamp = None
        self.create_subscription(Float64MultiArray, "/goal_torque",
                                 self._cb_tau, 10)

    def _cb_tau(self, _msg):
        self._tau_stamp = time.perf_counter()

    def wait_for_save(self, timeout=30.0):
        """Block until the torque node stops publishing -- it exits once written."""
        if self._tau_stamp is None:
            print("[replay] no torque node was publishing -- nothing to wait for",
                  flush=True)
            return
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.perf_counter() - self._tau_stamp > 1.5:
                print("[replay] torque node has stopped", flush=True)
                return
            time.sleep(0.1)
        print(f"[replay] torque node still publishing after {timeout:.0f} s "
              "-- check its terminal", flush=True)

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
    conds = getattr(cfg, "GAUSS_CONDITIONS", None)
    if conds:
        bad = [n for n in (a.src, a.run) if n not in conds]
        if bad:
            sys.exit(f"[ERROR] not a case of {cfg.__name__}: {', '.join(bad)}\n"
                     f"        this config knows: {', '.join(conds)}\n"
                     f"        --run has to be the SAME name the torque node was given.")

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
        # Publishing a trajectory at nothing wastes the run and, worse, would
        # still reach the homing at the end and move the arm. Check first.
        print("[replay] waiting for the torque node on /goal_torque ...", flush=True)
        deadline = time.time() + 10.0
        while time.time() < deadline and node._tau_stamp is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node._tau_stamp is None:
            sys.exit(f"\n[ERROR] nothing is publishing /goal_torque -- the torque node "
                     f"is not running.\n"
                     f"        start it in terminal 2 first:\n"
                     f"          python3 ../VMC/GUI_MultiVMC_Torque.py --config "
                     f"{a.config} --run {a.run}\n"
                     f"        (nothing was published, and the arm was not moved)\n")
        print("[replay] torque node is up", flush=True)

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
        node.wait_for_save()

        run_path = os.path.join(_HERE, "outputs", folder, a.run + ".npz")
        if os.path.exists(run_path):
            print(f"[replay] saved: {run_path}  "
                  f"({os.path.getsize(run_path) / 1e6:.1f} MB)", flush=True)
        else:
            print(f"\n[replay] *** {a.run}.npz WAS NOT WRITTEN ***\n"
                  f"         expected at {run_path} -- read the torque node's terminal",
                  flush=True)

        if a.home:
            # Same ending as the GUI's Home & Quit, and in the same order: the
            # run is written first, then the bus is taken off dynamixel_node
            # (which closes start_arm.sh), then the arm is homed. The run is
            # already on disk by here, so a failure below costs the homing only
            # -- report it, do not bury it in a traceback.
            try:
                import MultiGoHome as gh
                gh.release_port(timeout=12.0)
                gh.home_sequence(velocity=int(getattr(cfg, "HOME_VELOCITY", 40)),
                                 timeout=float(getattr(cfg, "HOME_TIMEOUT", 120.0)))
            except Exception as exc:                     # noqa: BLE001 -- reported
                print(f"\n[replay] *** HOMING FAILED -- {type(exc).__name__}: {exc}\n"
                      f"         The run IS saved. The motors are left holding in "
                      f"position mode and were NOT handed back to current mode:\n"
                      f"         a delta that is a multiple of 4096 is a lost turn "
                      f"count, not a motor that failed to move.\n"
                      f"         Power-cycle, re-place by hand, and re-capture with "
                      f"python3 ../VMC/MultiGoHome.py --read\n", flush=True)
                return 1
        else:
            print("[replay] --no-home: driver still up, arm left where it is",
                  flush=True)
    except KeyboardInterrupt:
        print("\n[replay] interrupted -- the torque node still holds the run; "
              "Ctrl+C there to write it")
    finally:
        # shutdown first, then destroy -- the other order tears the node out
        # from under the executor.
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
