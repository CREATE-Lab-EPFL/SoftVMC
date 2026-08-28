#!/usr/bin/env python3
"""
The house-of-cards demo, scripted end to end. No GUI.

    python3 castle_demo.py --run gauss        # field on  -- the tip stops short
    python3 castle_demo.py --run no_gauss     # no field  -- the tip goes in
    python3 castle_demo.py --run gauss --dry-run

Two elements, both on the TIP, with two different targets: a linear spring
pulling towards TARGET_XY and a Gaussian pushing away from CASTLE_XY, which sits
between the tip and where it is being sent. So the tip is asked to cross the
castle and the field is the only thing in the way. The demo is to walk the goal
in and watch where the tip stalls.

--dry-run solves for the stall point along the path before anything is run, so a
change to K_REACH, k_gauss, SIGMA or the two points can be checked before the
cards are built.

In order, with nothing touched by hand:

    1. the field comes up 0 -> k over RAMP_S, slowly. The tip goal is still
       sitting on the tip, so the linear spring pulls NOTHING while it does.
    2. --settle: the arm settles with the field at full strength.
    3. the tip goal walks from the tip to TARGET_XY at --speed, straight past
       the castle.
    4. --arrive: the arm follows -- stalling short of the cards if the field is
       on, driving through them if it is not.
    5. the window is marked and held for --window, and the run is written.
    6. the arm is homed and the driver stopped, as the GUI's Home & Quit does.

Terminal 3 runs this INSTEAD of the GUI:

    terminal 1  bash ../start_arm.sh
    terminal 2  python3 ../VMC/GUI_MultiVMC_Torque.py --config _card_castle --run gauss
    terminal 3  python3 castle_demo.py --run gauss

--run must be the same in terminals 2 and 3: this one drives the goals, that one
holds the log and writes the file.
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

import rclpy                                              # noqa: E402
from rclpy.node import Node                               # noqa: E402
from std_msgs.msg import Bool, Float64MultiArray, String  # noqa: E402

from VMC_utils.ConfigHelpers import load_config           # noqa: E402


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default="_card_castle", help="config variant")
    p.add_argument("--run", required=True, metavar="CASE",
                   help="which case -- must match the torque node's --run")
    p.add_argument("--settle", type=float, default=None, metavar="S",
                   help="after the field's ramp, before moving (config: DEMO_SETTLE_S)")
    p.add_argument("--speed", type=float, default=None, metavar="CM_S",
                   help="how fast the tip goal walks in (config: DEMO_SPEED)")
    p.add_argument("--arrive", type=float, default=None, metavar="S",
                   help="hold after it arrives (config: DEMO_ARRIVE_S)")
    p.add_argument("--window", type=float, default=None, metavar="S",
                   help="measurement window after the mark (config: DEMO_WINDOW_S)")
    p.add_argument("--no-home", dest="home", action="store_false",
                   help="do not stop the driver and home at the end")
    p.add_argument("--dry-run", action="store_true",
                   help="print the plan and publish nothing")
    return p.parse_args()


def equilibria(start, target, castle, k_lin, k_gauss, sigma, n=4001):
    """Where the two springs cancel along the straight path start -> target.

    A closed form exists only when both are aimed at the same point; with
    separate targets the balance is solved numerically. Returns (position,
    stable, gap to the castle) for every zero crossing of the net force
    projected on the path.
    """
    start, target, castle = (np.asarray(v, dtype=float) for v in (start, target, castle))
    span = float(np.linalg.norm(target - start))
    if k_gauss <= 0.0 or span == 0.0:
        return []
    u = (target - start) / span

    def net(p):
        pos = start + p * u
        d = pos - castle
        f = k_lin * (target - pos) + k_gauss * np.exp(-d @ d / (2 * sigma ** 2)) * d
        return float(f @ u)

    ps = np.linspace(0.0, span, n)
    fs = np.array([net(p) for p in ps])
    out = []
    for i in np.where(np.diff(np.sign(fs)))[0]:
        pos = start + ps[i] * u
        out.append((pos, fs[i] > fs[i + 1], float(np.linalg.norm(pos - castle))))
    return out


class Driver(Node):
    def __init__(self):
        super().__init__("castle_demo")
        self.pub_goal = self.create_publisher(Float64MultiArray, "/VC_ref_pos", 10)
        self.pub_mark = self.create_publisher(String, "/vmc_mark", 10)
        self.pub_end = self.create_publisher(Bool, "/vmc_end_run", 10)
        self._tau_stamp = None
        self.create_subscription(Float64MultiArray, "/goal_torque", self._cb_tau, 10)

    def _cb_tau(self, _msg):
        self._tau_stamp = time.perf_counter()

    def send(self, goal):
        self.pub_goal.publish(Float64MultiArray(data=goal.reshape(-1).tolist()))

    def hold(self, goal, seconds, label):
        """Keep publishing one goal for a while -- the node wants it repeated."""
        print(f"[demo] {label} ({seconds:.0f} s)", flush=True)
        end = time.time() + seconds
        while time.time() < end:
            self.send(goal)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

    def wait_for_node(self, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end and self._tau_stamp is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._tau_stamp is not None

    def wait_for_save(self, timeout=30.0):
        if self._tau_stamp is None:
            return
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.perf_counter() - self._tau_stamp > 1.5:
                print("[demo] torque node has stopped", flush=True)
                return
            time.sleep(0.1)
        print(f"[demo] torque node still publishing after {timeout:.0f} s "
              "-- check its terminal", flush=True)


def main():
    a = parse_args()
    cfg = load_config(a.config, _HERE)
    conds = getattr(cfg, "GAUSS_CONDITIONS", {})
    if conds and a.run not in conds:
        sys.exit(f"[ERROR] {a.run!r} is not a case of {cfg.__name__}.\n"
                 f"        this config knows: {', '.join(conds)}")

    settle = a.settle if a.settle is not None else getattr(cfg, "DEMO_SETTLE_S", 15.0)
    speed = a.speed if a.speed is not None else getattr(cfg, "DEMO_SPEED", 2.0)
    arrive = a.arrive if a.arrive is not None else getattr(cfg, "DEMO_ARRIVE_S", 20.0)
    window = a.window if a.window is not None else getattr(cfg, "DEMO_WINDOW_S", 20.0)
    ramp_s = float(getattr(cfg, "RAMP_S", 0.0))

    start = cfg.DEFAULT_START.copy()
    castle = np.asarray(getattr(cfg, "CASTLE_XY"), dtype=float)
    target = np.asarray(getattr(cfg, "TARGET_XY", castle), dtype=float)
    tip_from = start[0, :2].copy()
    dist = float(np.linalg.norm(target - tip_from))
    walk_s = dist * 100.0 / speed if speed > 0 else 0.0
    k = float(conds.get(a.run, getattr(cfg, "K_GAUSS", 0.0)))
    eq = equilibria(tip_from, target, castle, cfg.K_REACH, k, cfg.SIGMA)

    print(f"[demo] config   {cfg.__name__}   case {a.run!r}  (k_gauss = {k:g} N/m)")
    print(f"[demo] castle   {tuple(castle)}    target {tuple(target)}    "
          f"goal starts {tuple(tip_from)}")
    print(f"[demo] the goal walks {dist * 100:.0f} cm, straight past the castle at "
          f"{float(np.linalg.norm(castle - tip_from)) * 100:.0f} cm")
    print(f"[demo] 1. field ramps 0 -> {k:g} over {ramp_s:.0f} s, tip goal held still")
    print(f"[demo] 2. settle {settle:.0f} s")
    print(f"[demo] 3. tip goal walks to the target at {speed:g} cm/s -> {walk_s:.0f} s")
    print(f"[demo] 4. arrive {arrive:.0f} s   5. mark + window {window:.0f} s")
    print(f"[demo] total    {ramp_s + settle + walk_s + arrive + window:.0f} s")
    stable = [e for e in eq if e[1]]
    if stable:
        print(f"[demo] expected: the tip stalls {stable[0][2] * 100:.1f} cm short "
              f"of the castle")
        for pos, is_stable, g in eq:
            print(f"[demo]     {'stable  ' if is_stable else 'unstable'} at "
                  f"({pos[0]:+.3f}, {pos[1]:+.3f}), {g * 100:5.1f} cm from the cards")
        if len(eq) > 1:
            print("[demo]     the barrier is finite -- past the unstable point the "
                  "tip would pop through")
    else:
        print("[demo] expected: nothing stops the tip -- it drives through the cards")
    if a.dry_run:
        print("[demo] --dry-run: nothing published")
        return 0

    rclpy.init()
    node = Driver()
    rc = 0
    try:
        print("[demo] waiting for the torque node on /goal_torque ...", flush=True)
        if not node.wait_for_node():
            sys.exit(f"\n[ERROR] nothing is publishing /goal_torque -- the torque node "
                     f"is not running.\n        start it in terminal 2 first:\n"
                     f"          python3 ../VMC/GUI_MultiVMC_Torque.py --config "
                     f"{a.config} --run {a.run}\n"
                     f"        (nothing was published, and the arm was not moved)\n")
        print("[demo] torque node is up", flush=True)

        goal = start.copy()
        node.hold(goal, ramp_s + settle, "field coming up, tip goal held on the tip")

        print(f"[demo] walking the tip goal to the target, past the castle, "
              f"at {speed:g} cm/s", flush=True)
        t0, step = time.time(), None
        while True:
            frac = min(1.0, (time.time() - t0) / walk_s) if walk_s > 0 else 1.0
            goal[0, :2] = tip_from + frac * (target - tip_from)
            node.send(goal)
            if step is None or time.time() - step > 10.0:
                step = time.time()
                print(f"[demo]   {frac * 100:3.0f}%  goal at "
                      f"({goal[0, 0]:+.3f}, {goal[0, 1]:+.3f})", flush=True)
            if frac >= 1.0:
                break
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.03)

        node.hold(goal, arrive, "arrived -- settling")
        node.pub_mark.publish(String(data="target reached"))
        print("[demo] mark: 'target reached'", flush=True)
        node.hold(goal, window, "measurement window")

        print("[demo] done -- ending the run so the torque node writes it", flush=True)
        for _ in range(5):
            node.pub_end.publish(Bool(data=True))
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.1)
        node.wait_for_save()

        path = os.path.join(_HERE, "outputs", getattr(cfg, "OUTPUT_FOLDER", ""),
                            a.run + ".npz")
        if os.path.exists(path):
            print(f"[demo] saved: {path}  ({os.path.getsize(path) / 1e6:.1f} MB)",
                  flush=True)
        else:
            print(f"\n[demo] *** {a.run}.npz WAS NOT WRITTEN ***\n"
                  f"       expected at {path} -- read the torque node's terminal",
                  flush=True)

        if a.home:
            try:
                import MultiGoHome as gh
                gh.release_port(timeout=12.0)
                gh.home_sequence(velocity=int(getattr(cfg, "HOME_VELOCITY", 40)),
                                 timeout=float(getattr(cfg, "HOME_TIMEOUT", 120.0)))
            except Exception as exc:                       # noqa: BLE001 -- reported
                print(f"\n[demo] *** HOMING FAILED -- {type(exc).__name__}: {exc}\n"
                      f"       The run IS saved. Motors left holding in position mode, "
                      f"NOT handed back to current mode.\n"
                      f"       A delta that is a multiple of 4096 is a lost turn count: "
                      f"power-cycle, re-place by hand,\n"
                      f"       and re-capture with python3 ../VMC/MultiGoHome.py --read\n",
                      flush=True)
                rc = 1
        else:
            print("[demo] --no-home: driver still up, arm left where it is", flush=True)
    except KeyboardInterrupt:
        print("\n[demo] interrupted -- the torque node still holds the run; "
              "Ctrl+C there to write it")
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
    return rc


if __name__ == "__main__":
    sys.exit(main())
