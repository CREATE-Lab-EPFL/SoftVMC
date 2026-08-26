#!/usr/bin/env python3
"""
The whole condition sweep, unattended, with no GUI.

    python3 run_sweep.py                        # _button: no_gauss, weak, medium, strong
    python3 run_sweep.py --config _person       # _person: no_gauss, medium, strong
    python3 run_sweep.py --only weak strong
    python3 run_sweep.py --dry-run              # print the plan and the timings, touch nothing

Every condition starts from the same arm pose and drives to the same target, so
the only thing that differs between them is the Gaussian strength. That is what
the sweep is for, and it is what dragging by hand could not give: `weak` and
`medium` recorded by hand landed 7.4 cm apart in x and 5.4 cm apart in y.

Per condition, in this order:

    launch dynamixel_node (torque mode), wait for /joint_positions
    launch GUI_MultiVMC_Torque.py --run <condition>
    publish /VC_ref_pos from HERE -- no GUI:
        tip goal at the straight-arm tip, so the linear spring pulls NOTHING
        --settle-gauss   the Gaussians ramp in and push the arm off the obstacle
        --ramp-speed     the tip goal walks to the target, slowly
        --settle-arrive  the arm follows it and stops
    Ctrl+C #1 to the torque node -> marks "target reached"
    --window                     -> the measurement window
    Ctrl+C #2                    -> writes outputs/<folder>/<condition>.{npz,png}
    kill dynamixel_node   (it cannot change operating mode while running, and it
                           holds /dev/ttyUSB0, so the bus is unreachable until it
                           is gone)
    home: extended position mode, slow profile, back to MultiGoHome.HOME_POSITION,
          verified tick for tick -- a miss that is a multiple of 4096 is a lost
          turn, and homing on a lost turn puts the arm 15 cm from where the ticks
          say it is
    hand back in current mode and go again

Any failure aborts the sweep: the motors are left holding, the driver is not
restarted, and the conditions already recorded are named. Nothing is recorded
from a zero that was not verified.

Terminal 1 is NOT used -- this script owns the driver. Run the load cell in its
own terminal first (`python3 ../LoadCell/arduino_force.py`) for `_button`.
"""

import argparse
import importlib.util
import os
import queue
import signal
import subprocess
import sys
import threading
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
_VMC  = os.path.join(_ROOT, "VMC")
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.join(_ROOT, "Kinematics"))

import rclpy                                        # noqa: E402
from rclpy.executors import SingleThreadedExecutor    # noqa: E402
from rclpy.node import Node                           # noqa: E402
from std_msgs.msg import Float64MultiArray            # noqa: E402

# The driver's launch line, kept identical to start_arm.sh so the motor
# calibration cannot drift between the two ways of starting the arm.
DRIVER_CMD = [
    "ros2", "run", "dynamixel_interface", "dynamixel_node", "--ros-args",
    "-p", "baudrate:=1000000",
    "-p", "motor_ids:=[11,12,13,14,15,16,17,18,19]",
    "-p", "control_mode:=torque",
    "-p", "use_kt:=true",
    "-p", "kt:=0.000354",
]


# ── helpers ───────────────────────────────────────────────────────────────────

def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def pump(stream, tag, q):
    for line in stream:
        line = line.rstrip("\n")
        print(f"[{tag}] {line}", flush=True)
        q.put((tag, line))
    q.put((tag, None))


def wait_for(q, needle, timeout, what):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            tag, line = q.get(timeout=0.2)
        except queue.Empty:
            continue
        if line is None:
            raise SweepError(f"the {tag} process exited before {what}")
        if needle in line:
            return
    raise SweepError(f"timed out after {timeout:.0f} s waiting for {what}")


class SweepError(RuntimeError):
    """Anything that must stop the sweep rather than be recorded around."""


# ── the goal publisher (this is what the GUI used to be) ──────────────────────

class GoalPublisher(Node):
    """Publishes /VC_ref_pos at a steady rate and listens for the driver.

    The GUI publishes only when a goal is dragged, which is exactly what makes a
    hand-driven run unrepeatable. Here the reference is asserted continuously
    from a single array, so the goal is a number in this file rather than
    wherever the mouse happened to stop.
    """

    def __init__(self, goal_start, rate_hz=20.0):
        super().__init__("sweep_goal_pub")
        self.goal = np.array(goal_start, dtype=float)      # (N, 3)
        self._pub = self.create_publisher(Float64MultiArray, "/VC_ref_pos", 10)
        self._joints_seen = threading.Event()
        self.create_subscription(Float64MultiArray, "/joint_positions",
                                 lambda _m: self._joints_seen.set(), 10)
        self.create_timer(1.0 / rate_hz, self._tick)

    def _tick(self):
        msg = Float64MultiArray()
        msg.data = self.goal.reshape(-1).tolist()
        self._pub.publish(msg)

    def wait_for_driver(self, timeout):
        """The torque node computes from q = 0 until the driver speaks. Starting
        a run before then would drive the arm from a pose it is not in."""
        if not self._joints_seen.wait(timeout):
            raise SweepError(f"no /joint_positions within {timeout:.0f} s -- "
                             "is the driver up and the bus connected?")

    def forget_driver(self):
        self._joints_seen.clear()


def spin_in_background(node):
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    return ex


# ── the sweep ─────────────────────────────────────────────────────────────────

class Sweep:
    def __init__(self, args, cfg, home):
        self.a    = args
        self.cfg  = cfg
        self.home = home                    # the MultiGoHome module
        self.recorded = []
        self.driver = None

        names = [c.name for c in cfg.VIRTUAL_COMPONENTS]
        self.tip = names.index("lin_tip")

        # Start: every goal where the config puts it, EXCEPT the tip, which
        # starts at the straight-arm tip so the linear spring pulls nothing at
        # all while the Gaussians do their work. The config's own start would
        # have it 5.5 cm short of the rest pose, i.e. 1.65 N from the first tick.
        from VMC_utils.ConfigHelpers import rest_pose
        self.goal_start = cfg.DEFAULT_START.copy()
        self.goal_start[self.tip] = rest_pose(1.0)

        self.goal_end = self.goal_start.copy()
        self.goal_end[self.tip] = [args.target_x, args.target_y, cfg.TIP_GOAL_Z]

        self.travel = float(np.linalg.norm(
            self.goal_end[self.tip] - self.goal_start[self.tip]))

    # ── phases ────────────────────────────────────────────────────────────────

    def start_driver(self):
        print("[sweep] starting dynamixel_node (torque mode)", flush=True)
        self.driver = subprocess.Popen(DRIVER_CMD, stdout=subprocess.DEVNULL,
                                       stderr=subprocess.DEVNULL)
        self.pub.forget_driver()
        self.pub.wait_for_driver(self.a.driver_timeout)
        print("[sweep] driver up, /joint_positions flowing", flush=True)

    def stop_driver(self):
        if self.driver is None or self.driver.poll() is not None:
            self.driver = None
            return
        print("[sweep] stopping dynamixel_node to free the bus", flush=True)
        self.driver.terminate()
        try:
            self.driver.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.driver.kill()
            self.driver.wait(timeout=5)
        self.driver = None
        time.sleep(self.a.bus_settle)      # let the port close before reopening

    def go_home(self):
        """Position control, slowly, back to HOME_POSITION -- verified."""
        gh = self.home
        gh.PROFILE_VELOCITY = self.a.home_velocity
        print(f"[sweep] homing at PROFILE_VELOCITY={gh.PROFILE_VELOCITY} "
              f"(~{gh.PROFILE_VELOCITY * 0.229:.1f} rpm)", flush=True)
        ph, pk = gh.init_bus()
        try:
            gh.set_extended_position_mode(ph, pk)
            gh.send_home_abs(ph, pk)
            # Judge by arrival, not by the clock: at ~2 rpm a wound motor needs
            # far longer than MultiGoHome's own 15 s default.
            gh.wait_until_home(ph, pk, timeout=self.a.home_timeout)
            if not gh.report_home(ph, pk):
                raise SweepError("the arm is not at the commanded home "
                                 "(see the table above)")
            # Hand back in CURRENT mode. The driver rewrites the operating mode
            # at startup without disabling torque first, and mode is an EEPROM
            # register, so that write is rejected unless the motors are already
            # where it wants them.
            gh.set_current_mode_zero(ph, pk)
        finally:
            ph.closePort()
        time.sleep(self.a.settle_home)

    def record(self, condition):
        """One condition: torque node up, scripted goal, mark, window, save."""
        q = queue.Queue()
        torque = subprocess.Popen(
            [sys.executable, os.path.join(_VMC, "GUI_MultiVMC_Torque.py"),
             "--config", self.a.config, "--run", condition],
            cwd=_HERE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1)
        threading.Thread(target=pump, args=(torque.stdout, "torque", q),
                         daemon=True).start()
        try:
            wait_for(q, "VMCTorqueNode started", self.a.startup_timeout,
                     "the torque node")

            # From here the torque node is driving the arm. The tip goal is at
            # the rest tip, so only the Gaussians act.
            self.pub.goal = self.goal_start.copy()
            print(f"[sweep] {condition}: Gaussians ramping, tip goal inert, "
                  f"settling {self.a.settle_gauss:.0f} s", flush=True)
            time.sleep(self.a.settle_gauss)

            self.ramp_goal(condition)

            print(f"[sweep] {condition}: settling {self.a.settle_arrive:.0f} s",
                  flush=True)
            time.sleep(self.a.settle_arrive)

            print(f"[sweep] {condition}: marking", flush=True)
            torque.send_signal(signal.SIGINT)
            print(f"[sweep] {condition}: measurement window "
                  f"{self.a.window:.0f} s", flush=True)
            time.sleep(self.a.window)

            print(f"[sweep] {condition}: saving", flush=True)
            torque.send_signal(signal.SIGINT)
            torque.wait(timeout=self.a.save_timeout)
            if torque.returncode not in (0, None):
                raise SweepError(f"the torque node exited {torque.returncode}")
        except subprocess.TimeoutExpired:
            raise SweepError("the torque node did not finish saving in time")
        finally:
            if torque.poll() is None:
                torque.kill()

        out = os.path.join("outputs", getattr(self.cfg, "OUTPUT_FOLDER", ""),
                           f"{condition}.npz")
        if not os.path.exists(os.path.join(_HERE, out)):
            raise SweepError(f"{out} was not written")
        self.recorded.append(condition)

    def ramp_goal(self, condition):
        """Walk the tip goal to the target at a fixed speed, in real time."""
        speed = self.a.ramp_speed / 100.0                  # cm/s -> m/s
        secs  = self.travel / speed
        print(f"[sweep] {condition}: tip goal "
              f"{np.round(self.goal_start[self.tip], 3)} -> "
              f"{np.round(self.goal_end[self.tip], 3)}  "
              f"({self.travel * 100:.1f} cm at {self.a.ramp_speed:.1f} cm/s "
              f"= {secs:.0f} s)", flush=True)
        t0 = time.perf_counter()
        while True:
            frac = min(1.0, (time.perf_counter() - t0) / secs)
            g = self.goal_start.copy()
            g[self.tip] = (self.goal_start[self.tip] * (1.0 - frac)
                           + self.goal_end[self.tip] * frac)
            self.pub.goal = g
            if frac >= 1.0:
                return
            time.sleep(0.02)

    # ── driver ────────────────────────────────────────────────────────────────

    def run(self, conditions):
        rclpy.init()
        self.pub = GoalPublisher(self.goal_start)
        ex = spin_in_background(self.pub)
        try:
            for i, cond in enumerate(conditions, 1):
                print(f"\n{'=' * 70}\n[sweep] {i}/{len(conditions)}  {cond} "
                      f"(strength {self.cfg.GAUSS_CONDITIONS[cond]:.0f})\n{'=' * 70}",
                      flush=True)
                self.start_driver()
                self.record(cond)
                self.stop_driver()
                self.go_home()
            print(f"\n[sweep] DONE. recorded: {', '.join(self.recorded)}", flush=True)
            return 0
        except SweepError as exc:
            print(f"\n[sweep] ABORTED -- {exc}", flush=True)
            print(f"[sweep] recorded before the failure: "
                  f"{', '.join(self.recorded) or '(none)'}", flush=True)
            print("[sweep] the driver is not being restarted; check the arm "
                  "before running anything else.", flush=True)
            return 1
        except KeyboardInterrupt:
            print("\n[sweep] interrupted", flush=True)
            return 130
        finally:
            self.stop_driver()
            ex.shutdown()
            self.pub.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


# ── entry point ───────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default="_button", help="config variant (default: _button)")
    p.add_argument("--only", nargs="+", metavar="RUN",
                   help="record only these conditions, in this order")
    p.add_argument("--dry-run", action="store_true",
                   help="print the plan and the timings, touch no hardware")

    p.add_argument("--target-x", type=float, default=-0.30, help="[m] (default: -0.30)")
    p.add_argument("--target-y", type=float, default=0.0,   help="[m] (default: 0)")

    p.add_argument("--settle-gauss",  type=float, default=15.0,
                   help="[s] Gaussians alone before the tip goal moves (default: 15)")
    p.add_argument("--ramp-speed",    type=float, default=2.0,
                   help="[cm/s] tip goal travel speed (default: 2)")
    p.add_argument("--settle-arrive", type=float, default=15.0,
                   help="[s] after the goal arrives, before the mark (default: 15)")
    p.add_argument("--window",        type=float, default=20.0,
                   help="[s] measurement window (default: 20)")
    p.add_argument("--home-velocity", type=int,   default=10,
                   help="PROFILE_VELOCITY for homing, 0.229 rpm each (default: 10)")
    p.add_argument("--settle-home",   type=float, default=5.0,
                   help="[s] after homing (default: 5)")

    p.add_argument("--startup-timeout", type=float, default=180.0)
    p.add_argument("--driver-timeout",  type=float, default=30.0)
    p.add_argument("--home-timeout",    type=float, default=180.0)
    p.add_argument("--save-timeout",    type=float, default=180.0)
    p.add_argument("--bus-settle",      type=float, default=2.0,
                   help="[s] after killing the driver, before reopening the port")
    return p.parse_args()


def main():
    a   = parse_args()
    cfg = load_module(os.path.join(_HERE, f"GUI_MultiVMC_Config{a.config}.py"),
                      f"cfg{a.config}")
    gh  = load_module(os.path.join(_VMC, "MultiGoHome.py"), "multigohome")

    known = list(cfg.GAUSS_CONDITIONS)
    conditions = a.only or known
    bad = [c for c in conditions if c not in cfg.GAUSS_CONDITIONS]
    if bad:
        sys.exit(f"[ERROR] not conditions of {cfg.__name__}: {', '.join(bad)}\n"
                 f"        this config knows: {', '.join(known)}")
    if getattr(cfg, "COLLECTED_DATA", False):
        sys.exit(f"[ERROR] {cfg.__name__} has COLLECTED_DATA = True -- "
                 "nothing would be saved. Set it False to record.")

    sweep = Sweep(a, cfg, gh)
    per = (3 + 6.3 + a.settle_gauss + sweep.travel * 100 / a.ramp_speed
           + a.settle_arrive + a.window + 8
           + 1.15 * 60 / (a.home_velocity * 0.229) + a.settle_home)
    print(f"[sweep] {cfg.__name__}: {len(conditions)} condition(s) -- "
          f"{', '.join(conditions)}")
    print(f"[sweep] start  {np.round(sweep.goal_start[sweep.tip], 3)}  "
          f"(rest tip -- the linear spring pulls nothing)")
    print(f"[sweep] target {np.round(sweep.goal_end[sweep.tip], 3)}  "
          f"({sweep.travel * 100:.1f} cm at {a.ramp_speed:.1f} cm/s)")
    print(f"[sweep] ~{per:.0f} s per condition, ~{per * len(conditions) / 60:.1f} min "
          f"total (homing assumes ~1.15 turns of wind)")

    if a.dry_run:
        print("[sweep] --dry-run: stopping here, nothing touched.")
        return 0
    return sweep.run(conditions)


if __name__ == "__main__":
    sys.exit(main())
