#!/usr/bin/env python3
"""
One condition, start to finish, from one terminal.

    python3 run_condition.py weak
    python3 run_condition.py strong --config _person
    python3 run_condition.py medium --hold 20

Replaces terminals 3 and 4: starts the torque node and the GUI together, waits
for the arm to settle at the config's fixed goal, marks, holds, saves, and shuts
both down. Nothing is dragged, so every run in a sweep aims at exactly the same
point and the conditions are comparable.

    torque node + GUI up
      -> wait for "first reference received" (the GUI is publishing, t = 0)
      -> --settle seconds for the arm to reach the goal and stop moving
      -> Ctrl+C #1 to the torque node: marks "target reached"
      -> --hold seconds: THIS is the measurement window
      -> Ctrl+C #2: ends the run and writes outputs/<folder>/<run>.{npz,png}
      -> GUI closed

Terminal 1 (hardware) and terminal 2 (load cell, `_button` only) are unchanged.

Ctrl+C here at any point stops both children without saving a partial run --
the checkpoint from AUTOSAVE_S is what survives that.
"""

import argparse
import os
import queue
import signal
import subprocess
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_VMC  = os.path.join(os.path.dirname(_HERE), "VMC")


def _pump(stream, tag, q):
    """Forward a child's output, tagged, and hand each line to the waiter."""
    for line in stream:
        line = line.rstrip("\n")
        print(f"[{tag}] {line}", flush=True)
        q.put(line)
    q.put(None)


def _wait_for(q, needle, timeout, what):
    """Block until a line containing `needle` shows up, or give up."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            line = q.get(timeout=0.2)
        except queue.Empty:
            continue
        if line is None:
            raise RuntimeError(f"the torque node exited before {what} "
                               f"-- see its output above")
        if needle in line:
            return
    raise TimeoutError(f"timed out after {timeout:.0f} s waiting for {what}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", help="condition to record: no_gauss, weak, medium, strong")
    ap.add_argument("--config", default="_button", help="config variant (default: _button)")
    ap.add_argument("--settle", type=float, default=20.0,
                    help="[s] after the GUI starts publishing, before the mark (default: 20)")
    ap.add_argument("--hold", type=float, default=10.0,
                    help="[s] measurement window, between the mark and the end (default: 10)")
    ap.add_argument("--startup-timeout", type=float, default=180.0,
                    help="[s] allowed for the sympy Jacobian build (default: 180)")
    ap.add_argument("--no-gui", action="store_true",
                    help="do not start the GUI (then nothing publishes and the run never begins "
                         "-- only useful for checking the torque node comes up)")
    args = ap.parse_args()

    env = dict(os.environ)
    torque = subprocess.Popen(
        [sys.executable, os.path.join(_VMC, "GUI_MultiVMC_Torque.py"),
         "--config", args.config, "--run", args.run],
        cwd=_HERE, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1)

    q = queue.Queue()
    threading.Thread(target=_pump, args=(torque.stdout, "torque", q), daemon=True).start()

    gui = None

    def shutdown(msg):
        print(f"\n[run] {msg}", flush=True)
        for p in (gui, torque):
            if p is not None and p.poll() is None:
                p.kill()

    try:
        # The torque node refuses an unknown --run in about a second, so a bad
        # name fails here rather than after the Jacobian build.
        if torque.poll() is not None:
            print("[run] torque node refused to start -- see above", flush=True)
            return 1

        print(f"[run] building Jacobians, up to {args.startup_timeout:.0f} s...", flush=True)
        _wait_for(q, "VMCTorqueNode started", args.startup_timeout, "it was ready")

        if args.no_gui:
            print("[run] --no-gui: nothing will publish, so no run starts. Stopping.", flush=True)
            shutdown("done")
            return 0

        gui = subprocess.Popen(
            [sys.executable, os.path.join(_VMC, "GUI_MultiVMC.py"),
             "--config", args.config, "--run", args.run],
            cwd=_HERE, env=env)

        # The GUI publishes DEFAULT_START as soon as it is up; that message is
        # t = 0 and the moment the arm starts moving towards the goal.
        _wait_for(q, "first reference received", args.startup_timeout, "the GUI to publish")

        print(f"[run] settling for {args.settle:.0f} s...", flush=True)
        time.sleep(args.settle)

        print("[run] marking 'target reached'", flush=True)
        torque.send_signal(signal.SIGINT)

        print(f"[run] holding {args.hold:.0f} s -- this is the measurement window", flush=True)
        time.sleep(args.hold)

        print("[run] ending the run and saving", flush=True)
        torque.send_signal(signal.SIGINT)

        # The save renders a figure as well, so give it room. The .npz is
        # written first, so even a timeout here costs the plot, not the run.
        torque.wait(timeout=180)
        print(f"[run] torque node exited ({torque.returncode})", flush=True)

    except KeyboardInterrupt:
        shutdown("interrupted -- stopping both, nothing saved "
                 "(the .autosave.npz checkpoint is what survives)")
        return 130
    except (TimeoutError, RuntimeError) as exc:
        shutdown(f"{exc} -- stopping both")
        return 1
    except subprocess.TimeoutExpired:
        shutdown("the torque node did not finish saving in time -- stopping")
        return 1
    finally:
        if gui is not None and gui.poll() is None:
            gui.terminate()
            try:
                gui.wait(timeout=10)
            except subprocess.TimeoutExpired:
                gui.kill()
        if torque.poll() is None:
            torque.kill()

    print(f"[run] done: {args.run} -> outputs/<folder>/{args.run}.{{npz,png}}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
