"""
Go Home Script — Multi-Segment Soft Robot (3 motors, absolute ticks)
====================================================================

Uses Dynamixel SDK directly (no ROS node required) to:
- read absolute encoder ticks (ADDR_PRESENT_POSITION)
- command absolute goal ticks (ADDR_GOAL_POSITION) in extended position mode
- wait until within threshold
- optionally switch back to torque/current mode so ROS dynamixel_node can take over

Usage:
  python3 VMC/MultiGoHome.py --read
  python3 VMC/MultiGoHome.py
  python3 VMC/MultiGoHome.py --no-torque-mode
"""

import argparse
import glob
import os
import signal
import sys
import time
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# --- Hardware settings -------------------------------------------------------
BAUDRATE    = 1000000
DEVICE_PORT = "/dev/ttyUSB0"
PROTOCOL    = 2.0

# Motor IDs 
MOTOR_IDS = [11, 12, 13, 14, 15, 16, 17, 18, 19] 

# --- Control table addresses (XL330-M288) -----------------------------------
ADDR_TORQUE_ENABLE    = 64 # enable/disable
ADDR_OPERATING_MODE   = 11 # current control/extended position control (see below)
ADDR_GOAL_POSITION    = 116 
ADDR_GOAL_CURRENT     = 102
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_VELOCITY = 112

# Operating modes
MODE_CURRENT_CONTROL = 0
MODE_EXT_POSITION    = 4  # "extended position control" for XL330 series

# Motion tuning
# 150 ticks = 0.69 mm of tendon = about 9 mm at the tip. A tendon-loaded servo
# settles with a steady-state error of ~55 ticks in position mode, so the old
# 10-tick window rejected a perfectly good homing. A lost turn is 4096 ticks
# (151 mm at the tip, 45x the residual), so this still separates the two easily.
POSITION_THRESHOLD = 150  # ticks
PROFILE_VELOCITY   = 40   # ~9 rpm (slow/safe)
TICKS_PER_TURN     = 4096 # one full revolution -- a miss that is a multiple of
                          # this is a turn error, not a mechanical failure

# --- HOME POSITION (ABSOLUTE ticks) -----------------------------------------
# Update: put robot in desired home pose, run with --read, copy values here.
HOME_POSITION = {
 11: 537,
 12: 3159,
 13: 1454,
 14: 2095,
 15: 1718,
 16: 3576,
 17: 1330,
 18: 393,
 19: 83,
}

def init_bus():
    ph = PortHandler(DEVICE_PORT)
    pk = PacketHandler(PROTOCOL)

    if not ph.openPort():
        raise RuntimeError(f"Cannot open {DEVICE_PORT}")
    if not ph.setBaudRate(BAUDRATE):
        raise RuntimeError(f"Cannot set baudrate {BAUDRATE}")

    return ph, pk

def read_positions_abs(ph, pk):
    positions = {}
    for mid in MOTOR_IDS:
        pos, result, error = pk.read4ByteTxRx(ph, mid, ADDR_PRESENT_POSITION)
        if result == COMM_SUCCESS and error == 0:
            # interpret as signed 32-bit
            if pos > 0x7FFFFFFF:
                pos -= 0x100000000
            positions[mid] = int(pos)
        else:
            positions[mid] = None
    return positions

def set_extended_position_mode(ph, pk, velocity=None):
    velocity = PROFILE_VELOCITY if velocity is None else int(velocity)
    for mid in MOTOR_IDS:
        pk.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 0)
        pk.write1ByteTxRx(ph, mid, ADDR_OPERATING_MODE, MODE_EXT_POSITION)
        pk.write4ByteTxRx(ph, mid, ADDR_PROFILE_VELOCITY, velocity)
        pk.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 1)
    print(f"[home] motors in extended position mode, PROFILE_VELOCITY={velocity} "
          f"(~{velocity * 0.229:.1f} rpm)")


def send_home_abs(ph, pk):
    for mid in MOTOR_IDS:
        if mid not in HOME_POSITION:
            raise KeyError(f"Missing HOME_POSITION for motor {mid}")
        pk.write4ByteTxRx(ph, mid, ADDR_GOAL_POSITION, int(HOME_POSITION[mid]))
    print("[home] goal ticks sent")


def wait_until_home(ph, pk, timeout=10.0):
    time.sleep(0.5)
    t0 = time.time()
    while time.time() - t0 < timeout:
        pos = read_positions_abs(ph, pk)
        ok = True
        for mid in MOTOR_IDS:
            if pos[mid] is None:
                ok = False
                break
            if abs(pos[mid] - HOME_POSITION[mid]) > POSITION_THRESHOLD:
                ok = False
                break
        if ok:
            return True
        time.sleep(0.05)
    return False


def home_error(ph, pk):
    """
    Per motor: (present - HOME_POSITION) in ticks, and whether that miss is a
    whole number of turns.

    Extended position control (mode 4) does take multi-turn goals, so the tick
    values in HOME_POSITION are commanded exactly as written -- but the
    revolution count behind them lives in the servo's RAM, not in the encoder.
    A power cycle or a reboot re-derives the position from the single-turn
    absolute encoder, so ticks captured in one session can mean a pose one or
    more turns away in the next. That miss is silent unless it is checked for,
    which is what this is: a delta of ~4096 is 2*pi in the wrong place, not a
    motor that failed to arrive.
    """
    pos = read_positions_abs(ph, pk)
    out = {}
    for mid in MOTOR_IDS:
        if pos[mid] is None:
            out[mid] = (None, None, False)
            continue
        delta = pos[mid] - HOME_POSITION[mid]
        turns = round(delta / TICKS_PER_TURN)
        is_turn = turns != 0 and abs(delta - turns * TICKS_PER_TURN) <= POSITION_THRESHOLD
        out[mid] = (pos[mid], delta, is_turn)
    return out


def tip_error_mm(deltas):
    """What the residual ticks mean where it matters: at the tip.

    Ticks are not comparable across the arm -- the same tick error on a
    proximal motor moves the tip further than on a distal one -- so the table
    ends with the one number that is."""
    try:
        import numpy as np
        from FK_Multi import FK_Multi
    except Exception:
        return None
    q = np.array([deltas.get(m, 0) for m in MOTOR_IDS], dtype=float) / 4096.0 * 2 * np.pi
    home = np.asarray(FK_Multi(np.zeros(9), 1.0)).reshape(-1)[:3]
    here = np.asarray(FK_Multi(q, 1.0)).reshape(-1)[:3]
    return float(np.linalg.norm(here - home) * 1000.0)


def report_home(ph, pk):
    """Print where every motor actually landed. True when all are within
    POSITION_THRESHOLD of the commanded ticks."""
    errs = home_error(ph, pk)
    ok = True
    print(f"\n{'motor':>6} {'commanded':>10} {'present':>10} {'delta':>8}")
    deltas = {}
    for mid in MOTOR_IDS:
        present, delta, is_turn = errs[mid]
        if present is None:
            print(f"{mid:>6} {HOME_POSITION[mid]:>10} {'READ ERROR':>10} {'--':>8}")
            ok = False
            continue
        note = ""
        if is_turn:
            note = f"  <-- OFF BY {round(delta / TICKS_PER_TURN):+d} TURN(S), not a missed move"
            ok = False
        elif abs(delta) > POSITION_THRESHOLD:
            note = "  <-- did not arrive"
            ok = False
        deltas[mid] = delta
        print(f"{mid:>6} {HOME_POSITION[mid]:>10} {present:>10} {delta:>+8}{note}")

    tip_mm = tip_error_mm(deltas)
    if tip_mm is not None:
        print(f"{'':>6} {'':>10} {'':>10} {'':>8}   -> {tip_mm:.1f} mm at the tip "
              f"(tolerance {POSITION_THRESHOLD} ticks)")
    return ok


def set_current_mode_zero(ph, pk):
    for mid in MOTOR_IDS:
        pk.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 0)
        pk.write1ByteTxRx(ph, mid, ADDR_OPERATING_MODE, MODE_CURRENT_CONTROL)
        pk.write2ByteTxRx(ph, mid, ADDR_GOAL_CURRENT, 0)
        pk.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 1)
    print("[home] handed back in current mode, zero current")


# ─────────────────────────────────────────────────────────────────────────────
# Bus ownership
#
# Homing talks to the servos over the raw serial port, and dynamixel_node holds
# that port for as long as it runs -- so whoever wants to home has to take the
# bus off it first. Kept here rather than in a caller because getting it wrong
# fails as a bare "device reports readiness to read but returned no data",
# minutes into a run.
# ─────────────────────────────────────────────────────────────────────────────

def port_holders(dev=DEVICE_PORT):
    """PIDs with `dev` open, read straight from /proc.

    pyserial's `exclusive=True` is an advisory flock, and the C++ driver opens
    the port without taking one -- so an exclusive open SUCCEEDS while the
    driver still has it. /proc is the only check that actually answers the
    question.
    """
    holders = set()
    for fd in glob.glob("/proc/[0-9]*/fd/*"):
        try:
            if os.readlink(fd) == dev:
                holders.add(int(fd.split("/")[2]))
        except OSError:
            continue
    return sorted(h for h in holders if h != os.getpid())


def describe(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as fh:
            cmd = fh.read().replace(b"\x00", b" ").decode(errors="replace").strip()
        return f"{pid} ({cmd[:60]})"
    except OSError:
        return str(pid)


def release_port(dev=DEVICE_PORT, timeout=12.0, log=print):
    """Take the bus off whoever holds it, and return only once it is free.

    `ros2 run` execs the node as a child, so signalling the wrapper can leave
    the node orphaned and still holding the port -- the wrapper exiting proves
    nothing. The only success condition is the bus actually being free, so
    every signal is judged by re-reading /proc, and the whole process group is
    signalled rather than the pid.
    """
    held = port_holders(dev)
    if not held:
        return
    log(f"[home] {dev} is held by {', '.join(describe(p) for p in held)} -- releasing it")
    for sig, grace in ((signal.SIGTERM, timeout), (signal.SIGKILL, 10.0)):
        for pid in port_holders(dev):
            try:
                pgid = os.getpgid(pid)
            except ProcessLookupError:
                continue
            try:
                # The group, not the pid: `ros2 run` execs the node as a child,
                # and start_arm.sh is its parent, so signalling the group is
                # what actually ends that terminal. Unless it is OUR group --
                # everything launched from one terminal -- in which case that
                # would kill the caller mid-homing.
                if pgid == os.getpgid(0):
                    os.kill(pid, sig)
                else:
                    os.killpg(pgid, sig)
            except (ProcessLookupError, PermissionError):
                try:
                    os.kill(pid, sig)
                except (ProcessLookupError, PermissionError):
                    pass
        deadline = time.time() + grace
        while time.time() < deadline:
            if not port_holders(dev):
                log(f"[home] {dev} free")
                return
            time.sleep(0.3)
        log(f"[home] still held after {sig.name}, escalating")
    still = port_holders(dev)
    raise RuntimeError(f"{dev} is still held by "
                       f"{', '.join(describe(p) for p in still)} -- kill it by hand")


def home_sequence(velocity=PROFILE_VELOCITY, timeout=60.0, hand_back=True, log=print):
    """Extended position control, slowly, back to HOME_POSITION -- verified.

    Raises rather than returning False when the arm is not where it was told to
    go: in current mode the arm goes limp and whatever pose it is actually in
    becomes the model's zero the moment dynamixel_node starts, so a home that
    is a turn out must never be handed over. Left in position mode the motors
    keep holding.
    """
    ph, pk = init_bus()
    try:
        set_extended_position_mode(ph, pk, velocity)
        send_home_abs(ph, pk)
        log("[home] moving...")
        # Judge by arrival, not by the clock: a wound motor at 9 rpm needs far
        # longer than the old 15 s default.
        reached = wait_until_home(ph, pk, timeout=timeout)
        if not report_home(ph, pk):
            raise RuntimeError("the arm is not at the commanded home (see the table above)")
        log(f"[home] every motor within {POSITION_THRESHOLD} ticks of the commanded value"
            + ("" if reached else "  (arrived after the wait timed out)"))
        if hand_back:
            # Hand back in CURRENT mode. The driver rewrites the operating mode
            # at startup without disabling torque first, and mode is an EEPROM
            # register, so that write is rejected unless the motors are already
            # in the mode it wants.
            set_current_mode_zero(ph, pk)
    finally:
        ph.closePort()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--read", action="store_true",
                        help="Read and print current absolute positions (ticks) and exit.")
    parser.add_argument("--timeout", type=float, default=60.0,
                        help="seconds to wait for the arm to arrive (default 60)")
    parser.add_argument("--velocity", type=int, default=PROFILE_VELOCITY,
                        metavar="V",
                        help=f"PROFILE_VELOCITY, 0.229 rpm per unit "
                             f"(default {PROFILE_VELOCITY}, ~{PROFILE_VELOCITY * 0.229:.1f} rpm)")
    parser.add_argument("--no-torque-mode", action="store_true",
                        help="Do not switch back to current mode at the end.")
    args = parser.parse_args()

    print(f"Opening {DEVICE_PORT} @ {BAUDRATE}, protocol {PROTOCOL}")
    if args.read:
        ph, pk = init_bus()
        try:
            pos = read_positions_abs(ph, pk)
            print("Current absolute positions (ticks):")
            for mid in MOTOR_IDS:
                print(f" {mid}: {pos[mid] if pos[mid] is not None else 'READ ERROR'},")
            print("\nCopy these into HOME_POSITION for your desired home pose.")
        finally:
            ph.closePort()
        return

    try:
        home_sequence(velocity=args.velocity, timeout=args.timeout,
                      hand_back=not args.no_torque_mode)
    except RuntimeError as exc:
        # Do NOT hand over. In current mode the arm goes limp, and whatever
        # pose it is actually in becomes the zero reference the moment
        # dynamixel_node starts -- so a home that is a turn out silently
        # becomes the model's "straight arm". Left in position mode the motors
        # keep holding, and start_arm.sh (set -e) stops here.
        print(f"\n[FAIL] {exc}. Motors left holding in extended position mode "
              "-- NOT handed over to current mode.\n"
              "       A delta that is a multiple of 4096 means the turn count was "
              "lost since HOME_POSITION was captured:\n"
              "       power-cycle, re-place the arm by hand, and re-capture with --read.")
        sys.exit(1)


if __name__ == "__main__":
    main()