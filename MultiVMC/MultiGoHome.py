"""
Go Home Script — Multi-Segment Soft Robot (3 motors, absolute ticks)
====================================================================

Uses Dynamixel SDK directly (no ROS node required) to:
- read absolute encoder ticks (ADDR_PRESENT_POSITION)
- command absolute goal ticks (ADDR_GOAL_POSITION) in extended position mode
- wait until within threshold
- optionally switch back to torque/current mode so ROS dynamixel_node can take over

Usage:
  python3 MultiVMC/MultiGoHome.py --read
  python3 MultiVMC/MultiGoHome.py
  python3 MultiVMC/MultiGoHome.py --no-torque-mode
"""

import argparse
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
POSITION_THRESHOLD = 10   # ticks
PROFILE_VELOCITY   = 40   # ~9 rpm (slow/safe)

# --- HOME POSITION (ABSOLUTE ticks) -----------------------------------------
# Update: put robot in desired home pose, run with --read, copy values here.
HOME_POSITION = {
 11: 1949,
 12: 2216,
 13: 537,
 14: 1803,
 15: 1376,
 16: 3440,
 17: 3030,
 18: 2121,
 19: 2384,
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

def set_extended_position_mode(ph, pk):
    for mid in MOTOR_IDS:
        pk.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 0)
        pk.write1ByteTxRx(ph, mid, ADDR_OPERATING_MODE, MODE_EXT_POSITION)
        pk.write4ByteTxRx(ph, mid, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY)
        pk.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 1)
    print("Motors set to extended position mode")


def send_home_abs(ph, pk):
    for mid in MOTOR_IDS:
        if mid not in HOME_POSITION:
            raise KeyError(f"Missing HOME_POSITION for motor {mid}")
        pk.write4ByteTxRx(ph, mid, ADDR_GOAL_POSITION, int(HOME_POSITION[mid]))
    print("Home (absolute ticks) command sent")


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


def set_current_mode_zero(ph, pk):
    for mid in MOTOR_IDS:
        pk.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 0)
        pk.write1ByteTxRx(ph, mid, ADDR_OPERATING_MODE, MODE_CURRENT_CONTROL)
        pk.write2ByteTxRx(ph, mid, ADDR_GOAL_CURRENT, 0)
        pk.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 1)
    print("Motors set to current/torque mode (zero current)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--read", action="store_true",
                        help="Read and print current absolute positions (ticks) and exit.")
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--no-torque-mode", action="store_true",
                        help="Do not switch back to current mode at the end.")
    args = parser.parse_args()

    ph, pk = init_bus()
    try:
        print(f"Opening {DEVICE_PORT} @ {BAUDRATE}, protocol {PROTOCOL}")
        if args.read:
            pos = read_positions_abs(ph, pk)
            print("Current absolute positions (ticks):")
            for mid in MOTOR_IDS:
                print(f" {mid}: {pos[mid] if pos[mid] is not None else 'READ ERROR'},")
            print("\nCopy these into HOME_POSITION for your desired home pose.")
            return

        set_extended_position_mode(ph, pk)
        send_home_abs(ph, pk)
        print("Moving to home...")

        if wait_until_home(ph, pk, timeout=args.timeout):
            print("Home reached.")
        else:
            print("Warning: timeout before reaching home.")

        if not args.no_torque_mode:
            set_current_mode_zero(ph, pk)

    finally:
        ph.closePort()


if __name__ == "__main__":
    main()