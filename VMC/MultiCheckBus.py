"""
Bus health check -- read-only, no ROS, no motion.

    python3 VMC/MultiCheckBus.py            # ping + status + 100 read attempts each
    python3 VMC/MultiCheckBus.py --reads 500

Run this when the dynamixel node logs `SyncRead failed: -3001/-3002` or when
homing reports `timeout before reaching home`. Both mean the bus is not
answering reliably, and neither is a control problem: on a failed read the node
publishes nothing at all, so GUI_MultiVMC_Torque.py keeps commanding torque at
330 Hz from the last joint state it saw. Stale feedback in a closed loop is
what a buzzing, oscillating arm looks like.

Result codes: 0 ok, -3001 RX timeout (no answer), -3002 RX corrupt (garbled),
-1001 TX fail. A motor that fails intermittently is usually a connector or a
power problem; one that fails always is usually in an error state -- see the
hardware-error column.
"""

import argparse
import sys

from dynamixel_sdk import PacketHandler, PortHandler

DEVICE_PORT = "/dev/ttyUSB0"
BAUDRATE    = 1000000
PROTOCOL    = 2.0
MOTOR_IDS   = [11, 12, 13, 14, 15, 16, 17, 18, 19]

ADDR_HARDWARE_ERROR = 70    # 1 byte
ADDR_PRESENT_POSITION = 132  # 4 bytes
ADDR_PRESENT_VOLTAGE = 144  # 2 bytes, units of 0.1 V
ADDR_PRESENT_TEMP    = 146  # 1 byte, degC

# XL330 hardware error status bits
ERROR_BITS = {0: "input voltage", 2: "overheating", 3: "encoder",
              4: "electrical shock", 5: "overload"}


def decode_errors(byte):
    if byte == 0:
        return "-"
    return ", ".join(name for bit, name in ERROR_BITS.items() if byte & (1 << bit)) \
           or f"raw 0x{byte:02x}"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reads", type=int, default=100,
                    help="position reads per motor, to measure reliability (default 100)")
    args = ap.parse_args()

    ph, pk = PortHandler(DEVICE_PORT), PacketHandler(PROTOCOL)
    if not ph.openPort():
        sys.exit(f"cannot open {DEVICE_PORT}")
    if not ph.setBaudRate(BAUDRATE):
        sys.exit(f"cannot set baudrate {BAUDRATE}")

    print(f"{DEVICE_PORT} @ {BAUDRATE}, protocol {PROTOCOL}, {args.reads} reads per motor\n")
    print(f"{'motor':<7}{'ping':<8}{'fail rate':<12}{'position':<11}"
          f"{'volt':<8}{'temp':<7}{'hardware error'}")

    bad = []
    for mid in MOTOR_IDS:
        _, ping_res, _ = pk.ping(ph, mid)
        ping_ok = ping_res == 0

        fails = 0
        pos = None
        for _ in range(args.reads):
            p, res, err = pk.read4ByteTxRx(ph, mid, ADDR_PRESENT_POSITION)
            if res != 0 or err != 0:
                fails += 1
            else:
                pos = p
        rate = fails / args.reads

        volt, vres, _ = pk.read2ByteTxRx(ph, mid, ADDR_PRESENT_VOLTAGE)
        temp, tres, _ = pk.read1ByteTxRx(ph, mid, ADDR_PRESENT_TEMP)
        herr, hres, _ = pk.read1ByteTxRx(ph, mid, ADDR_HARDWARE_ERROR)

        flag = "" if (ping_ok and rate == 0 and hres == 0 and herr == 0) else "   <--"
        print(f"{mid:<7}{'ok' if ping_ok else 'FAIL':<8}"
              f"{f'{rate:.0%} ({fails}/{args.reads})':<12}"
              f"{pos if pos is not None else '--':<11}"
              f"{f'{volt/10:.1f} V' if vres == 0 else '--':<8}"
              f"{f'{temp} C' if tres == 0 else '--':<7}"
              f"{decode_errors(herr) if hres == 0 else 'unreadable'}{flag}")
        if flag:
            bad.append(mid)

    ph.closePort()
    print()
    if bad:
        print(f"suspect motor(s): {bad}")
        print("  intermittent failures  -> connector/daisy-chain at that motor, or supply voltage")
        print("  always fails           -> that motor is down; check its hardware error above")
        print("  every motor fails      -> port, baudrate, or power to the whole chain")
    else:
        print("bus is clean: every motor pinged, no failed reads, no hardware errors.")
        print("If the arm still oscillates, the feedback path is fine and the cause is elsewhere.")


if __name__ == "__main__":
    main()
