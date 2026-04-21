"""
UWB serial reading + parsing test — HaoruTech ULA1 (ESP32 + DWM1000).

Tests the serial reading and parsing logic from uwb_localizer.py in isolation,
without running any localization math. Prints parsed distances in metres so you
can verify the hardware output is being read and decoded correctly.

Before running: plug in both tag modules and check port assignments:
    dmesg | grep ttyUSB
Then update LEFT_PORT and RIGHT_PORT below to match.
/dev/ttyUSB0 is reserved for LiDAR.
Expected output format (ULA1 packet — 4 RANGE fields, no timestamp, no $K line):
    [LEFT]  raw: mc 0f 00000663 000005a3 00000512 000004cb 095f c1 0 a0:0
    [LEFT]  parsed: A0 = 1.635 m, A1 = 1.443 m
    [RIGHT] raw: mc 0f 00000512 000004cb 00000663 000005a3 095f c1 0 a0:1
    [RIGHT] parsed: A0 = 1.298 m, A1 = 1.227 m

Run:  python test_uwb_reading_serial.py
Stop: Ctrl+C
"""

import threading
import time
from typing import Optional

import serial

LEFT_PORT  = "/dev/ttyUSB0"   # Tag LEFT  (T0)
RIGHT_PORT = "/dev/ttyUSB1"   # Tag RIGHT (T1)
BAUD       = 115200


def parse(line: str) -> Optional[tuple[float, float]]:
    """
    Parse one ULM3 'mc' packet to (dist_anchor_A, dist_anchor_B) in metres.

    Validates the MASK field and rejects ffffffff (invalid) ranges.
    Returns None if the line is not a valid mc packet or either range is invalid.
    """
    parts = line.split()
    if len(parts) < 4 or parts[0] != "mc":
        return None
    try:
        mask = int(parts[1], 16)
        if (mask & 0x03) != 0x03:
            return None
        range0_hex = parts[2]
        range1_hex = parts[3]
        if range0_hex == "ffffffff" or range1_hex == "ffffffff":
            return None
        dist_a_mm = int(range0_hex, 16)
        dist_b_mm = int(range1_hex, 16)
        return dist_a_mm / 1000.0, dist_b_mm / 1000.0
    except (ValueError, IndexError):
        return None


def read_and_parse_loop(port: str, label: str) -> None:
    """Read serial lines, parse mc packets, and print raw + parsed output."""
    try:
        ser = serial.Serial(port, BAUD, timeout=1.0)
        time.sleep(1.5)  # let ESP32 finish whatever boot state it entered
        ser.dtr = False  # GPIO0 HIGH → normal boot mode
        ser.rts = True   # EN LOW → hold in reset
        time.sleep(0.1)
        ser.rts = False  # EN HIGH → release, ESP32 reboots into ranging firmware
        time.sleep(1.5)  # wait for clean boot and ranging init
        print(f"[{label}] connected on {port}")
    except serial.SerialException as e:
        print(f"[{label}] failed to open {port}: {e}")
        return

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            # Print all raw lines so we can see the full output
            print(f"[{label}] raw: {line}")

            # Only parse mc packets
            if line.startswith("mc"):
                result = parse(line)
                if result:
                    dist_a, dist_b = result
                    print(f"[{label}] parsed: A0 = {dist_a:.3f} m, A1 = {dist_b:.3f} m")
                else:
                    print(f"[{label}] parsed: INVALID (mask or range check failed)")

        except Exception as e:
            print(f"[{label}] error: {e}")
            break


threading.Thread(target=read_and_parse_loop, args=(LEFT_PORT,  "LEFT"),  daemon=True).start()
time.sleep(1.5)
threading.Thread(target=read_and_parse_loop, args=(RIGHT_PORT, "RIGHT"), daemon=True).start()

print("Reading and parsing UWB serial — Ctrl+C to stop\n")
try:
    threading.Event().wait()
except KeyboardInterrupt:
    print("\nStopped.")
