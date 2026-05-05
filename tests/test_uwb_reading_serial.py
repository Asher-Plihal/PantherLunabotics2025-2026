"""
UWB serial reading + parsing test — HaoruTech ULA1 (ESP32 + DWM1000).

Tests the serial reading and parsing logic from uwb_localizer.py in isolation,
without running any localization math. Prints parsed distances in metres so you
can verify the hardware output is being read and decoded correctly.

Partial-data tolerant: each of the four distances (d_LA, d_LB, d_RA, d_RB) is
tracked independently. Values display as '---' until that slot receives a reading,
so you can see interference or anchor dropouts immediately.

Before running: plug in both tag modules and check port assignments:
    dmesg | grep ttyUSB
Then update LEFT_PORT and RIGHT_PORT below to match.

Run:  python tests/test_uwb_reading_serial.py
Stop: Ctrl+C
"""

import threading
import time
from typing import Optional

import serial

LEFT_PORT  = "/dev/ttyUSB0"   # Tag LEFT  (T0)
RIGHT_PORT = "/dev/ttyUSB1"   # Tag RIGHT (T1)
BAUD       = 115_200
PRINT_HZ   = 10

# Latest distance per tag/anchor slot — updated independently by each reader thread.
_lock   = threading.Lock()
_latest: dict[str, dict[str, Optional[float]]] = {
    "LEFT":  {"A": None, "B": None},
    "RIGHT": {"A": None, "B": None},
}


def _parse_partial(line: str) -> tuple[Optional[float], Optional[float]]:
    """
    Parse one ULA1 'mc' packet, returning (dist_A_m, dist_B_m).

    Either value may be None if that anchor's bit is not set in the mask or its
    range field is 0xffffffff (invalid).  Both None means the line has no usable data.
    """
    parts = line.split()
    if len(parts) < 4 or parts[0] != "mc":
        return None, None
    try:
        mask = int(parts[1], 16)
        r0   = parts[2]
        r1   = parts[3]
        dist_a = int(r0, 16) / 1000.0 if (mask & 0x01) and r0 != "ffffffff" else None
        dist_b = int(r1, 16) / 1000.0 if (mask & 0x02) and r1 != "ffffffff" else None
        return dist_a, dist_b
    except (ValueError, IndexError):
        return None, None


def _reader(port: str, label: str) -> None:
    """Read mc packets from one serial port and update _latest for that tag."""
    try:
        ser = serial.Serial()
        ser.port     = port
        ser.baudrate = BAUD
        ser.timeout  = 1.0
        ser.dtr      = False  # keep GPIO0 HIGH — prevents ESP32 booting into download mode
        ser.open()
        ser.rts = True   # EN LOW → hold in reset
        time.sleep(0.1)
        ser.rts = False  # EN HIGH → release, ESP32 boots into ranging firmware
        time.sleep(1.0)  # wait for boot
        print(f"[{label}] connected on {port}")
    except serial.SerialException as e:
        print(f"[{label}] failed to open {port}: {e}")
        return

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line.startswith("mc"):
                continue
            dist_a, dist_b = _parse_partial(line)
            with _lock:
                if dist_a is not None:
                    _latest[label]["A"] = dist_a
                if dist_b is not None:
                    _latest[label]["B"] = dist_b
        except Exception as e:
            print(f"[{label}] error: {e}")
            break


def _fmt(val: Optional[float]) -> str:
    """Format a distance value, or '---' if not yet received."""
    return f"{val:.3f} m" if val is not None else "  ---  "


threading.Thread(target=_reader, args=(LEFT_PORT,  "LEFT"),  daemon=True).start()
threading.Thread(target=_reader, args=(RIGHT_PORT, "RIGHT"), daemon=True).start()

print("Reading UWB serial — Ctrl+C to stop\n")
interval = 1.0 / PRINT_HZ
try:
    while True:
        t0 = time.monotonic()
        with _lock:
            d_LA = _latest["LEFT"]["A"]
            d_LB = _latest["LEFT"]["B"]
            d_RA = _latest["RIGHT"]["A"]
            d_RB = _latest["RIGHT"]["B"]
        print(
            f"d_LA={_fmt(d_LA)}  d_RA={_fmt(d_RA)}  "
            f"d_LB={_fmt(d_LB)}  d_RB={_fmt(d_RB)}"
        )
        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, interval - elapsed))
except KeyboardInterrupt:
    print("\nStopped.")
