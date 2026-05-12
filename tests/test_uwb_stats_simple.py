#!/usr/bin/env python3
"""
UWB ranging statistics test — reads raw serial from both tags without RTS/DTR toggles.
Just opens the ports and reads what's already running.
"""

import statistics
import threading
import time
from collections import defaultdict
from typing import Optional

import serial

LEFT_PORT  = "/dev/ttyUSB0"
RIGHT_PORT = "/dev/ttyUSB1"
BAUD       = 115200

_samples: dict[tuple[str, str], list[float]] = defaultdict(list)
_lock = threading.Lock()


def parse(line: str) -> Optional[tuple[Optional[float], Optional[float]]]:
    """Parse one ULA1 'mc' packet to (dist_A0, dist_A1) in metres."""
    parts = line.split()
    if len(parts) < 4 or parts[0] != "mc":
        return None
    try:
        mask = int(parts[1], 16)
        range0_hex = parts[2]
        range1_hex = parts[3]
        dist_a_mm = int(range0_hex, 16) if (mask & 0x01) and range0_hex != "ffffffff" else None
        dist_b_mm = int(range1_hex, 16) if (mask & 0x02) and range1_hex != "ffffffff" else None
        if dist_a_mm is None and dist_b_mm is None:
            return None
        return (dist_a_mm / 1000.0 if dist_a_mm is not None else None,
                dist_b_mm / 1000.0 if dist_b_mm is not None else None)
    except (ValueError, IndexError):
        return None


def read_loop(port: str, label: str) -> None:
    """Read serial, parse packets, accumulate samples."""
    try:
        ser = serial.Serial(port=port, baudrate=BAUD, timeout=1.0)
        print(f"[{label}] connected on {port}")
    except serial.SerialException as e:
        print(f"[{label}] failed to open {port}: {e}")
        return

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line.startswith("mc"):
                continue

            # Skip anchor-originated packets
            parts_line = line.split()
            if parts_line and not parts_line[-1].startswith("t"):
                continue

            result = parse(line)
            if not result:
                continue

            dist_a, dist_b = result
            parts_out = []
            if dist_a is not None:
                parts_out.append(f"A0={dist_a:.3f}")
                with _lock:
                    _samples[(label, "A0")].append(dist_a)
            if dist_b is not None:
                parts_out.append(f"A1={dist_b:.3f}")
                with _lock:
                    _samples[(label, "A1")].append(dist_b)
            print(f"[{label}]  {', '.join(parts_out)} m")
        except Exception as e:
            print(f"[{label}] error: {e}")
            break


def print_stats() -> None:
    """Print accumulated per-anchor statistics."""
    with _lock:
        if not _samples:
            print("No samples collected.")
            return
        print("\n--- Stats ---")
        for (label, anchor), vals in sorted(_samples.items()):
            if len(vals) < 2:
                print(f"[{label}] {anchor}: only {len(vals)} sample(s) — not enough for stats")
                continue
            mean   = statistics.mean(vals)
            median = statistics.median(vals)
            stdev  = statistics.stdev(vals)
            print(
                f"[{label}] {anchor}:  n={len(vals):4d}  "
                f"mean={mean:.3f} m  median={median:.3f} m  "
                f"stdev={stdev * 1000:.0f} mm  "
                f"min={min(vals):.3f}  max={max(vals):.3f}"
            )


threading.Thread(target=read_loop, args=(LEFT_PORT,  "LEFT"),  daemon=True).start()
time.sleep(0.5)
threading.Thread(target=read_loop, args=(RIGHT_PORT, "RIGHT"), daemon=True).start()

print("Collecting UWB ranging stats — Ctrl+C to stop and print summary\n")
try:
    threading.Event().wait()
except KeyboardInterrupt:
    print_stats()
    print("\nStopped.")
