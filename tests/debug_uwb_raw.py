#!/usr/bin/env python3
"""
Debug script to see raw serial output from both UWB tags.
Prints the first 20 lines from each port, then stops.
"""

import serial
import threading
import time
from typing import Optional

LEFT_PORT  = "/dev/ttyUSB0"
RIGHT_PORT = "/dev/ttyUSB1"
BAUD       = 115200

_lines_received = {"LEFT": 0, "RIGHT": 0}
_lock = threading.Lock()


def read_raw(port: str, label: str) -> None:
    """Read and print raw serial lines."""
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = BAUD
        ser.timeout = 1.0
        ser.dtr = False
        ser.open()
        ser.rts = True
        time.sleep(0.1)
        ser.rts = False
        time.sleep(1.0)
        print(f"[{label}] connected on {port}")
    except serial.SerialException as e:
        print(f"[{label}] FAILED to open {port}: {e}")
        return

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            
            with _lock:
                _lines_received[label] += 1
                count = _lines_received[label]
            
            print(f"[{label:5s}] {count:2d}: {line}")
            
            if count >= 30:
                break
        except Exception as e:
            print(f"[{label}] error: {e}")
            break


threading.Thread(target=read_raw, args=(LEFT_PORT,  "LEFT"),  daemon=True).start()
time.sleep(3.0)
threading.Thread(target=read_raw, args=(RIGHT_PORT, "RIGHT"), daemon=True).start()

print("Reading raw serial from both tags...\n")
time.sleep(30)
print("\nDone.")
