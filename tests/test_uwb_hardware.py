"""
Minimal UWB hardware test — HaoruTech ULA1 (ESP32 + DWM1000).

Before running: plug in both UWB modules and check what ports the Jetson assigned:
    dmesg | grep ttyUSB
Then update BASE_PORT and TAG_PORT below to match.
/dev/ttyUSB0 is reserved for LiDAR — UWB modules will likely be ttyUSB1 and ttyUSB2,
but plug order determines the number so always verify with dmesg first.

If output looks like garbage, BAUD is probably wrong (default is 115200 for ULA1).

Expected output format (tag):
    mc 0f 00000663 000005a3 00000512 000004cb 095f c1 0 a0:0

Note: unlike the ULM3, the ULA1 does NOT output a $K tag-position line.
The packet has 4 RANGE fields, no MCU timestamp, and a single-digit debug field.

Set BASE_PORT to the anchor (fixed, known position).
Set TAG_PORT to the tag (the one you move around at different distances).

Run:  python test_uwb_hardware.py
Stop: Ctrl+C
"""

import threading
import time
import serial

BASE_PORT = "/dev/ttyUSB1"   # anchor — fixed reference
TAG_PORT  = "/dev/ttyUSB0"   # tag — move this one to test distances
BAUD      = 115200


def read_loop(port: str, label: str) -> None:
    try:
        ser = serial.Serial(port, BAUD, timeout=1.0)
        ser.dtr = False  # GPIO0 HIGH → normal boot (not download mode)
        ser.rts = True   # EN LOW → hold in reset
        time.sleep(0.1)
        ser.rts = False  # EN HIGH → release reset, ESP32 boots
        time.sleep(1)  # wait for ESP32 to fully boot and start ranging
        print(f"[{label}] connected on {port}")
    except serial.SerialException as e:
        print(f"[{label}] failed to open {port}: {e}")
        return

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                print(f"[{label}] {line}")
        except Exception as e:
            print(f"[{label}] error: {e}")
            break


threading.Thread(target=read_loop, args=(BASE_PORT, "BASE"), daemon=True).start()
time.sleep(3.0)
threading.Thread(target=read_loop, args=(TAG_PORT,  "TAG"),  daemon=True).start()

print("Reading from both UWB sensors — Ctrl+C to stop\n")
try:
    threading.Event().wait()
except KeyboardInterrupt:
    print("\nStopped.")
