"""
Minimal UWB hardware test — HaoruTech ULM3 (STM32 + DWM3000).

Before running: plug in both UWB modules and check what ports the Jetson assigned:
    dmesg | grep ttyUSB
Then update BASE_PORT and TAG_PORT below to match.
/dev/ttyUSB0 is reserved for LiDAR — UWB modules will likely be ttyUSB1 and ttyUSB2,
but plug order determines the number so always verify with dmesg first.

If output looks like garbage, BAUD is probably wrong (default is 115200 for ULM3).

Expected output format (tag):
    mc 0f 00000663 000005a3 ... 095f c1 00146fb7 a0:0
    $KT0, 1.69, 2.93, ...

LED status: green blink = tag ranging OK, red blink = no anchor response,
light blue blink = anchor connected to tag.

Set BASE_PORT to the anchor (fixed, known position).
Set TAG_PORT to the tag (the one you move around at different distances).

Run:  python test_uwb_hardware.py
Stop: Ctrl+C
"""

import threading
import serial

BASE_PORT = "/dev/ttyUSB1"   # anchor — fixed reference
TAG_PORT  = "/dev/ttyUSB2"   # tag — move this one to test distances
BAUD      = 115200


def read_loop(port: str, label: str) -> None:
    try:
        ser = serial.Serial(port, BAUD, timeout=1.0)
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
threading.Thread(target=read_loop, args=(TAG_PORT,  "TAG"),  daemon=True).start()

print("Reading from both UWB sensors — Ctrl+C to stop\n")
try:
    threading.Event().wait()
except KeyboardInterrupt:
    print("\nStopped.")
