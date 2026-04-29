"""
Minimal UWB hardware test — HaoruTech ULA1 (ESP32 + DWM1000).

Before running: plug in both tag modules and check what ports the Jetson assigned:
    ls /dev/ttyUSB*
Then update T0_PORT and T1_PORT below to match.
Plug order determines the port number so always verify before running.

If output looks like garbage, BAUD is probably wrong (default is 115200 for ULA1).

Expected output format (tag):
    mc 0f 00000663 000005a3 00000512 000004cb 095f c1 0 t0:0
    mc 0f 00000663 000005a3 00000512 000004cb 095f c1 0 t1:0

The last field shows which tag produced the packet: t0 or t1.

Note: unlike the ULM3, the ULA1 does NOT output a $K tag-position line.
The packet has 4 RANGE fields, no MCU timestamp, and a single-digit debug field.

Run:  python test_uwb_hardware.py
Stop: Ctrl+C
"""

import threading
import time
import serial

T0_PORT = "/dev/ttyUSB0"   # tag 0 (LEFT)
T1_PORT = "/dev/ttyUSB1"   # tag 1 (RIGHT)
BAUD    = 115200


def read_loop(port: str, label: str) -> None:
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = BAUD
        ser.timeout = 1.0
        ser.dtr = False  # keep GPIO0 HIGH — prevents ESP32 booting into download mode
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
            if line:
                print(f"[{label}] {line}")
        except Exception as e:
            print(f"[{label}] error: {e}")
            break


threading.Thread(target=read_loop, args=(T0_PORT, "T0"), daemon=True).start()
threading.Thread(target=read_loop, args=(T1_PORT, "T1"), daemon=True).start()

print("Reading from both tags — Ctrl+C to stop\n")
try:
    threading.Event().wait()
except KeyboardInterrupt:
    print("\nStopped.")
