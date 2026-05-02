"""
UWB distance print test — connects to both tag serial ports and prints the
four raw distances (left/right tag to anchor A/B) to the terminal.

Before running:
    dmesg | grep ttyUSB   # find port assignments
    /dev/ttyUSB0 is reserved for LiDAR; tags are typically ttyUSB1 and ttyUSB2.

Run:  python tests/test_uwb_localizer.py
Stop: Ctrl+C
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from library.uwb_localizer import UWBLocalizer

LEFT_PORT  = "/dev/ttyUWB_LEFT"
RIGHT_PORT = "/dev/ttyUWB_RIGHT"

# Required by the constructor but unused here — _last_d is raw hardware distances,
# not derived from anchor positions. Only needed once we call get_pose().
AX, AY = 0.5842, 0.0
BX, BY = 0.0,   0.7874

TAG_SEP        = 0.4445
FORWARD_OFFSET = 0.0

PRINT_HZ = 10  # how many times per second to print


def main() -> None:
    """Connect to both UWB tags and print the four distances at PRINT_HZ."""
    print(f"Connecting — LEFT: {LEFT_PORT}  RIGHT: {RIGHT_PORT}")
    loc = UWBLocalizer(
        AX, AY, BX, BY,
        tag_sep=TAG_SEP,
        forward_offset=FORWARD_OFFSET,
        use_hardware=True,
        left_port=LEFT_PORT,
        right_port=RIGHT_PORT,
    )
    loc.start()
    print("Connected. Press Ctrl+C to stop.\n")

    interval = 1.0 / PRINT_HZ
    try:
        while True:
            t0 = time.monotonic()
            if loc._last_d is not None:
                d_LA, d_RA, d_LB, d_RB = loc._last_d
                print(
                    f"d_LA={d_LA:.3f} m  d_RA={d_RA:.3f} m  "
                    f"d_LB={d_LB:.3f} m  d_RB={d_RB:.3f} m"
                )
            else:
                print("waiting for data...")
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, interval - elapsed))
    except KeyboardInterrupt:
        pass
    finally:
        loc.stop()
        print("\nStopped.")


if __name__ == "__main__":
    main()
