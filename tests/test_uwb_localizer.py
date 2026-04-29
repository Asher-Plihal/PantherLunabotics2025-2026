"""
UWB Localizer live test with dashboard — connects to both tag serial ports,
runs the localizer in a background thread, and displays position + full
trilateration overlay in PantherDashboard.

Before running:
    dmesg | grep ttyUSB   # find port assignments
    /dev/ttyUSB0 is reserved for LiDAR; tags are typically ttyUSB1 and ttyUSB2.

Run:  python tests/test_uwb_localizer.py
Stop: Q / Escape / close window
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from library.uwb_localizer import UWBLocalizer
from library.panther_dashboard import PantherDashboard

LEFT_PORT  = "/dev/ttyUWB_LEFT"
RIGHT_PORT = "/dev/ttyUWB_RIGHT"

# Anchor positions (metres, arena coordinates) — update when measured
AX, AY = 0.5842, 0.0   # Anchor A
BX, BY = 0.0,   0.7874  # Anchor B

TAG_SEP        = 0.4445  # left-to-right tag separation (metres)
FORWARD_OFFSET = 0.0     # tag midpoint ahead of robot centre (metres)

# Overlay colours
_BLUE   = (100, 160, 255)
_LBLUE  = ( 60, 110, 200)
_RED    = (255,  90,  90)
_LRED   = (200,  50,  50)
_YELLOW = (255, 220,   0)
_ORANGE = (255, 140,   0)


class UWBLocalizerTester:
    """
    Connects to both UWB tags, runs the localizer in a background thread,
    and renders the live position and trilateration geometry in PantherDashboard.
    """

    def __init__(self) -> None:
        """Open serial ports, start the localizer background thread, open the dashboard."""
        print(f"Connecting — LEFT: {LEFT_PORT}  RIGHT: {RIGHT_PORT}")
        self._loc = UWBLocalizer(
            AX, AY, BX, BY,
            tag_sep=TAG_SEP,
            forward_offset=FORWARD_OFFSET,
            use_hardware=True,
            left_port=LEFT_PORT,
            right_port=RIGHT_PORT,
        )
        self._loc.start()
        print("Connected. Opening dashboard (check your desktop display)...")
        self._dash = PantherDashboard("Panther Dashboard — UWB Live", fullscreen=False)
        print("Dashboard open. Close the window or press Q to stop.")

    def run(self) -> None:
        """Dashboard loop — renders at 30 Hz until window is closed."""
        while True:
            self._update_overlay()
            if not self._dash.update():
                break
        self._loc.stop()

    def _update_overlay(self) -> None:
        """Read current localizer state and push it to the dashboard each frame."""
        loc  = self._loc
        dash = self._dash

        # Trilateration circles — one per tag per anchor
        if loc._last_d is not None:
            d_LA, d_RA, d_LB, d_RB = loc._last_d
            dash.add_circle("cLA", AX, AY, d_LA, color=_BLUE,  width=1)
            dash.add_circle("cLB", BX, BY, d_LB, color=_LBLUE, width=1)
            dash.add_circle("cRA", AX, AY, d_RA, color=_RED,   width=1)
            dash.add_circle("cRB", BX, BY, d_RB, color=_LRED,  width=1)
            dash.put("d_LA", f"{d_LA:.3f} m")
            dash.put("d_RA", f"{d_RA:.3f} m")
            dash.put("d_LB", f"{d_LB:.3f} m")
            dash.put("d_RB", f"{d_RB:.3f} m")

        # Both candidate intersection points per tag (before pair selection)
        if loc._left_candidates is not None:
            for i, (cpx, cpy) in enumerate(loc._left_candidates):
                dash.add_point(f"L{i}", cpx, cpy, color=_BLUE, radius=4)
        if loc._right_candidates is not None:
            for i, (cpx, cpy) in enumerate(loc._right_candidates):
                dash.add_point(f"R{i}", cpx, cpy, color=_RED,  radius=4)

        # Anchors
        dash.add_point("Anchor A", AX, AY, color=_YELLOW, radius=8)
        dash.add_point("Anchor B", BX, BY, color=_ORANGE, radius=8)

        # Resolved tag positions
        if loc._left_tag:
            lx, ly = loc._left_tag
            dash.add_point("Tag L", lx, ly, color=_BLUE, radius=6, label_offset=(9, 6))
        if loc._right_tag:
            rx, ry = loc._right_tag
            dash.add_point("Tag R", rx, ry, color=_RED,  radius=6, label_offset=(9, -18))

        # Tag separation — sanity check vs TAG_SEP
        if loc._left_tag and loc._right_tag:
            lx, ly = loc._left_tag
            rx, ry = loc._right_tag
            sep = ((rx - lx) ** 2 + (ry - ly) ** 2) ** 0.5
            dash.put("tag sep", f"{sep:.3f} m  (exp {TAG_SEP:.4f} m)")

        # Robot pose
        pos = loc.get_pose()
        if pos:
            dash.set_robot(pos.x, pos.y, pos.heading)
            dash.put("x",       f"{pos.x:.3f} m")
            dash.put("y",       f"{pos.y:.3f} m")
            dash.put("heading", f"{pos.heading:.1f} deg")
        else:
            dash.put("position", "NO SOLUTION")


if __name__ == "__main__":
    UWBLocalizerTester().run()
