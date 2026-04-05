from __future__ import annotations
import copy
import os
import sys
import time
import threading
from typing import TYPE_CHECKING
import robot_params

if TYPE_CHECKING:
    from library.uwb_localizer import UWBLocalizer

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))


class Dashboard:
    """
    Robot-side dashboard subsystem. Sends robot state to mission control
    via server.send_telemetry() so the laptop can render it in PantherDashboard.

    Rate-limited to 10 Hz to avoid flooding the TCP connection.

    _always_on() is called every cycle — put anything that should always
    be transmitted here (e.g. fixed targets, mode info).
    """

    _SEND_RATE_HZ = 10
    _SEND_PERIOD  = 1.0 / _SEND_RATE_HZ

    def __init__(self, server):
        """Initialize and start the sender thread if fieldDashboard is enabled."""
        self._server = server
        self._last_send = 0.0
        self._pending: dict = {}
        self._lock = threading.Lock()

        if not robot_params.RobotConfig.fieldDashboard:
            print("[Dashboard] Disabled")
            self._running = False
            return

        self._uwb_localizer: UWBLocalizer | None = None
        self._running = True
        threading.Thread(target=self._run, daemon=True).start()
        print("[Dashboard] Sender started")

    def _run(self) -> None:
        """Send pending state to mission control at the configured rate."""
        while self._running:
            now = time.monotonic()
            if now - self._last_send >= self._SEND_PERIOD:
                self._always_on()
                with self._lock:
                    payload = copy.deepcopy(self._pending)
                if payload:
                    self._server.send_telemetry(payload)
                self._last_send = now
            time.sleep(0.005)

    def _always_on(self) -> None:
        """Called every send cycle. Add permanent field markers or telemetry here."""
        # ── Fixed field markers ────────────────────────────────────────────────
        self.set_target(5.38, 0.6, 90.0)   # berm centre — face toward excavation zone

        # ── UWB overlay ───────────────────────────────────────────────────────
        if robot_params.RobotConfig.uwbDashboard and self._uwb_localizer is not None:
            self.set_uwb_overlay(self._uwb_localizer)

    # ── public API ────────────────────────────────────────────────────────────

    def set_robot(self, x: float, y: float, heading_deg: float = 0.0) -> None:
        """Send the robot's current position and heading to the dashboard."""
        if not self._running:
            return
        with self._lock:
            self._pending["pose"] = {"x": x, "y": y, "heading": heading_deg}

    def set_target(self, x: float, y: float, heading_deg: float = 0.0) -> None:
        """Send the target position and heading the robot is navigating toward."""
        if not self._running:
            return
        with self._lock:
            self._pending["target"] = {"x": x, "y": y, "heading": heading_deg}

    def put(self, key: str, value) -> None:
        """Send a telemetry key-value entry to the dashboard panel."""
        if not self._running:
            return
        with self._lock:
            if "telemetry" not in self._pending:
                self._pending["telemetry"] = {}
            self._pending["telemetry"][key] = str(value)

    def set_circles(self, circles: dict) -> None:
        """
        Send overlay circles to the field dashboard.

        circles is a dict of {label: {"cx", "cy", "r", "color", "width"}} where
        cx/cy are arena coordinates (metres), r is radius (metres),
        color is an RGB list, and width is line width in pixels.

        Replaces the previous circles dict entirely. No-op when disabled.
        """
        if not self._running:
            return
        with self._lock:
            self._pending["circles"] = circles

    def set_points(self, points: dict) -> None:
        """
        Send overlay points to the field dashboard.

        points is a dict of {label: {"x", "y", "color", "radius", "label_offset"}} where
        x/y are arena coordinates (metres), color is an RGB list, radius is pixels,
        and label_offset is an optional [dx, dy] pixel offset for the label.

        Replaces the previous points dict entirely. No-op when disabled.
        """
        if not self._running:
            return
        with self._lock:
            self._pending["points"] = points

    def set_uwb_source(self, localizer: UWBLocalizer) -> None:
        """Register a UWBLocalizer to be sampled automatically every send cycle."""
        self._uwb_localizer = localizer

    def set_uwb_overlay(self, localizer: UWBLocalizer) -> None:
        """
        Package UWBLocalizer internal state as generic circles and points and send
        to the field dashboard. No-op when disabled.

        Draws the four trilateration circles (one per tag per anchor), the candidate
        intersection points, anchor markers, and the resolved tag positions.
        """
        if not self._running:
            return
        circles: dict = {}
        points:  dict = {}
        ax, ay = localizer.ax, localizer.ay
        bx, by = localizer.bx, localizer.by

        if localizer._last_d is not None:
            d_LA, d_RA, d_LB, d_RB = localizer._last_d
            circles["cLA"] = {"cx": ax, "cy": ay, "r": d_LA, "color": [100, 160, 255], "width": 1}
            circles["cLB"] = {"cx": bx, "cy": by, "r": d_LB, "color": [ 60, 110, 200], "width": 1}
            circles["cRA"] = {"cx": ax, "cy": ay, "r": d_RA, "color": [255,  90,  90], "width": 1}
            circles["cRB"] = {"cx": bx, "cy": by, "r": d_RB, "color": [200,  50,  50], "width": 1}

        if localizer._left_candidates is not None:
            for i, (cpx, cpy) in enumerate(localizer._left_candidates):
                points[f"L{i}"] = {"x": cpx, "y": cpy, "color": [100, 160, 255], "radius": 4}
        if localizer._right_candidates is not None:
            for i, (cpx, cpy) in enumerate(localizer._right_candidates):
                points[f"R{i}"] = {"x": cpx, "y": cpy, "color": [255,  90,  90], "radius": 4}

        points["Anchor A"] = {"x": ax, "y": ay, "color": [255, 220,   0], "radius": 8}
        points["Anchor B"] = {"x": bx, "y": by, "color": [255, 140,   0], "radius": 8}

        if localizer._left_tag:
            points["Tag L"] = {"x": localizer._left_tag[0],  "y": localizer._left_tag[1],
                               "color": [100, 160, 255], "radius": 6, "label_offset": [9, 6]}
        if localizer._right_tag:
            points["Tag R"] = {"x": localizer._right_tag[0], "y": localizer._right_tag[1],
                               "color": [255,  90,  90], "radius": 6, "label_offset": [9, -18]}

        self.set_circles(circles)
        self.set_points(points)

    def shutdown(self) -> None:
        """Stop the sender thread."""
        self._running = False
