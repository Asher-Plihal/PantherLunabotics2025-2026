import copy
import os
import sys
import time
import threading
import robot_params

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

    def shutdown(self) -> None:
        """Stop the sender thread."""
        self._running = False
