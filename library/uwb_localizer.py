"""
UWB Localizer — 2-anchor, 2-tag implementation.

Coordinate system: +X = east, +Y = north.
Heading: 0° = north, clockwise positive (90° = east).

Four measurements per cycle:
    d_LA, d_LB : TAG_LEFT  distances to Anchor A and Anchor B
    d_RA, d_RB : TAG_RIGHT distances to Anchor A and Anchor B

With two labeled tags and two anchors at known positions the system is
over-constrained (4 measurements, 3 unknowns). Absolute heading is derived
from UWB alone — no IMU required.

Usage:
    # Manual / test
    loc = UWBLocalizer(ax=0.0, ay=5.0, bx=2.5, by=5.0, tag_sep=0.6)
    pos = loc.update(d_LA=3.08, d_RA=3.27, d_LB=3.50, d_RB=3.23)
    if pos:
        print(pos.x, pos.y, pos.heading)

    # Hardware (ports passed directly to constructor, then call update() with no args)
    loc = UWBLocalizer(ax=0.0, ay=5.0, bx=2.5, by=5.0, tag_sep=0.6,
                       use_hardware=True, left_port="/dev/ttyUWB_LEFT", right_port="/dev/ttyUWB_RIGHT")
    pos = loc.update()

Run directly to test with manual values and visualise in PantherDashboard:
    python uwb_localizer.py
"""

import math
import sys
import os
import threading
import time
from typing import Optional

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from library.pid_drive import Position


class UWBLocalizer:
    """
    Computes robot (x, y, heading) from two UWB tags and two fixed anchors.

    Args:
        ax, ay:         Anchor A position in arena coordinates (metres).
        bx, by:         Anchor B position in arena coordinates (metres).
        tag_sep:        Left-to-right tag separation W (metres). Measure on robot.
        forward_offset: Distance the tag midpoint is ahead of robot centre (metres).
                        Positive = tags mounted forward of centre. Default 0.
    """

    DEFAULT_BAUD  = 115_200
    SEP_TOLERANCE = 0.08     # metres — acceptable deviation from tag_sep when selecting valid pair

    def __init__(
        self,
        ax: float,
        ay: float,
        bx: float,
        by: float,
        tag_sep: float,
        forward_offset: float = 0.0,
        use_hardware: bool = False,
        left_port: Optional[str] = None,
        right_port: Optional[str] = None,
        baud: int = DEFAULT_BAUD,
    ):
        """Store anchor/tag geometry and optionally open serial ports (see class docstring for args)."""
        self.ax = ax
        self.ay = ay
        self.bx = bx
        self.by = by
        self.tag_sep        = tag_sep
        self.forward_offset = forward_offset
        self._anchor_dist   = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)

        self._left_serial:  Optional[serial.Serial] = None
        self._right_serial: Optional[serial.Serial] = None
        self._position:     Optional[Position] = None

        self._thread:     Optional[threading.Thread] = None
        self._stop_event: threading.Event = threading.Event()

        # Last computed tag positions — stored for dashboard / debugging
        self._left_tag:  Optional[tuple[float, float]] = None
        self._right_tag: Optional[tuple[float, float]] = None

        # All four candidate points from the last trilateration (for visualisation)
        self._left_candidates:  Optional[tuple[tuple[float, float], tuple[float, float]]] = None
        self._right_candidates: Optional[tuple[tuple[float, float], tuple[float, float]]] = None

        # Last input distances (for drawing circles on the dashboard)
        self._last_d: Optional[tuple[float, float, float, float]] = None  # d_LA, d_RA, d_LB, d_RB

        if use_hardware:
            if left_port is None or right_port is None:
                raise ValueError("left_port and right_port are required when use_hardware=True")
            self.init_hardware(left_port, right_port, baud)

    def init_hardware(
        self,
        left_port:  str,
        right_port: str,
        baud:       int = DEFAULT_BAUD,
    ) -> None:
        """
        Open serial connections to TAG_LEFT and TAG_RIGHT.

        To find port names on the Jetson:
            dmesg | grep ttyUSB
        /dev/ttyUSB0 is reserved for LiDAR. Tags will typically be ttyUSB1 and
        ttyUSB2. Use udev rules for stable names (/dev/ttyUWB_LEFT, etc.).

        Args:
            left_port:  Serial port for TAG_LEFT  (e.g. "/dev/ttyUSB1").
            right_port: Serial port for TAG_RIGHT (e.g. "/dev/ttyUSB2").
            baud:       Baud rate — 115200 matches the ESP32/DWM1000 (ULA1) default.
        """
        self._left_serial  = self._open_port(left_port,  baud)
        self._right_serial = self._open_port(right_port, baud)

    @staticmethod
    def _open_port(port: str, baud: int) -> serial.Serial:
        """Open a serial port and reset the ESP32 into ranging firmware via the CH340 RTS line."""
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 1.0
        ser.dtr = False
        ser.open()
        ser.dtr = False  # re-apply — driver may briefly assert DTR during open(), pulling GPIO0 LOW
        time.sleep(0.5)  # let the capacitor transient fully dissipate before reset fires
        ser.rts = True   # EN LOW → hold in reset
        time.sleep(0.1)
        ser.rts = False  # EN HIGH → release; GPIO0 is now stable HIGH → normal boot
        time.sleep(1.5)  # wait for boot + ranging init
        return ser

    # -----------------------------------------------------------------------
    # Math — To find the robot position from the four distance measurements
    # -----------------------------------------------------------------------

    def _trilaterate(
        self,
        r1: float,
        r2: float,
    ) -> Optional[tuple[tuple[float, float], tuple[float, float]]]:
        """
        Intersect circle(Anchor A, r1) with circle(Anchor B, r2).

        Returns the two candidate intersection points (P1, P2), or None when the
        circles do not intersect (sensor noise or tag out of range).

        Args:
            r1: Distance from the tag to Anchor A (metres).
            r2: Distance from the tag to Anchor B (metres).
        """
        d = self._anchor_dist
        if d == 0.0:
            return None

        # Circles do not intersect
        if d > r1 + r2 or d < abs(r1 - r2):
            return None

        # Signed distance along A→B to the radical axis
        a = (r1 ** 2 - r2 ** 2 + d ** 2) / (2.0 * d)

        # Half-chord length perpendicular to A→B at the foot P
        h = math.sqrt(max(r1 ** 2 - a ** 2, 0.0))

        # Unit vector A→B
        ux = (self.bx - self.ax) / d
        uy = (self.by - self.ay) / d

        # Foot of perpendicular on the A→B line
        px = self.ax + a * ux
        py = self.ay + a * uy

        # Perpendicular unit vector (A→B rotated 90° CCW)
        qx = -uy
        qy =  ux

        p1 = (px + h * qx, py + h * qy)
        p2 = (px - h * qx, py - h * qy)
        return p1, p2

    def _select_valid_pair(
        self,
        left_candidates:  tuple[tuple[float, float], tuple[float, float]],
        right_candidates: tuple[tuple[float, float], tuple[float, float]],
    ) -> Optional[tuple[tuple[float, float], tuple[float, float]]]:
        """
        Test all 4 (LEFT, RIGHT) combinations and return the pair whose
        separation matches tag_sep within SEP_TOLERANCE.

        If two pairs pass (rare — occurs near the anchor diagonal), the one
        whose midpoint is closest to the previous position estimate is returned.
        Returns None if no pair matches.
        """
        valid = []
        for lp in left_candidates:
            for rp in right_candidates:
                sep = self._tag_separation(lp[0], lp[1], rp[0], rp[1])
                if abs(sep - self.tag_sep) < self.SEP_TOLERANCE:
                    valid.append((lp, rp))

        if not valid:
            return None
        if len(valid) == 1:
            return valid[0]

        # Tiebreak: pick the pair whose midpoint is closest to the last known position
        if self._position is not None:
            prev = self._position
            def dist_to_prev(pair):
                """Squared distance from pair midpoint to the last known position."""
                lp, rp = pair
                mx = (lp[0] + rp[0]) / 2.0
                my = (lp[1] + rp[1]) / 2.0
                return (mx - prev.x) ** 2 + (my - prev.y) ** 2
            return min(valid, key=dist_to_prev)

        # No previous position to tiebreak with — arbitrarily take the first valid pair.
        return valid[0]

    def _robot_center(
        self,
        lx: float, ly: float,
        rx: float, ry: float,
        heading_deg: float,
    ) -> tuple[float, float]:
        """
        Compute robot centre from the tag midpoint, applying the forward offset.

        The tag midpoint is ahead of robot centre by forward_offset metres along
        the robot's forward axis. Subtracting that offset gives the true centre.

        Args:
            lx, ly:      TAG_LEFT arena position (metres).
            rx, ry:      TAG_RIGHT arena position (metres).
            heading_deg: Robot heading (degrees, 0 = north, CW positive).

        Returns:
            (x, y) of robot centre in arena coordinates.
        """
        mx = (lx + rx) / 2.0
        my = (ly + ry) / 2.0

        # Forward unit vector at heading θ (0=north CW): (sin θ, cos θ)
        theta = math.radians(heading_deg)
        x = mx - self.forward_offset * math.sin(theta)
        y = my - self.forward_offset * math.cos(theta)
        return x, y

    def _compute_heading(
        self,
        lx: float, ly: float,
        rx: float, ry: float,
    ) -> float:
        """
        Derive robot heading from the TAG_LEFT → TAG_RIGHT lateral vector.

        At heading θ (0=north, CW positive) the right-lateral direction is
        (cos θ, −sin θ), so:
            vx = rx − lx = W·cos θ
            vy = ry − ly = −W·sin θ
            θ = atan2(−vy, vx)

        Returns heading in degrees, normalised to [0°, 360°).
        """
        vx = rx - lx
        vy = ry - ly
        theta_deg = math.degrees(math.atan2(-vy, vx)) % 360.0
        return theta_deg

    # -----------------------------------------------------------------------
    # Position update
    # -----------------------------------------------------------------------

    def update(
        self,
        d_LA: Optional[float] = None,
        d_RA: Optional[float] = None,
        d_LB: Optional[float] = None,
        d_RB: Optional[float] = None,
    ) -> Optional[Position]:
        """
        Compute and store the robot position from four UWB range measurements.

        Call with no arguments to read distances from the hardware serial ports
        (requires use_hardware=True at construction). Pass all four values
        explicitly to use synthetic or pre-parsed distances instead.

        Args:
            d_LA: TAG_LEFT  → Anchor A distance (metres).
            d_RA: TAG_RIGHT → Anchor A distance (metres).
            d_LB: TAG_LEFT  → Anchor B distance (metres).
            d_RB: TAG_RIGHT → Anchor B distance (metres).

        Returns:
            Position(x, y, heading) if a valid solution was found, else None.

        Raises:
            RuntimeError: if called with no arguments but hardware is not initialised.
        """
        if d_LA is None or d_RA is None or d_LB is None or d_RB is None:
            if not self._left_serial or not self._right_serial:
                raise RuntimeError("Pass distances explicitly or initialise hardware via use_hardware=True")
            left_ranges  = self._read_serial(self._left_serial)
            right_ranges = self._read_serial(self._right_serial)
            if left_ranges is None or right_ranges is None:
                return None
            d_LA, d_LB = left_ranges
            d_RA, d_RB = right_ranges

        if any(d <= 0 for d in (d_LA, d_RA, d_LB, d_RB)):
            return None

        self._last_d = (d_LA, d_RA, d_LB, d_RB)

        # Step 1 — Trilaterate each tag independently.
        # Each tag has two distance measurements (one to Anchor A, one to Anchor B).
        # Intersecting those two circles gives two candidate positions for that tag.
        left_candidates  = self._trilaterate(d_LA, d_LB)
        right_candidates = self._trilaterate(d_RA, d_RB)

        self._left_candidates  = left_candidates
        self._right_candidates = right_candidates

        if left_candidates is None or right_candidates is None:
            return None

        # Step 2 — Pick the correct (LEFT, RIGHT) candidate pair.
        # Each tag has 2 candidates, giving 4 possible combinations. Only the
        # pair whose separation matches the known physical tag spacing is valid.
        pair = self._select_valid_pair(left_candidates, right_candidates)
        if pair is None:
            return None

        (lx, ly), (rx, ry) = pair

        # Step 3 — Derive heading from the tag lateral vector.
        # The LEFT→RIGHT vector is perpendicular to the robot's forward axis,
        # so its direction directly encodes the robot's heading in arena coordinates.
        heading    = self._compute_heading(lx, ly, rx, ry)

        # Step 4 — Compute the robot centre from the tag midpoint.
        # The midpoint of the two tags may be offset forward of the robot centre
        # by forward_offset metres; this step subtracts that offset along the
        # heading direction to get the true robot centre position.
        cx, cy     = self._robot_center(lx, ly, rx, ry, heading)

        self._left_tag  = (lx, ly)
        self._right_tag = (rx, ry)
        self._position  = Position(cx, cy, heading)
        return self._position

    def get_pose(self) -> Optional[Position]:
        """Return the most recently computed Position, or None before first update()."""
        return self._position

    # -----------------------------------------------------------------------
    # Background thread — for hardware
    # -----------------------------------------------------------------------

    def start(self) -> None:
        """
        Start a daemon thread that calls update() continuously from hardware.
        Requires use_hardware=True (or a prior call to init_hardware()).
        PIDDrive reads the result via get_pose() without blocking on the math.
        """
        if not self._left_serial or not self._right_serial:
            raise RuntimeError("Hardware must be initialised before calling start()")
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="uwb_localizer")
        self._thread.start()

    def stop(self) -> None:
        """Signal the background thread to exit."""
        self._stop_event.set()

    def _run(self) -> None:
        """Loop calling update() from hardware until the stop event is set."""
        while not self._stop_event.is_set():
            self.update()
    
    # -----------------------------------------------------------------------
    # Math — helper functions
    # -----------------------------------------------------------------------

    def _tag_separation(
        self,
        lx: float, ly: float,
        rx: float, ry: float,
    ) -> float:
        """Euclidean distance between two tag candidate positions."""
        return math.sqrt((rx - lx) ** 2 + (ry - ly) ** 2)

    # -----------------------------------------------------------------------
    # Serial helpers
    # -----------------------------------------------------------------------

    def _read_serial(self, ser: serial.Serial) -> Optional[tuple[float, float]]:
        """
        Read lines from a serial port until a valid 'mc' packet is found and parsed.

        The ULA1 outputs only 'mc' packets continuously — no $K tag-position line.
        This method skips any non-'mc' lines and parses the first valid ranging packet.
        Returns None if no valid packet is found or the port errors.
        """
        try:
            for _ in range(5):  # read up to 5 lines to find a valid mc packet
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if line.startswith("mc"):
                    return self._parse(line)
        except serial.SerialException:
            return None
        return None

    @staticmethod
    def _parse(line: str) -> Optional[tuple[float, float]]:
        """
        Parse one ULA1 serial packet to (dist_anchor_A, dist_anchor_B) in metres.

        ULA1 format (ESP32 + DWM1000, from manual):
            mc 0f 00000663 000005a3 00000512 000004cb 095f c1 0 a0:0
               MASK RANGE0    RANGE1    RANGE2    RANGE3

        4 RANGE fields only (no RANGE4–7, no MCU timestamp, no DIAGNOSIS field).
        MASK is a bitmask of valid ranges: bit 0 = RANGE0, bit 1 = RANGE1, etc.
        RANGE values are hex millimetres. 0xffffffff = invalid/no anchor.

        For our 2-anchor setup we need RANGE0 (Anchor A) and RANGE1 (Anchor B).
        Returns None if either range is invalid.
        """
        parts = line.split()
        # Minimum: mc MASK RANGE0 RANGE1 ... (at least 4 fields for header + mask + 2 ranges)
        if len(parts) < 4 or parts[0] != "mc":
            return None
        try:
            mask = int(parts[1], 16)
            # Verify RANGE0 and RANGE1 are both valid per the mask
            if (mask & 0x03) != 0x03:
                return None
            range0_hex = parts[2]
            range1_hex = parts[3]
            # 0xffffffff means invalid/no anchor
            if range0_hex == "ffffffff" or range1_hex == "ffffffff":
                return None
            dist_a_mm = int(range0_hex, 16)
            dist_b_mm = int(range1_hex, 16)
            return dist_a_mm / 1000.0, dist_b_mm / 1000.0
        except (ValueError, IndexError):
            return None


# ---------------------------------------------------------------------------
# Synthetic test helper — compute distances from a known robot state
# ---------------------------------------------------------------------------

def _make_synthetic(
    robot_x:        float,
    robot_y:        float,
    heading_deg:    float,
    ax: float, ay: float,
    bx: float, by: float,
    tag_sep:        float,
    forward_offset: float = 0.0,
) -> tuple[float, float, float, float, float, float, float, float]:
    """
    Given a known robot state, return the four UWB distances plus tag positions.

    Returns: (d_LA, d_RA, d_LB, d_RB, lx, ly, rx, ry)
    """
    theta   = math.radians(heading_deg)
    # Forward direction (0=north CW): (sin θ, cos θ)
    fwd_x, fwd_y = math.sin(theta), math.cos(theta)
    # Right-lateral direction: (cos θ, −sin θ)
    rgt_x, rgt_y = math.cos(theta), -math.sin(theta)

    # Tag midpoint is forward_offset ahead of robot centre
    mid_x = robot_x + forward_offset * fwd_x
    mid_y = robot_y + forward_offset * fwd_y

    lx = mid_x - (tag_sep / 2.0) * rgt_x
    ly = mid_y - (tag_sep / 2.0) * rgt_y
    rx = mid_x + (tag_sep / 2.0) * rgt_x
    ry = mid_y + (tag_sep / 2.0) * rgt_y

    d_LA = math.sqrt((lx - ax) ** 2 + (ly - ay) ** 2)
    d_RA = math.sqrt((rx - ax) ** 2 + (ry - ay) ** 2)
    d_LB = math.sqrt((lx - bx) ** 2 + (ly - by) ** 2)
    d_RB = math.sqrt((rx - bx) ** 2 + (ry - by) ** 2)

    return d_LA, d_RA, d_LB, d_RB, lx, ly, rx, ry


# ---------------------------------------------------------------------------
# Dashboard visualiser — run:  python uwb_localizer.py
# SPACE steps through test cases, Q or close window to quit.
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import pygame
    from library.panther_dashboard import PantherDashboard

    # ── Anchor positions (metres, arena coordinates) ─────────────────────────
    # Anchor A on the left wall of the starting zone, Anchor B on the right wall.
    AX, AY = 0.0,  2.0   # Anchor A — top-left corner
    BX, BY = 2.0,  0.0   # Anchor B — top of starting zone, right edge

    TAG_SEP        = 0.6   # metres — measure actual left-to-right tag separation
    FORWARD_OFFSET = 0.0   # metres — tag midpoint offset from robot centre

    # ── Test cases: known robot state → synthetic distances ──────────────────
    # Each entry: (robot_x, robot_y, heading_deg, label)
    RAW_CASES = [
        # Starting zone area
        (1.0, 1.5,   0.0, "start zone — facing north"),
        (1.0, 1.5, 180.0, "start zone — facing south"),
        # Mid excavation zone
        (1.2, 3.5,  90.0, "excavation mid — facing east"),
        (1.2, 3.5, 270.0, "excavation mid — facing west"),
        # Crossing into obstacle zone (centre field)
        (3.5, 2.5,   0.0, "centre field — facing north"),
        (3.5, 2.5,  90.0, "centre field — facing east"),
        (3.5, 2.5, 135.0, "centre field — facing SE"),
        # Far right / obstacle zone
        (5.5, 3.5, 270.0, "far right obstacle — facing west"),
        (5.5, 1.5,  45.0, "far right low — facing NE"),
        # Near construction / berm zone
        (5.4, 0.6,   0.0, "near berm — facing north"),
        (5.4, 0.6, 270.0, "near berm — facing west"),
        # Top of field
        (3.0, 4.5, 180.0, "top field — facing south"),
    ]

    TEST_CASES = []
    for rx_, ry_, hdg_, lbl_ in RAW_CASES:
        d_LA_, d_RA_, d_LB_, d_RB_, *_ = _make_synthetic(
            rx_, ry_, hdg_, AX, AY, BX, BY, TAG_SEP, FORWARD_OFFSET
        )
        TEST_CASES.append((d_LA_, d_RA_, d_LB_, d_RB_, rx_, ry_, hdg_, lbl_))

    # ── Localizer + dashboard ─────────────────────────────────────────────────
    loc  = UWBLocalizer(AX, AY, BX, BY, TAG_SEP, FORWARD_OFFSET)
    dash = PantherDashboard("Panther Dashboard — UWB Debug", fullscreen=True)

    ADVANCE_MS   = 3000   # ms to display each test case before auto-advancing
    idx          = 0
    last_advance = pygame.time.get_ticks()

    def run_case(i):
        """Run the localizer on TEST_CASES[i] and return all computed and expected values."""
        d_LA, d_RA, d_LB, d_RB, ex, ey, eh, label = TEST_CASES[i]
        return loc.update(d_LA, d_RA, d_LB, d_RB), d_LA, d_RA, d_LB, d_RB, ex, ey, eh, label

    pos, d_LA, d_RA, d_LB, d_RB, ex, ey, eh, label = run_case(idx)

    # Colours
    _BLUE   = (100, 160, 255)
    _LBLUE  = ( 60, 110, 200)
    _RED    = (255,  90,  90)
    _LRED   = (200,  50,  50)
    _YELLOW = (255, 220,   0)
    _ORANGE = (255, 140,   0)
    _GREEN  = ( 80, 220,  80)

    while True:
        # ── Trilateration circles ──────────────────────────────────────────────
        if loc._last_d is not None:
            d_LA_, d_RA_, d_LB_, d_RB_ = loc._last_d
            # Left-tag circles (blue): one from each anchor
            dash.add_circle("cLA", AX, AY, d_LA_, color=_BLUE,  width=1)
            dash.add_circle("cLB", BX, BY, d_LB_, color=_LBLUE, width=1)
            # Right-tag circles (red): one from each anchor
            dash.add_circle("cRA", AX, AY, d_RA_, color=_RED,   width=1)
            dash.add_circle("cRB", BX, BY, d_RB_, color=_LRED,  width=1)

        # ── Candidate intersection points ──────────────────────────────────────
        if loc._left_candidates is not None:
            for i, (cpx, cpy) in enumerate(loc._left_candidates):
                dash.add_point(f"L{i}", cpx, cpy, color=_BLUE, radius=4)
        if loc._right_candidates is not None:
            for i, (cpx, cpy) in enumerate(loc._right_candidates):
                dash.add_point(f"R{i}", cpx, cpy, color=_RED, radius=4)

        # ── Selected tag positions (larger, labelled) ─────────────────────────
        dash.add_point("Anchor A", AX, AY, color=_YELLOW, radius=8)
        dash.add_point("Anchor B", BX, BY, color=_ORANGE, radius=8)

        if loc._left_tag:
            dash.add_point("Tag L", loc._left_tag[0],  loc._left_tag[1],  color=_BLUE, radius=6, label_offset=(9, 6))
        if loc._right_tag:
            dash.add_point("Tag R", loc._right_tag[0], loc._right_tag[1], color=_RED,  radius=6, label_offset=(9, -18))

        # Expected (ground truth) robot centre
        dash.add_point("Expected", ex, ey, color=_GREEN, radius=4, label_offset=(7, -18))

        # ── Robot pose ────────────────────────────────────────────────────────
        if pos:
            dash.set_robot(pos.x, pos.y, pos.heading)

        # ── Telemetry panel ───────────────────────────────────────────────────
        dash.put("case",       f"[{idx + 1}/{len(TEST_CASES)}] {label}")
        dash.put("d_LA",       f"{d_LA:.3f} m")
        dash.put("d_RA",       f"{d_RA:.3f} m")
        dash.put("d_LB",       f"{d_LB:.3f} m")
        dash.put("d_RB",       f"{d_RB:.3f} m")
        if pos:
            dash.put("x",          f"{pos.x:.3f} m  (exp {ex:.3f})")
            dash.put("y",          f"{pos.y:.3f} m  (exp {ey:.3f})")
            dash.put("heading",    f"{pos.heading:.1f} deg  (exp {eh:.1f})")
        else:
            dash.put("result", "NO SOLUTION")

        if not dash.update():
            break

        now = pygame.time.get_ticks()
        if dash.key_pressed(pygame.K_SPACE) or now - last_advance >= ADVANCE_MS:
            idx = (idx + 1) % len(TEST_CASES)
            pos, d_LA, d_RA, d_LB, d_RB, ex, ey, eh, label = run_case(idx)
            dash.clear_telemetry()
            last_advance = now
