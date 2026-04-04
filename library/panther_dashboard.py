"""
PantherDashboard — field viewer + telemetry panel for PantherLunabotics.

Arena: 6.88 m wide × 5.0 m tall.
Coordinate origin (0, 0) is the bottom-left corner.
X increases to the right (east), Y increases upward (north).
Heading: 0° = north, positive = clockwise (90° = east).

API
───
    dash = PantherDashboard()

    while True:
        dash.set_robot(x, y, heading_deg)          # robot position on field
        dash.set_target(tx, ty, heading_deg)       # target position on field
        dash.put("x", f"{x:.3f} m")        # telemetry key-value

        if not dash.update():               # renders + handles events
            break

        if dash.key_pressed(pygame.K_SPACE):
            ...
"""

import math
import os
import queue
import sys
import pygame

# ── Arena geometry (metres) ───────────────────────────────────────────────────
ARENA_W = 6.88
ARENA_H = 5.0

# ── Robot footprint — read from RobotConfig; falls back to defaults if unavailable ──
_onboard = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'onboard_software')
if _onboard not in sys.path:
    sys.path.insert(0, _onboard)
try:
    from robot_params import RobotConfig as _cfg
    ROBOT_LENGTH = _cfg.ROBOT_LENGTH
    ROBOT_WIDTH  = _cfg.ROBOT_WIDTH
except ImportError:
    ROBOT_LENGTH = 1.08   # metres front-to-back
    ROBOT_WIDTH  = 1.108  # metres left-to-right

# ── Default window size ───────────────────────────────────────────────────────
_DEFAULT_W   = 1400
_DEFAULT_H   = 750
_MIN_PANEL_W = 250
_PAD         = 12

# ── Colours ───────────────────────────────────────────────────────────────────
_C = {
    "bg":           (24,  24,  24),
    "arena_bg":     (200, 200, 195),
    "border":       (40,  40,  40),
    "panel_bg":     (32,  32,  32),
    "panel_border": (65,  65,  65),
    "panel_title":  (255, 200, 50),
    "key":          (140, 190, 255),
    "val":          (235, 235, 235),
    "robot":        (255, 255, 255),
    "arrow":        (255, 130, 30),
    "target":       (80,  220, 80),
}

# Path to the field image, relative to this file
_FIELD_IMAGE = "../field.png"


class PantherDashboard:
    """Field + telemetry viewer. See module docstring for API."""

    def __init__(self, title: str = "Panther Dashboard"):
        """Initialize the window, fonts, and layout."""
        pygame.init()
        self._screen     = pygame.display.set_mode((_DEFAULT_W, _DEFAULT_H))
        pygame.display.set_caption(title)
        self._clock      = pygame.time.Clock()
        self._font_sm    = pygame.font.SysFont("monospace", 15)
        self._font_title = pygame.font.SysFont("monospace", 18, bold=True)

        self._robot:        tuple[float, float, float] | None = None
        self._target:       tuple[float, float, float] | None = None
        self._robot_length: float = ROBOT_LENGTH
        self._robot_width:  float = ROBOT_WIDTH
        self._telemetry: dict[str, str] = {}
        self._running    = True
        self.keys_pressed: set[int] = set()

        img_path = os.path.join(os.path.dirname(__file__), _FIELD_IMAGE)
        self._field_img_src = (
            pygame.image.load(img_path).convert()
            if os.path.exists(img_path) else None
        )

        self._calc_layout(_DEFAULT_W, _DEFAULT_H)
        self._static = self._build_static()

    # ── layout ────────────────────────────────────────────────────────────────

    def _calc_layout(self, win_w: int, win_h: int) -> None:
        """Compute arena and panel pixel dimensions from the window size."""
        self._win_w = win_w
        self._win_h = win_h
        avail_w = win_w - _MIN_PANEL_W - _PAD * 3
        avail_h = win_h - _PAD * 4
        if avail_w * ARENA_H / ARENA_W <= avail_h:
            self._arena_px_w = avail_w
            self._arena_px_h = int(avail_w * ARENA_H / ARENA_W)
        else:
            self._arena_px_h = avail_h
            self._arena_px_w = int(avail_h * ARENA_W / ARENA_H)
        self._arena_x = _PAD
        self._arena_y = (win_h - self._arena_px_h) // 2
        self._panel_x = self._arena_x + self._arena_px_w + _PAD
        self._panel_w = win_w - self._panel_x - _PAD

    # ── coordinate helpers ────────────────────────────────────────────────────

    def _to_px(self, x_m: float, y_m: float) -> tuple[int, int]:
        """Convert arena coordinates (metres) to screen pixel coordinates."""
        px = int(x_m / ARENA_W * self._arena_px_w) + self._arena_x
        py = self._arena_px_h - int(y_m / ARENA_H * self._arena_px_h) + self._arena_y
        return px, py

    # ── static surface ────────────────────────────────────────────────────────

    def _build_static(self) -> pygame.Surface:
        """Build the background surface: field image (or fallback), border, and panel."""
        surf = pygame.Surface((self._win_w, self._win_h))
        surf.fill(_C["bg"])

        if self._field_img_src is not None:
            scaled = pygame.transform.smoothscale(
                self._field_img_src, (self._arena_px_w, self._arena_px_h))
            surf.blit(scaled, (self._arena_x, self._arena_y))
        else:
            arena = pygame.Rect(self._arena_x, self._arena_y,
                                self._arena_px_w, self._arena_px_h)
            pygame.draw.rect(surf, _C["arena_bg"], arena)
            msg = self._font_sm.render(
                f"Drop field image here: {_FIELD_IMAGE}", True, (180, 80, 80))
            surf.blit(msg, msg.get_rect(
                center=(self._arena_x + self._arena_px_w // 2,
                        self._arena_y + self._arena_px_h // 2)))

        pygame.draw.rect(surf, _C["border"],
                         pygame.Rect(self._arena_x, self._arena_y,
                                     self._arena_px_w, self._arena_px_h), 2)

        panel = pygame.Rect(self._panel_x, _PAD, self._panel_w, self._win_h - _PAD * 2)
        pygame.draw.rect(surf, _C["panel_bg"], panel)
        pygame.draw.rect(surf, _C["panel_border"], panel, 1)

        t = self._font_title.render("Telemetry", True, _C["panel_title"])
        surf.blit(t, (self._panel_x + 10, _PAD + 10))
        pygame.draw.line(surf, _C["panel_border"],
                         (self._panel_x + 8,                _PAD + 34),
                         (self._panel_x + self._panel_w - 8, _PAD + 34), 1)
        return surf

    # ── public API ────────────────────────────────────────────────────────────

    def set_robot(self, x: float, y: float, heading_deg: float = 0.0) -> None:
        """Update the robot's current position and heading."""
        self._robot = (x, y, heading_deg)

    def set_target(self, x: float, y: float, heading_deg: float = 0.0) -> None:
        """Set the target position and heading the robot is navigating toward."""
        self._target = (x, y, heading_deg)

    def put(self, key: str, value) -> None:
        """Add or update a telemetry key-value entry."""
        self._telemetry[key] = str(value)

    def update(self) -> bool:
        """Render one frame and process events. Returns False when the window is closed."""
        self._render()
        self._clock.tick(30)
        self.keys_pressed = set()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._running = False
            if event.type == pygame.KEYDOWN:
                self.keys_pressed.add(event.key)
                if event.key == pygame.K_q:
                    self._running = False
        if not self._running:
            pygame.quit()
        return self._running

    def key_pressed(self, key: int) -> bool:
        """Check if a pygame key constant was pressed this frame."""
        return key in self.keys_pressed

    # ── internal rendering ────────────────────────────────────────────────────

    def _render(self) -> None:
        """Draw one frame: static background, target, robot, then telemetry panel."""
        self._screen.blit(self._static, (0, 0))
        if self._target is not None:
            self._draw_target(*self._target)
        if self._robot is not None:
            self._draw_robot(*self._robot)
        self._draw_panel()
        pygame.display.flip()

    def _draw_target(self, x: float, y: float, heading_deg: float) -> None:
        """Draw the target as a scaled-down robot outline with a heading arrow."""
        _TARGET_SCALE = 0.35
        cx, cy = self._to_px(x, y)
        rad = math.radians(90 - heading_deg)  # convert: 0=north CW → standard math angle
        color = _C["arrow"]

        half_l = int(self._robot_length * _TARGET_SCALE / ARENA_W * self._arena_px_w / 2)
        half_w = int(self._robot_width  * _TARGET_SCALE / ARENA_H * self._arena_px_h / 2)
        corners_local = [
            ( half_l,  half_w),
            ( half_l, -half_w),
            (-half_l, -half_w),
            (-half_l,  half_w),
        ]
        cos_r, sin_r = math.cos(rad), math.sin(rad)
        corners = [
            (int(cx + lx * cos_r - ly * sin_r),
             int(cy - lx * sin_r - ly * cos_r))
            for lx, ly in corners_local
        ]
        pygame.draw.polygon(self._screen, color, corners, 2)

        tip = (int(cx + half_l * cos_r), int(cy - half_l * sin_r))
        pygame.draw.line(self._screen, color, (cx, cy), tip, 2)
        for side in (0.4, -0.4):
            ah = (int(tip[0] - 6 * math.cos(rad - side)),
                  int(tip[1] + 6 * math.sin(rad - side)))
            pygame.draw.line(self._screen, color, tip, ah, 2)


    def _draw_robot(self, x: float, y: float, heading_deg: float) -> None:
        """Draw the robot rectangle, heading arrow, and line to target."""
        cx, cy = self._to_px(x, y)
        rad = math.radians(90 - heading_deg)  # convert: 0=north CW → standard math angle

        half_l = int(self._robot_length / ARENA_W * self._arena_px_w / 2)
        half_w = int(self._robot_width  / ARENA_H * self._arena_px_h / 2)
        corners_local = [
            ( half_l,  half_w),
            ( half_l, -half_w),
            (-half_l, -half_w),
            (-half_l,  half_w),
        ]
        cos_r, sin_r = math.cos(rad), math.sin(rad)
        corners = [
            (int(cx + lx * cos_r - ly * sin_r),
             int(cy - lx * sin_r - ly * cos_r))
            for lx, ly in corners_local
        ]
        pygame.draw.polygon(self._screen, _C["robot"], corners)
        pygame.draw.polygon(self._screen, _C["arrow"], corners, 2)

        tip = (int(cx + half_l * cos_r), int(cy - half_l * sin_r))
        pygame.draw.line(self._screen, _C["arrow"], (cx, cy), tip, 2)
        for side in (0.4, -0.4):
            ah = (int(tip[0] - 8 * math.cos(rad - side)),
                  int(tip[1] + 8 * math.sin(rad - side)))
            pygame.draw.line(self._screen, _C["arrow"], tip, ah, 2)

        # Draw line from robot to target if both are set
        if self._target is not None:
            tx, ty = self._to_px(self._target[0], self._target[1])
            pygame.draw.line(self._screen, _C["target"], (cx, cy), (tx, ty), 1)

    def _draw_panel(self) -> None:
        """Render telemetry key-value entries in the right panel."""
        if not self._telemetry:
            return
        x         = self._panel_x + 10
        panel_right = self._panel_x + self._panel_w - 8
        row_h     = 20
        cur_x     = x
        cur_y     = _PAD + 42

        for key, val in self._telemetry.items():
            label = self._font_sm.render(f"{key}: ", True, _C["key"])
            value = self._font_sm.render(val,        True, _C["val"])

            fits_inline = cur_x + label.get_width() + value.get_width() <= panel_right

            self._screen.blit(label, (cur_x, cur_y))

            if fits_inline:
                self._screen.blit(value, (cur_x + label.get_width(), cur_y))
                cur_x += label.get_width() + value.get_width() + 16
            else:
                cur_y += row_h
                self._screen.blit(value, (x, cur_y))
                cur_y += row_h + 4
                cur_x  = x


# ── DashboardView — runs as a separate process on the laptop ─────────────────

class DashboardView:
    """
    Launched by control.py as a daemon process via multiprocessing.
    Reads telemetry dicts from a multiprocessing.Queue and renders them
    in PantherDashboard, keeping pygame isolated from the joystick process.

    Expected dict structure from the robot (via Dashboard.send_telemetry):
        "pose":      {"x": float, "y": float, "heading": float}
        "target":    {"x": float, "y": float, "heading": float}
        "telemetry": {"Key": "value", ...}  — shown in the panel

    Usage (in control.py):
        q = multiprocessing.Queue()
        p = multiprocessing.Process(target=DashboardView.run, args=(q,), daemon=True)
        p.start()
        ...
        q.put(telemetry_dict)
    """

    @staticmethod
    def run(q) -> None:
        """Entry point for the dashboard process. Reads from queue and renders each frame."""
        dash = PantherDashboard("Panther Dashboard")
        while dash.update():
            # Drain all pending updates — only care about the latest state
            data = None
            try:
                while True:
                    data = q.get_nowait()
            except queue.Empty:
                pass

            if data is None:
                continue

            if "pose" in data:
                pose = data["pose"]
                dash.set_robot(pose["x"], pose["y"], pose.get("heading", 0.0))

            if "target" in data:
                target = data["target"]
                dash.set_target(target["x"], target["y"], target.get("heading", 0.0))

            if "telemetry" in data:
                for key, val in data["telemetry"].items():
                    dash.put(key, val)


# ── standalone smoke test ─────────────────────────────────────────────────────
if __name__ == "__main__":
    dash = PantherDashboard("Panther Dashboard — smoke test")

    # Berm target position (construction zone centre)
    dash.set_target(5.38, 0.6, 90.0)

    cx, cy = ARENA_W / 2, ARENA_H / 2
    t = 0.0
    while True:
        t += 0.04
        x   = cx + 1.5 * math.cos(t)
        y   = cy + 1.5 * math.sin(t)
        # Tangent to circle in 0=north CW system: heading = -t (robot starts at east, moves north)
        hdg = (-math.degrees(t)) % 360
        dash.set_robot(x, y, hdg)
        dash.put("Position", f"X: {x:.3f} m, Y: {y:.3f} m, Heading: {hdg:.1f} deg")
        if not dash.update():
            break
