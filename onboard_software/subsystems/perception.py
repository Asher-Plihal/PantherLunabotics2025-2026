from __future__ import annotations
import datetime
import json
import math
import os
import threading
import time
from typing import TYPE_CHECKING
import numpy as np
from library.streaming import Stream, StreamMode
from robot_params import RobotConfig

if TYPE_CHECKING:
    from rplidar import RPLidar  # type: ignore


# ---------------------------------------------------------------------------
# Perception — subsystem that manages all sensor streams (lidar, camera)
# ---------------------------------------------------------------------------

class Perception:
    """Manages sensor streams based on RobotConfig settings."""

    def __init__(self):
        self.lidar_stream = Lidar(
            RobotConfig.lidarStream, RobotConfig.lidarStreamPort
        )
        self.camera_stream = CameraStream(
            RobotConfig.cameraStream, RobotConfig.cameraStreamPort
        )

    def start(self):
        if RobotConfig.useLidar:
            self.lidar_stream.start()
            if RobotConfig.logLiDarTelemetry:
                self.lidar_stream.start_logging()
        self.camera_stream.start()

    def stop(self):
        self.lidar_stream.stop()
        self.camera_stream.stop()


# ---------------------------------------------------------------------------
# Lidar
# ---------------------------------------------------------------------------

class Lidar:
    """Manages lidar hardware, streaming, and logging."""

    _PORT = "/dev/ttyUSB0"

    # ── Display ────────────────────────────────────────────────────────────

    class Display:
        """Radar display constants and rendering helpers."""
        WIDTH, HEIGHT = 600, 600
        CENTER = (WIDTH // 2, HEIGHT // 2)
        MIN_DIST, MAX_DIST = 50, 3000
        SCALE = (WIDTH // 2) / MAX_DIST

        BLACK      = (0, 0, 0)
        GREEN      = (0, 255, 0)
        YELLOW     = (255, 255, 0)
        RED        = (255, 0, 0)
        DARK_GREEN = (0, 100, 0)
        WHITE      = (255, 255, 255)

        @staticmethod
        def polar_to_cartesian(angle_deg, distance_mm):
            angle_rad = math.radians((angle_deg + 180) % 360)
            r = distance_mm * D.SCALE
            x = D.CENTER[0] + int(r * math.cos(angle_rad))
            y = D.CENTER[1] + int(r * math.sin(angle_rad))
            return x, y

        @staticmethod
        def init(pygame):
            """Create pygame window and pre-rendered overlay."""
            screen = pygame.display.set_mode((D.WIDTH, D.HEIGHT))
            pygame.display.set_caption("RPLidar Radar Map")
            clock = pygame.time.Clock()

            overlay = pygame.Surface((D.WIDTH, D.HEIGHT), pygame.SRCALPHA)
            font_small = pygame.font.SysFont(None, 20)
            font_title = pygame.font.SysFont(None, 28, bold=True)
            for r in range(500, D.MAX_DIST + 1, 500):
                pygame.draw.circle(overlay, D.DARK_GREEN, D.CENTER, int(r * D.SCALE), 1)
                label = font_small.render(f"{r // 10} cm", True, D.WHITE)
                overlay.blit(label, (D.CENTER[0] + int(r * D.SCALE) - 25, D.CENTER[1]))
            title = font_title.render("RPLidar Radar Map", True, D.WHITE)
            overlay.blit(title, (D.WIDTH // 2 - title.get_width() // 2, D.HEIGHT - 40))

            return screen, clock, overlay

        @staticmethod
        def render(pygame, screen, overlay, points):
            """Render one frame of lidar points onto the screen."""
            screen.fill(D.BLACK)
            screen.blit(overlay, (0, 0))
            for angle, distance in points:
                px, py = D.polar_to_cartesian(angle, distance)
                color = D.RED if distance <= 1000 else D.YELLOW if distance <= 2000 else D.GREEN
                pygame.draw.circle(screen, color, (px, py), 2)
            pygame.display.flip()

    # ── Viewer (mission control side) ─────────────────────────────────────

    @staticmethod
    def run_viewer():
        """Lidar viewer process — connects to robot and displays radar feed."""
        import pygame

        stream = Stream(RobotConfig.lidarStreamPort, "LidarViewer")
        stream.connect(RobotConfig.Robot_IP)

        pygame.init()
        screen, clock, overlay = D.init(pygame)

        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return
                points = json.loads(stream.recv_frame())
                D.render(pygame, screen, overlay, points)
                clock.tick(60)
        except (ConnectionError, KeyboardInterrupt):
            print("\nDisconnected.")
        finally:
            stream.stop()
            pygame.quit()

    # ── Instance (robot side) ─────────────────────────────────────────────

    def __init__(self, mode: StreamMode, port: int):
        self.mode = mode
        self.stream = Stream(port, "LidarStream")
        self.lidar: RPLidar | None = None
        self._log_data: list = []
        self._logging = False
        self._log_dir = os.path.join(os.path.dirname(__file__), '..', 'logs')
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self):
        self._running = True
        if self.mode == StreamMode.REMOTE:
            self.stream._running = True  # allow accept_viewer() loop and stop()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print(f"[Lidar] Hardware started (stream={self.mode.value})")

    def stop(self):
        if not self._running and self._thread is None:
            return
        self._running = False
        self.stream.stop()
        if self._thread:
            self._thread.join(timeout=3)
            self._thread = None
        self._cleanup_hardware()
        self.stop_logging()

    def start_logging(self):
        if self._logging:
            print("[Lidar] Logging already active")
            return
        self._log_data = []
        self._logging = True
        print("[Lidar] Logging started")

    def stop_logging(self):
        if not self._logging:
            return
        self._logging = False
        if not self._log_data:
            return
        os.makedirs(self._log_dir, exist_ok=True)
        file_tag = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filepath = os.path.join(self._log_dir, f"lidar_{file_tag}.txt")
        data = np.array(self._log_data)
        np.savetxt(filepath, data, header="angle_deg distance_mm", fmt="%.3f")
        print(f"[Lidar] Logging stopped — {len(data)} points saved to {filepath}")
        self._log_data = []

    def log_data(self):
        pass  # Lidar logging is driven by the scan thread; no periodic action needed

    def _run(self):
        try:
            self._init_hardware()
            if self.mode == StreamMode.REMOTE:
                self.stream.accept_viewer()
                self._scan_loop(lambda pts: self.stream.send_frame(json.dumps(pts).encode()))
            elif self.mode == StreamMode.LOCAL:
                import pygame
                if "DISPLAY" not in os.environ:
                    os.environ["DISPLAY"] = ":0"
                pygame.init()
                screen, clock, overlay = D.init(pygame)
                try:
                    self._scan_loop(lambda pts: (D.render(pygame, screen, overlay, pts), clock.tick(60)))
                finally:
                    pygame.quit()
            else:
                self._scan_loop()
        except Exception as e:
            print(f"[Lidar] Error: {e}")
        finally:
            self._cleanup_hardware()

    def _scan_loop(self, on_points=None):
        """Iterate lidar scans. on_points(pts) is called each frame; return False to stop."""
        assert self.lidar is not None
        for scan in self.lidar.iter_scans():
            if not self._running:
                break
            points = self._filter_scan(scan)
            self._log_scan(points)
            if on_points is not None and on_points(points) is False:
                break

    def _init_hardware(self):
        from rplidar import RPLidar  # type: ignore
        self.lidar = RPLidar(Lidar._PORT)
        time.sleep(1)

    def _cleanup_hardware(self):
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except Exception:
                pass
            self.lidar = None

    def _filter_scan(self, scan):
        return [
            (round(angle, 3), round(distance, 3))
            for _, angle, distance in scan
            if D.MIN_DIST <= distance <= D.MAX_DIST
        ]

    def _log_scan(self, points):
        if self._logging:
            self._log_data.extend(points)

# Module-level alias — defined once after Lidar so all methods can use D
D = Lidar.Display


# ---------------------------------------------------------------------------
# CameraStream — stub for future camera integration
# ---------------------------------------------------------------------------

class CameraStream:

    def __init__(self, mode: StreamMode, port: int):
        self.mode = mode
        self.stream = Stream(port, "CameraStream")

    def start(self):
        self.stream.start_source(self.mode, self._run)

    def stop(self):
        self.stream.stop()

    def _run(self):
        pass  # TODO: implement camera streaming
