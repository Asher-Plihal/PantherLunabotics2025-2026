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
        self.lidar_stream = LidarStream(
            RobotConfig.lidarStream, RobotConfig.lidarStreamPort
        )
        self.camera_stream = CameraStream(
            RobotConfig.cameraStream, RobotConfig.cameraStreamPort
        )

    def start(self):
        self.lidar_stream.start()
        self.camera_stream.start()

    def stop(self):
        self.lidar_stream.stop()
        self.camera_stream.stop()


# ---------------------------------------------------------------------------
# LidarStream
# ---------------------------------------------------------------------------

# Display constants
_WIDTH, _HEIGHT = 600, 600
_CENTER = (_WIDTH // 2, _HEIGHT // 2)
_MIN_DISTANCE = 50       # mm
_MAX_DISTANCE = 3000     # mm
_SCALE = (_WIDTH // 2) / _MAX_DISTANCE

# Colors
_BLACK = (0, 0, 0)
_GREEN = (0, 255, 0)
_YELLOW = (255, 255, 0)
_RED = (255, 0, 0)
_DARK_GREEN = (0, 100, 0)
_WHITE = (255, 255, 255)

_LIDAR_PORT = "/dev/ttyUSB0"


def _polar_to_cartesian(angle_deg, distance_mm):
    angle_rad = math.radians((angle_deg + 180) % 360)
    r = distance_mm * _SCALE
    x = _CENTER[0] + int(r * math.cos(angle_rad))
    y = _CENTER[1] + int(r * math.sin(angle_rad))
    return x, y


def _init_radar_display(pygame):
    """Create pygame window and pre-rendered overlay for the radar view."""
    screen = pygame.display.set_mode((_WIDTH, _HEIGHT))
    pygame.display.set_caption("RPLidar Radar Map")
    clock = pygame.time.Clock()

    overlay = pygame.Surface((_WIDTH, _HEIGHT), pygame.SRCALPHA)
    font_small = pygame.font.SysFont(None, 20)
    font_title = pygame.font.SysFont(None, 28, bold=True)
    for r in range(500, _MAX_DISTANCE + 1, 500):
        pygame.draw.circle(overlay, _DARK_GREEN, _CENTER, int(r * _SCALE), 1)
        label = font_small.render(f"{r // 10} cm", True, _WHITE)
        overlay.blit(label, (_CENTER[0] + int(r * _SCALE) - 25, _CENTER[1]))
    title = font_title.render("RPLidar Radar Map", True, _WHITE)
    overlay.blit(title, (_WIDTH // 2 - title.get_width() // 2, _HEIGHT - 40))

    return screen, clock, overlay


def _render_frame(pygame, screen, overlay, points):
    """Render one frame of lidar points. Points are (angle, distance) tuples."""
    screen.fill(_BLACK)
    screen.blit(overlay, (0, 0))
    for angle, distance in points:
        px, py = _polar_to_cartesian(angle, distance)
        if distance <= 1000:
            color = _RED
        elif distance <= 2000:
            color = _YELLOW
        else:
            color = _GREEN
        pygame.draw.circle(screen, color, (px, py), 2)
    pygame.display.flip()


def run_lidar_viewer():
    """Lidar viewer process — connects to robot and displays radar feed."""
    import pygame

    stream = Stream(RobotConfig.lidarStreamPort, "LidarViewer")
    stream.connect(RobotConfig.Robot_IP)

    pygame.init()
    screen, clock, overlay = _init_radar_display(pygame)

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
            points = json.loads(stream.recv_frame())
            _render_frame(pygame, screen, overlay, points)
            clock.tick(60)
    except (ConnectionError, KeyboardInterrupt):
        print("\nDisconnected.")
    finally:
        stream.stop()
        pygame.quit()


class LidarStream:

    def __init__(self, mode: StreamMode, port: int):
        self.mode = mode
        self.stream = Stream(port, "LidarStream")
        self.lidar: RPLidar | None = None
        self._log_data: list = []
        self._logging = False
        self._log_thread: threading.Thread | None = None
        self._log_dir = os.path.join(os.path.dirname(__file__), '..', 'logs')

    def start(self):
        self.stream.start_source(self.mode, self._run)

    def stop(self):
        self.stream.stop()
        self._cleanup_hardware()
        self.stop_logging()

    def start_logging(self):
        self._log_data = []
        self._logging = True
        print("[LidarStream] Logging started")
        if self.mode == StreamMode.NONE:
            self._log_thread = threading.Thread(target=self._run_logging_only, daemon=True)
            self._log_thread.start()

    def stop_logging(self):
        if not self._logging:
            return
        self._logging = False
        if self._log_thread:
            self._log_thread.join(timeout=5)
            self._log_thread = None
        if not self._log_data:
            return
        os.makedirs(self._log_dir, exist_ok=True)
        file_tag = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filepath = os.path.join(self._log_dir, f"lidar_{file_tag}.txt")
        data = np.concatenate(self._log_data, axis=1)
        np.savetxt(filepath, data)
        print(f"[LidarStream] Logging stopped — {data.shape[1]} points saved to {filepath}")
        self._log_data = []

    def _run_logging_only(self):
        try:
            self._init_hardware()
            assert self.lidar is not None
            for scan in self.lidar.iter_scans():
                if not self._logging:
                    break
                self._log_scan(scan)
        except Exception as e:
            print(f"[LidarStream] Logging error: {e}")
        finally:
            self._cleanup_hardware()

    def log_data(self):
        pass  # Lidar logging is driven by the scan thread; no periodic action needed

    def _log_scan(self, scan):
        if self._logging and scan:
            self._log_data.append(np.array(scan).T)

    def _run(self):
        try:
            self._init_hardware()
            if self.mode == StreamMode.REMOTE:
                self.stream.accept_viewer()
                self._run_remote()
            elif self.mode == StreamMode.LOCAL:
                self._run_local()
        except Exception as e:
            print(f"[LidarStream] Error: {e}")
        finally:
            self._cleanup_hardware()

    def _init_hardware(self):
        from rplidar import RPLidar  # type: ignore
        self.lidar = RPLidar(_LIDAR_PORT)
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
            (round(angle, 1), round(distance, 1))
            for _, angle, distance in scan
            if _MIN_DISTANCE <= distance <= _MAX_DISTANCE
        ]

    def _run_remote(self):
        assert self.lidar is not None
        for scan in self.lidar.iter_scans():
            if not self.stream.running:
                break
            self._log_scan(scan)
            points = self._filter_scan(scan)
            data = json.dumps(points).encode()
            if not self.stream.send_frame(data):
                break

    def _run_local(self):
        import pygame

        pygame.init()
        screen, clock, overlay = _init_radar_display(pygame)

        assert self.lidar is not None
        try:
            for scan in self.lidar.iter_scans():
                if not self.stream.running:
                    break
                self._log_scan(scan)
                points = self._filter_scan(scan)
                _render_frame(pygame, screen, overlay, points)
                clock.tick(60)
        finally:
            pygame.quit()


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
