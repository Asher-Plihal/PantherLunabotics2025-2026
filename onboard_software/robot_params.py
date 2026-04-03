import time
import sys
import os
from enum import Enum

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from library.streaming import StreamMode


class DriveMode(Enum):
    ARCADE = "arcade"
    TANK = "tank"

# Global timer instance to be initialized by robot.py on startup
robot_timer: "RobotTimer | None" = None

class RobotConfig:
    # Robot parameters
    ROBOT_LENGTH = 1.5 # Meters from front to back
    ROBOT_WIDTH  = 0.75 # Meters from left to right

    # Network configuration
    Robot_IP = "100.87.109.7"
    Robot_Port = 8080
    Robot_CAN_Interface = "can0"

    # Streaming configuration
    lidarStream = StreamMode.REMOTE
    lidarStreamPort = 5000
    cameraStream = StreamMode.NONE
    cameraStreamPort = 5001
    fieldDashboard = True

    # Subsystem configuration
    useDrivetrain = True
    useAuger = True
    useLidar = False

    drivetrainMode = DriveMode.ARCADE

    # Telemetry configuration
    useTelemetry = False
    logLiDarTelemetry = False
    logAugerTelemetry = False
    logDrivetrainTelemetry = False


class NetworkConfig:
    # Select which network to connect to on startup.
    # The key must match one of the entries in NETWORKS below.
    SELECTED_NETWORK = "FLTech-Guest"

    # Maps a friendly name to the nmcli connection profile name saved on the Jetson.
    # Add or rename entries to match what you see in `nmcli connection show`.
    NETWORKS = {
        "FLTech-Guest": "FLTech-Guest",
        "Ashers-HS": "Me phone ",
    }


class LoopConfig:
    UPDATE_RATE_HZ = 50  # Change this to adjust loop frequency
    UPDATE_PERIOD_S = 1.0 / UPDATE_RATE_HZ  # 0.02s at 50Hz

class RobotTimer:
    def __init__(self):
        self._start_time = None

    def start(self):
        self._start_time = time.monotonic()

    def elapsed(self):
        if self._start_time is None:
            return 0.0
        return time.monotonic() - self._start_time

    def timestamp(self):
        e = self.elapsed()
        minutes = int(e) // 60
        seconds = e % 60
        return f"[T+{minutes:02d}:{seconds:05.2f}]"


def print_config():
    cfg = RobotConfig
    print(f"[Config] Subsystems:  Drivetrain: {cfg.useDrivetrain}  Auger: {cfg.useAuger}  Lidar: {cfg.useLidar}")
    print(f"[Config] Telemetry:   Enabled: {cfg.useTelemetry}  Drivetrain: {cfg.logDrivetrainTelemetry}  Auger: {cfg.logAugerTelemetry}  Lidar: {cfg.logLiDarTelemetry}")


class Telemetry:
    PRINTS_PER_SECOND = 5  # Change this to adjust how often telemetry prints per second
    _timers = {}  # Per-call-site timers keyed by (filename, lineno)

    @classmethod
    def print_t(cls, *args, prints_per_second=PRINTS_PER_SECOND, **kwargs):
        period = 1.0 / prints_per_second
        frame = sys._getframe(1)
        key = (frame.f_code.co_filename, frame.f_lineno)
        now = time.monotonic()
        if now - cls._timers.get(key, 0.0) >= period:
            prefix = robot_timer.timestamp() + " " if robot_timer is not None else ""
            print(prefix + " ".join(str(a) for a in args), **kwargs)
            cls._timers[key] = now
