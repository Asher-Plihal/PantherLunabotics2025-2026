import time
import sys
import os
from enum import Enum

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from library.pid_drive import Position
from library.streaming import StreamMode
from library.pid_controller import PIDController


class DriveMode(Enum):
    """Drivetrain input interpretation mode."""
    ARCADE = "arcade"
    TANK = "tank"

# Global timer instance to be initialized by robot.py on startup
robot_timer: "RobotTimer | None" = None

class RobotConfig:
    """Central configuration for all robot hardware, networking, and software parameters."""

    # Robot parameters
    ROBOT_LENGTH = 1.08 # Meters from front to back
    ROBOT_WIDTH  = 1.108 # Meters from left to right

    # Network configuration
    Robot_IP = "192.168.0.101" #Tailscale 100.87.109.7
    Robot_Port = 8080
    Robot_CAN_Interface = "can0"

    # Streaming configuration
    lidarStream = StreamMode.REMOTE
    lidarStreamPort = 5000
    cameraStream = StreamMode.NONE
    cameraStreamPort = 5001
    fieldDashboard = False
    uwbDashboard   = False  # send UWB trilateration overlay (circles + points) to the field dashboard

    # Subsystem configuration
    useDrivetrain = True
    useAuger = True
    useLidar = False

    drivetrainMode = DriveMode.ARCADE
    drivetrainMaxSpeed = 0.2
    usePIDDrive = False

    # UWB Localizer — anchor positions (metres, arena coordinates) and tag geometry
    uwbAnchorAX, uwbAnchorAY = 0.0, 2.0
    uwbAnchorBX, uwbAnchorBY = 2.0, 0.0
    uwbTagSep:        float = 0.6   # left-to-right physical separation (metres)
    uwbForwardOffset: float = 0.0   # tag midpoint offset ahead of robot centre (metres)
    uwbLeftPort:      str   = "/dev/ttyUSB1"
    uwbRightPort:     str   = "/dev/ttyUSB2"

    # PID Drive coefficients — tune on hardware
    pidXCoeffs = PIDController.PIDCoefficients(kp=0, ki=0.0, kd=0.0)  # lateral (east/west)
    pidYCoeffs = PIDController.PIDCoefficients(kp=0, ki=0.0, kd=0.0)  # longitudinal (north/south)
    pidHCoeffs = PIDController.PIDCoefficients(kp=0, ki=0.0, kd=0.0)  # heading

    # Telemetry configuration
    useTelemetry = False
    logLiDarTelemetry = False
    logAugerTelemetry = False
    logDrivetrainTelemetry = False

class Positions:
    """ Default position used by pid_drive set_target() """

    dumpPos = Position(x=0.0, y=0.0, heading=0.0)
    excavatePos = Position(x=0.0, y=0.0, heading=0.0)


class NetworkConfig:
    """Network connection profile selection for the Jetson."""

    # Select which network to connect to on startup.
    # The key must match one of the entries in NETWORKS below.
    SELECTED_NETWORK = "Router_5"

    # Maps a friendly name to the nmcli connection profile name saved on the Jetson.
    # Add or rename entries to match what you see in `nmcli connection show`.
    NETWORKS = {
        "FLTech-Guest": "FLTech-Guest",
        "Ashers-HS": "Me phone ",
        "Router_5": "Team_9_5G",
        "Router_2.4" "Team_9"
    }


class LoopConfig:
    """Control loop timing constants."""

    UPDATE_RATE_HZ = 50  # Change this to adjust loop frequency
    UPDATE_PERIOD_S = 1.0 / UPDATE_RATE_HZ  # 0.02s at 50Hz

class RobotTimer:
    """Monotonic timer that starts when the robot receives READY from mission control."""

    def __init__(self):
        """Initialize the timer in the unstarted state."""
        self._start_time = None

    def start(self):
        """Record the current time as the competition start time."""
        self._start_time = time.monotonic()

    def elapsed(self):
        """Seconds elapsed since start(), or 0.0 if not yet started."""
        if self._start_time is None:
            return 0.0
        return time.monotonic() - self._start_time

    def timestamp(self):
        """Formatted elapsed time string (e.g. '[T+00:01.23]') for log prefixes."""
        e = self.elapsed()
        minutes = int(e) // 60
        seconds = e % 60
        return f"[T+{minutes:02d}:{seconds:05.2f}]"


def print_config():
    """Print the active subsystem and telemetry configuration to stdout."""
    cfg = RobotConfig
    print(f"[Config] Subsystems:  Drivetrain: {cfg.useDrivetrain}  Auger: {cfg.useAuger}  Lidar: {cfg.useLidar}")
    print(f"[Config] Telemetry:   Enabled: {cfg.useTelemetry}  Drivetrain: {cfg.logDrivetrainTelemetry}  Auger: {cfg.logAugerTelemetry}  Lidar: {cfg.logLiDarTelemetry}")


class Telemetry:
    """Rate-limited print utility that prepends a competition timestamp to each message."""

    PRINTS_PER_SECOND = 5  # Change this to adjust how often telemetry prints per second
    _timers = {}  # Per-call-site timers keyed by (filename, lineno)

    @classmethod
    def print_t(cls, *args, prints_per_second=PRINTS_PER_SECOND, **kwargs):
        """Print args at most prints_per_second times per call site; prepends the competition timestamp."""
        period = 1.0 / prints_per_second
        frame = sys._getframe(1)
        key = (frame.f_code.co_filename, frame.f_lineno)
        now = time.monotonic()
        if now - cls._timers.get(key, 0.0) >= period:
            prefix = robot_timer.timestamp() + " " if robot_timer is not None else ""
            print(prefix + " ".join(str(a) for a in args), **kwargs)
            cls._timers[key] = now
