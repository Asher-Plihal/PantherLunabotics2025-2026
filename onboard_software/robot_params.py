# RobotParams
# Stores all constants and configs used across the robot codebase.

import time
import sys

# Global timer instance to be initialized by robot.py on startup
robot_timer = None

class RobotConfig:
    useRiLidarRemoteStream = True
    useCameraRemoteStream = True

    useDrivetrain = True
    useAuger = True

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
            print(*args, **kwargs)
            cls._timers[key] = now