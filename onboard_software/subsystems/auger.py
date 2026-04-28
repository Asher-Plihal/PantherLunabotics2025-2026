import os
import sys
import time
import serial
import robot_params
from collections import deque
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from library import telemetry_logger
from library.subsystem import Subsystem

sys.path.append(os.path.join(os.path.dirname(__file__), '../../library/motor_controller/build'))
import motor_controller  # type: ignore

# Note: if this is changed, update log_data as well
_LOG_COLUMNS = ["Duty Cycle", "Velocity (RPM)", "Position (ticks)", "Current (A)", "Temp (°C)", "Bus Voltage (V)"]

_ACTUATOR_PORT = "/dev/ttyTHS1"
_ACTUATOR_BAUD = 9600

# Preset positions in inches
_TRANSPORT_INCHES = 1.0
_INTAKE_INCHES    = 2.0
_DUMP_INCHES      = 3.0

# Full-detection tuning parameters
_FULL_CURRENT_THRESHOLD_A = 80.0  # amps — sustained current above this signals a full auger
_FULL_DURATION_S = 0.5            # seconds — current must stay above threshold this long
_CURRENT_SMA_WINDOW = 150           # samples — moving average window to filter transient current drops

class Auger(Subsystem):
    """Sample collection motor subsystem (CAN ID 3)."""

    def __init__(self, mc):
        """Configure the auger motor, reset its position encoder, and optionally start logging."""
        super().__init__()
        self.mc = mc
        self.motor_id = 4
        self._logger = telemetry_logger.TelemetryLogger("auger")

        config = motor_controller.MotorConfig()
        config.idle_mode = motor_controller.IdleMode.BRAKE
        config.motor_type = motor_controller.MotorType.BRUSHLESS
        config.sensor_type = motor_controller.SensorType.HALL_SENSOR
        config.ramp_rate = 0.0
        config.inverted = True
        config.motor_kv = 480
        config.smart_current_free_limit = 20.0
        config.smart_current_stall_limit = 80.0

        self.mc.initialize_motor(self.motor_id, config)
        self.mc.reset_motor_position(self.motor_id)

        self._full_current_start: float | None = None
        self._current_window: deque[float] = deque(maxlen=_CURRENT_SMA_WINDOW)

        self._actuator_pos: float = 0.0
        self._actuator_moving: bool = False
        self._last_actuator_target: float | None = None
        self._pending_actuator_target: float | None = None
        try:
            self._actuator = serial.Serial(_ACTUATOR_PORT, _ACTUATOR_BAUD, timeout=0)
            time.sleep(2)  # allow Arduino to boot
        except serial.SerialException as e:
            print(f"[Auger] Linear actuator serial unavailable: {e}")
            self._actuator = None

        if robot_params.RobotConfig.logAugerTelemetry:
            self.start_logging()

    def set_power(self, owner, power):
        """Write duty cycle to the auger motor; blocked if owner check fails."""
        if not self.check_ownership(owner):
            self._print_motor_call_denied(owner)
            return
        self.mc.set_motor_duty_cycle(self.motor_id, power)

    def intake(self, owner=None):
        """Run the auger in the intake direction at 50% power."""
        self.set_power(owner, 0.5)

    def outtake(self, owner=None):
        """Run the auger in the outtake direction at 50% power."""
        self.set_power(owner, -0.5)

    def stop(self, owner=None):
        """Stop the auger motor."""
        self.set_power(owner, 0.0)

    def set_auger_angle(self, pos_inches: float) -> None:
        """Stage a target position for the linear actuator.

        Does not write to serial immediately — the command is flushed once per cycle
        by update_actuator(), so rapid button spam cannot overflow the serial buffer.
        """
        if self._actuator is None:
            return
        self._pending_actuator_target = pos_inches

    def get_auger_angle(self) -> float:
        """Return the last position reported by the linear actuator in inches."""
        return self._actuator_pos

    def update_actuator(self) -> None:
        """Flush any pending MOVE command and drain incoming serial; must be called each cycle.

        Sending is rate-limited to once per cycle, so no matter how fast the user
        triggers preset buttons the Arduino's serial buffer can never be overrun.
        """
        if self._actuator is None:
            return

        # Send at most one MOVE per cycle, only if the target actually changed.
        if self._pending_actuator_target is not None and self._pending_actuator_target != self._last_actuator_target:
            self._actuator.write(f"MOVE {self._pending_actuator_target:.2f}\n".encode())
            self._last_actuator_target = self._pending_actuator_target
            self._actuator_moving = True
        self._pending_actuator_target = None

        while self._actuator.in_waiting:
            line = self._actuator.readline().decode(errors="ignore").strip()
            if line.startswith("POS"):
                try:
                    self._actuator_pos = float(line.split()[1])
                except (IndexError, ValueError):
                    pass
            elif line == "DONE":
                self._actuator_moving = False
                self._last_actuator_target = None

    def set_auger_transport_angle(self) -> None:
        """Move the auger to its stowed/transport position."""
        self.set_auger_angle(_TRANSPORT_INCHES)

    def set_auger_intake_angle(self) -> None:
        """Move the auger to its digging/intake position."""
        self.set_auger_angle(_INTAKE_INCHES)

    def set_auger_dump_angle(self) -> None:
        """Move the auger to its dump position."""
        self.set_auger_angle(_DUMP_INCHES)

    @property
    def is_full(self) -> bool:
        """
        Returns True when the smoothed motor current (SMA over _CURRENT_SMA_WINDOW
        samples) has been continuously above _FULL_CURRENT_THRESHOLD_A for at least
        _FULL_DURATION_S seconds. Automatically resets when the smoothed current
        drops back below the threshold.
        """
        feedback = self.mc.get_motor_feedback(self.motor_id)
        self._current_window.append(feedback.current)
        smoothed_current = sum(self._current_window) / len(self._current_window)
        now = time.monotonic()
        if smoothed_current >= _FULL_CURRENT_THRESHOLD_A:
            if self._full_current_start is None:
                self._full_current_start = now
        else:
            self._full_current_start = None
        delta = now - self._full_current_start if self._full_current_start is not None else 0.0
        full = self._full_current_start is not None and delta >= _FULL_DURATION_S
        start_str = f"{self._full_current_start:.2f}s" if self._full_current_start is not None else "None"
        robot_params.Telemetry.print_t(
            f"[Auger] current={feedback.current:.2f}A  smoothed={smoothed_current:.2f}A  start={start_str}  delta={delta:.2f}s  is_full={full}",
            prints_per_second=10
        )
        return full

    def start_logging(self):
        """Open a CSV log file for auger telemetry."""
        self._logger.start_logging(_LOG_COLUMNS)

    def stop_logging(self):
        """Close the CSV log file."""
        self._logger.stop_logging()

    def shutdown(self):
        """Release ownership, stop the motor, close the log file, and close the serial port."""
        self.force_release()
        self.stop()
        self.stop_logging()
        if self._actuator is not None:
            self._actuator.close()

    def print_telemetry(self, duty_cycle=True, velocity=True, position=True, current=True, temperature=False, voltage=True, interval=0.1):
        """Print formatted motor feedback, rate-limited by interval seconds."""
        if not self._check_telemetry_interval(interval):
            return
        feedback = self.mc.get_motor_feedback(self.motor_id)
        parts = self._format_motor_feedback(feedback, duty_cycle, velocity, position, current, temperature, voltage)
        if not parts or robot_params.robot_timer is None:
            return
        print(f"{robot_params.robot_timer.timestamp()} [Auger] " + ", ".join(parts))

    def log_data(self):
        """Print telemetry if enabled, poll the actuator serial port, and append a CSV row if logging is active."""
        if robot_params.RobotConfig.useTelemetry:
            self.print_telemetry()
        if self._logger is None or not self._logger.is_logging:
            return
        feedback = self.mc.get_motor_feedback(self.motor_id)
        self._logger.log_row(
            [feedback.duty_cycle, feedback.velocity, feedback.position,
             feedback.current, feedback.temperature, feedback.voltage]
        )
