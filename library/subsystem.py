import time
import sys
import os
from abc import ABC, abstractmethod

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
try:
    import robot_params
except ImportError:
    robot_params = None


class Subsystem(ABC):
    """
    Base class for all robot subsystems.

    Provides two shared concerns:

    1. Subsystem ownership — ensures only one caller (TeleOp, an AutoTask,
       etc.) can drive a subsystem at a time.

       Ownership rules:
         - Ownership must be claimed explicitly via claim_ownership(owner).
         - Any motor method call is silently ignored if a different owner holds
           the subsystem. Calls with owner=None pass through only when the
           subsystem is unclaimed.
         - Only the current owner can release via release_ownership().
         - force_release() clears ownership unconditionally — use it during
           mode switches (TELEOP ↔ AUTO) so the incoming mode starts clean.

    2. Telemetry helpers — shared logging utilities used by all subsystems.
    """

    def __init__(self):
        """Initialize ownership state and telemetry interval timer."""
        self._owner = None
        self._last_telemetry_time = 0.0

    # ------------------------------------------------------------------
    # Ownership
    # ------------------------------------------------------------------

    def _print_ownership_telemetry(self, event: str, owner) -> None:
        """Print ownership event to terminal if telemetry is available."""
        if robot_params is not None:
            owner_name = owner.__class__.__name__ if owner is not None else "None"
            subsystem_name = self.__class__.__name__
            robot_params.Telemetry.print_t(
                f"[Ownership] {subsystem_name}: {event} by {owner_name}",
                prints_per_second=10
            )

    def _print_motor_call_denied(self, owner) -> None:
        """Print telemetry when a motor command is rejected due to ownership."""
        if robot_params is not None:
            owner_name = owner.__class__.__name__ if owner is not None else "None"
            current_owner = self._owner.__class__.__name__ if self._owner is not None else "None"
            subsystem_name = self.__class__.__name__
            robot_params.Telemetry.print_t(
                f"[Ownership] {subsystem_name}: MOTOR_CALL_DENIED from {owner_name} (currently owned by {current_owner})",
                prints_per_second=5
            )

    def is_owned(self) -> bool:
        """Returns True if any caller currently owns this subsystem."""
        return self._owner is not None

    def get_owner(self):
        """Returns the object that currently owns this subsystem, or None."""
        return self._owner

    def claim_ownership(self, owner) -> bool:
        """
        Explicitly claim ownership of this subsystem.

        Returns True if the caller now owns the subsystem (either it was free
        or the caller already owned it). Returns False if a different owner
        already holds the subsystem.
        """
        if not self.is_owned():
            self._owner = owner
            self._print_ownership_telemetry("CLAIMED", owner)
            return True
        already_owner = owner is self.get_owner()
        if not already_owner:
            self._print_ownership_telemetry("DENIED", owner)
        return already_owner

    def release_ownership(self, owner) -> None:
        """Release ownership. Ignored if the caller is not the current owner."""
        if owner is self._owner:
            self._print_ownership_telemetry("RELEASED", owner)
            self._owner = None

    def force_release(self) -> None:
        """Unconditionally clear ownership. Use during mode switches."""
        if self._owner is not None:
            self._print_ownership_telemetry("FORCE_RELEASED", self._owner)
        self._owner = None

    def check_ownership(self, owner) -> bool:
        """
        Called by subclass motor methods before writing to hardware.

        Returns True if the call should proceed, False if it should be blocked.
        """
        if not self.is_owned():
            return True
        return owner is self.get_owner()

    # ------------------------------------------------------------------
    # Telemetry helpers
    # ------------------------------------------------------------------

    def _check_telemetry_interval(self, interval: float) -> bool:
        """
        Returns True (and updates the timestamp) if enough time has passed
        since the last telemetry print. Returns False if the interval has not
        elapsed yet.
        """
        now = time.monotonic()
        if now - self._last_telemetry_time < interval:
            return False
        self._last_telemetry_time = now
        return True

    # ------------------------------------------------------------------
    # Required subsystem interface
    # ------------------------------------------------------------------

    @abstractmethod
    def shutdown(self) -> None:
        """Stop all actuators and release resources."""

    @abstractmethod
    def start_logging(self) -> None:
        """Begin writing telemetry data to CSV."""

    @abstractmethod
    def stop_logging(self) -> None:
        """Stop writing telemetry data to CSV."""

    @abstractmethod
    def log_data(self) -> None:
        """Write one row of telemetry data. Called each cycle while logging."""

    @abstractmethod
    def print_telemetry(self) -> None:
        """Print a human-readable telemetry snapshot to stdout."""

    # ------------------------------------------------------------------
    # Telemetry helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _format_motor_feedback(feedback, duty_cycle=True, velocity=True,
                               position=True, current=True,
                               temperature=False, voltage=True) -> list[str]:
        """Build the telemetry parts list from a single MotorFeedback object."""
        parts = []
        if duty_cycle:
            parts.append(f"Duty Cycle: {feedback.duty_cycle:.4f}")
        if velocity:
            parts.append(f"Velocity: {feedback.velocity:.2f} RPM")
        if position:
            parts.append(f"Position: {feedback.position:.1f} ticks")
        if current:
            parts.append(f"Current: {feedback.current:.2f} A")
        if temperature:
            parts.append(f"Temp: {feedback.temperature:.1f} °C")
        if voltage:
            parts.append(f"Bus: {feedback.voltage:.2f} V")
        return parts
