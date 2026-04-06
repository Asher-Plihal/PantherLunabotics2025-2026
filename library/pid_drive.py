from dataclasses import dataclass
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from typing import cast
from library.pid_controller import PIDController


@dataclass
class Position:
    """Arena-frame pose snapshot: x/y position in metres and heading in degrees."""

    x:       float  # metres, arena coordinate
    y:       float  # metres, arena coordinate
    heading: float  # degrees, from IMU


class PIDDrive:
    """
    Field-centric PID drive-to-point for a holonomic (mecanum/screw) drivetrain.

    Coordinate system:
      - Origin at arena bottom-left
      - +X = east (right), +Y = north (up)
      - Heading 0° = north, positive = clockwise (90° = east)

    Localizer must be a callable or an object with get_pose() returning a Position.
    """

    # Default tolerances — override with set_xy_tolerance / set_heading_tolerance
    DEFAULT_XY_TOLERANCE_M  = 0.05   # meters
    DEFAULT_H_TOLERANCE_DEG = 3.0    # degrees

    def __init__(self, drivetrain, localizer,
                 x_coeffs: PIDController.PIDCoefficients,
                 y_coeffs: PIDController.PIDCoefficients,
                 h_coeffs: PIDController.PIDCoefficients,
                 squid: bool = True):
        """
        Args:
            drivetrain: Drivetrain subsystem instance.
            localizer:  Callable or object with get_pose() -> Position.
            x_coeffs:   PID coefficients for lateral (east/west) error.
            y_coeffs:   PID coefficients for longitudinal (north/south) error.
            h_coeffs:   PID coefficients for heading error.
            squid:      If True, applies sqrt power scaling to all three PIDs.
        """
        self._drivetrain = drivetrain
        self._localizer  = localizer

        self._x_pid = PIDController(x_coeffs)
        self._y_pid = PIDController(y_coeffs)
        self._h_pid = PIDController(h_coeffs)

        # Heading wraps at ±180° (shortest path)
        self._h_pid.enableWrapTarget(-180.0, 180.0)

        if squid:
            self._x_pid.enable_squid()
            self._y_pid.enable_squid()
            self._h_pid.enable_squid()

        self._owner = None

        self._target:           Position | None = None
        self._current_position: Position | None = None
        self._on_target: bool = False

        self._xy_tolerance: float = self.DEFAULT_XY_TOLERANCE_M
        self._h_tolerance:  float = self.DEFAULT_H_TOLERANCE_DEG

        self.localizer_start()

    # ------------------------------------------------------------------
    # Target
    # ------------------------------------------------------------------

    def set_target(self, target: Position) -> None:
        """Set the target pose and reset all PIDs."""
        self._target    = target
        self._on_target = False
        self._x_pid.reset()
        self._y_pid.reset()
        self._h_pid.reset()

    @property
    def target(self) -> Position | None:
        """The most recently set target pose, or None."""
        return self._target

    @property
    def current_position(self) -> Position | None:
        """The position read during the last update() call, or None."""
        return self._current_position

    # ------------------------------------------------------------------
    # Main update — call at 50 Hz inside periodic_loop()
    # ------------------------------------------------------------------

    def claim_ownership(self, owner) -> None:
        """Set the ownership token forwarded to drivetrain.set_power() on each update()."""
        self._owner = owner

    def release_ownership(self) -> None:
        """Clear the ownership token so update() passes None to drivetrain.set_power()."""
        self._owner = None

    def update(self) -> None:
        """
        Fetch current position, compute and apply motor powers toward the target.
        Updates the internal on-target state — check with on_target().
        No-op if no target has been set.
        """
        if self._target is None:
            return

        self._current_position = self._get_pose()
        heading_rad = math.radians(self._current_position.heading)

        # PID outputs in field frame
        field_fwd = self._y_pid.calculate(self._target.y,       self._current_position.y)
        field_str = self._x_pid.calculate(self._target.x,       self._current_position.x)
        turn      = self._h_pid.calculate(self._target.heading,  self._current_position.heading)

        # Rotate field-frame translation into robot frame
        # (undo the robot's clockwise heading rotation)
        robot_fwd = field_fwd * math.cos(heading_rad) + field_str * math.sin(heading_rad)
        robot_str = -field_fwd * math.sin(heading_rad) + field_str * math.cos(heading_rad)

        # Mecanum mixing — same sign convention as _drive_task_arcade in drivetrain.py
        fl, fr, bl, br = self._drivetrain.calculate_arcade_powers(-robot_fwd, robot_str, turn)

        self._drivetrain.set_power(self._owner, fl, fr, bl, br)
        self._on_target = (self._x_pid.isOnTargetDefault(self._xy_tolerance) and
                           self._y_pid.isOnTargetDefault(self._xy_tolerance) and
                           self._h_pid.isOnTargetDefault(self._h_tolerance))

    def on_target(self) -> bool:
        """Returns the on-target state from the last update() call."""
        return self._on_target

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def __del__(self) -> None:
        """Shut down the controller on garbage collection."""
        self.shutdown()

    def localizer_start(self) -> None:
        """Start the localizer if it supports it (e.g. background polling thread)."""
        if hasattr(self._localizer, "start"):
            self._localizer.start()

    def shutdown(self) -> None:
        """Stop the localizer, clear target, reset all PIDs, and stop motors."""
        if hasattr(self._localizer, "stop"):
            self._localizer.stop()
        self._target    = None
        self._on_target = False
        self._x_pid.reset()
        self._y_pid.reset()
        self._h_pid.reset()
        self._drivetrain.stop()

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def stop(self) -> None:
        """Zero all drive motors."""
        self._drivetrain.stop()

    def reset(self) -> None:
        """Clear target, reset all PIDs, and stop motors."""
        self._target    = None
        self._on_target = False
        self._x_pid.reset()
        self._y_pid.reset()
        self._h_pid.reset()
        self.stop()

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def set_xy_tolerance(self, tolerance_m: float) -> None:
        """Override the XY on-target tolerance (metres). Default: DEFAULT_XY_TOLERANCE_M."""
        self._xy_tolerance = tolerance_m

    def set_heading_tolerance(self, tolerance_deg: float) -> None:
        """Override the heading on-target tolerance (degrees). Default: DEFAULT_H_TOLERANCE_DEG."""
        self._h_tolerance = tolerance_deg

    def set_max_translation_speed(self, speed: float) -> None:
        """Clamp X and Y PID output (0.0–1.0)."""
        self._x_pid.setOutputLimit(speed)
        self._y_pid.setOutputLimit(speed)

    def set_max_rotation_speed(self, speed: float) -> None:
        """Clamp heading PID output (0.0–1.0)."""
        self._h_pid.setOutputLimit(speed)

    def set_squid(self, enabled: bool) -> None:
        """Enable or disable sqrt power scaling on all three PIDs."""
        if enabled:
            self._x_pid.enable_squid()
            self._y_pid.enable_squid()
            self._h_pid.enable_squid()
        else:
            self._x_pid.disable_squid()
            self._y_pid.disable_squid()
            self._h_pid.disable_squid()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _get_pose(self) -> Position:
        """Retrieve the current pose from the localizer, supporting both callable and object forms."""
        if callable(self._localizer):
            return cast(Position, self._localizer())
        return cast(Position, self._localizer.get_pose())
