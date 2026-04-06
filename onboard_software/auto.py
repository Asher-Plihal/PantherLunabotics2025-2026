from __future__ import annotations
import time
import robot_params
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import robot

class Auto:
    """Manages autonomous mode: runs the 50 Hz periodic loop and dispatches button events."""

    def __init__(self, robot: robot.Robot):
        """Attach to the robot and initialize the loop timer."""
        self.robot = robot
        self._last_update_time = time.monotonic()

    # Called only when there is a button event
    def on_button_event(self, _button, _is_pressed):
        """Placeholder for future autonomous button handling; currently a no-op."""
        pass  # Autonomous button handling not yet implemented

    def periodic_loop(self):
        """Called at 50Hz — put all periodic tasks here."""

        # Update motor controller (must come before telemetry reads)
        self.robot.motor_controller.update()

        # Print telemetry and log data
        self.robot.auger.log_data()
        self.robot.drivetrain.log_data()
        self.robot.perception.lidar_stream.log_data()

        if robot_params.RobotConfig.usePIDDrive and self.robot.pid_drive is not None:
            self.robot.pid_drive.update()

    def run_auto_step(self):
        """Call periodic_loop() when the 50 Hz period has elapsed."""
        # Update periodic loop
        now = time.monotonic()
        elapsed = now - self._last_update_time
        if elapsed >= robot_params.LoopConfig.UPDATE_PERIOD_S:
            self.periodic_loop()
            self._last_update_time = now
