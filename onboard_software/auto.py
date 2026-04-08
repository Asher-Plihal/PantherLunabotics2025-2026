from __future__ import annotations
import time
import os
import sys
import robot_params
from typing import TYPE_CHECKING

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from library.protocol import Button

if TYPE_CHECKING:
    import robot

class Auto:
    """Manages autonomous mode: runs the 50 Hz periodic loop and dispatches button events."""

    def __init__(self, robot: robot.Robot):
        """Attach to the robot and initialize the loop timer."""
        self.robot = robot
        self._last_update_time = time.monotonic()

    def on_button_event(self, button, is_pressed: bool) -> None:
        """Toggle auto tasks on button press: LB = excavation, RB = dump."""
        if not is_pressed:
            return

        if button == Button.LB:
            """Toggle excavation: first press starts, second press stops."""
            task = self.robot.excavation_task
            if task.is_running:
                task.stop_auto_task()
            else:
                task.start_auto_task()

        elif button == Button.RB:
            """Toggle dump: first press starts, second press stops."""
            task = self.robot.dump_task
            if task.is_running:
                task.stop_auto_task()
            else:
                task.start_auto_task()

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

        # Run auto task state machines
        self.robot.excavation_task.run_task_states()
        self.robot.dump_task.run_task_states()

    def run_auto_step(self):
        """Call periodic_loop() when the 50 Hz period has elapsed."""
        now = time.monotonic()
        elapsed = now - self._last_update_time
        if elapsed >= robot_params.LoopConfig.UPDATE_PERIOD_S:
            self.periodic_loop()
            self._last_update_time = now
