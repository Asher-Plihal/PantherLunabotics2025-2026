from __future__ import annotations
import time
import robot_params
from typing import TYPE_CHECKING
from library.protocol import Button

if TYPE_CHECKING:
    import robot

class TeleOp:

    def __init__(self, robot: robot.Robot):
        self.robot = robot
        self._last_update_time = time.monotonic()
        self._active_drive_buttons: set = set()

    def on_button_event(self, button, is_pressed):
        if button in (Button.DPAD_UP, Button.DPAD_DOWN, Button.DPAD_LEFT, Button.DPAD_RIGHT,
                      Button.LB, Button.RB, Button.X):
            if robot_params.RobotConfig.useDrivetrain:
                if is_pressed:
                    self._active_drive_buttons.add(button)
                    if button == Button.DPAD_UP:
                        self.robot.drivetrain.drive_forward()
                    elif button == Button.DPAD_DOWN:
                        self.robot.drivetrain.drive_backward()
                    elif button == Button.DPAD_LEFT:
                        self.robot.drivetrain.strafe_left()
                    elif button == Button.DPAD_RIGHT:
                        self.robot.drivetrain.strafe_right()
                    elif button == Button.LB:
                        self.robot.drivetrain.turn_left()
                    elif button == Button.RB:
                        self.robot.drivetrain.turn_right()
                    elif button == Button.X:
                        self.robot.drivetrain.fold_out()
                else:
                    self._active_drive_buttons.discard(button)
                    if not self._active_drive_buttons:
                        self.robot.drivetrain.stop()
        elif button == Button.Y:
            if is_pressed:
                self.robot.auger.intake()
        elif button == Button.A:
            if is_pressed:
                self.robot.auger.outtake()
        elif button == Button.B:
            if is_pressed:
                self.robot.auger.stop()

    def periodic_loop(self):
        """Called at 50Hz — put all periodic tasks here."""
        if robot_params.RobotConfig.useDrivetrain and not self._active_drive_buttons:
            axes = self.robot.controller.axis_values
            self.robot.drivetrain.drive_task(
                axes.left_stick_y, axes.left_stick_x,
                axes.right_stick_y, axes.right_stick_x
            )

        # Update motor controller (must come before telemetry reads)
        self.robot.motor_controller.update()

        # Print telemetry and log data
        self.robot.auger.log_data()
        self.robot.drivetrain.log_data()
        self.robot.perception.lidar_stream.log_data()

    def run_teleOp_step(self):

        # Update periodic loop
        now = time.monotonic()
        elapsed = now - self._last_update_time
        if elapsed >= robot_params.LoopConfig.UPDATE_PERIOD_S:
            self.periodic_loop()
            self._last_update_time = now
