from __future__ import annotations
import time
import robot_params
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import robot

class TeleOp:

    def __init__(self, robot: robot.Robot):
        self.robot = robot
        self._last_update_time = time.monotonic()
        self._active_drive_buttons: set = set()

    def on_button_event(self, button, is_pressed):
        if button in ('DPAD_UP', 'DPAD_DOWN', 'DPAD_LEFT', 'DPAD_RIGHT', 'LB', 'RB', 'X'):
            if robot_params.RobotConfig.useDrivetrain:
                if is_pressed:
                    self._active_drive_buttons.add(button)
                    if button == 'DPAD_UP':
                        self.robot.drivetrain.drive_forward()
                    elif button == 'DPAD_DOWN':
                        self.robot.drivetrain.drive_backward()
                    elif button == 'DPAD_LEFT':
                        self.robot.drivetrain.strafe_left()
                    elif button == 'DPAD_RIGHT':
                        self.robot.drivetrain.strafe_right()
                    elif button == 'LB':
                        self.robot.drivetrain.turn_left()
                    elif button == 'RB':
                        self.robot.drivetrain.turn_right()
                    elif button == 'X':
                        self.robot.drivetrain.fold_out()
                else:
                    self._active_drive_buttons.discard(button)
                    if not self._active_drive_buttons:
                        self.robot.drivetrain.stop()
        elif button == 'Y':
            if is_pressed:
                # Auger intake
                print("Intake Auger")
                self.robot.auger.intake()
        elif button == 'A':
            if is_pressed:
                # Auger outtake
                print("Outtake Auger")
                self.robot.auger.outtake()
        elif button == 'B':
            if is_pressed:
                # Stop auger
                print("Stop Auger")
                self.robot.auger.stop()

    def periodic_loop(self):
        """Called at 50Hz — put all periodic tasks here."""
        if robot_params.RobotConfig.useDrivetrain and not self._active_drive_buttons:
            axes = self.robot.controller.AxisValues
            self.robot.drivetrain.drive_task(
                axes.left_stick_y, axes.left_stick_x,
                axes.right_stick_y, axes.right_stick_x
            )

        # Print telemetry and log data
        self.robot.auger.log_data()
        self.robot.drivetrain.log_data()
        self.robot.perception.lidar_stream.log_data()

        # Update motor controller
        self.robot.motor_controller.update()

    def run_teleOp_step(self):

        # Update periodic loop
        now = time.monotonic()
        elapsed = now - self._last_update_time
        if elapsed >= robot_params.LoopConfig.UPDATE_PERIOD_S:
            self.periodic_loop()
            self._last_update_time = now
