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
        self._button_drive_active = False

    # Called only when there is a button event
    def on_button_event(self, button, is_pressed):
        if button in ('A', 'Y', 'B', 'X', 'LB', 'RB'):
            if robot_params.RobotConfig.useDrivetrain:
                self._button_drive_active = is_pressed
                if is_pressed:
                    if button == 'A':
                        self.robot.drivetrain.set_power(0.5,0.5,0.5,0.5)
                    elif button == 'Y':
                        self.robot.drivetrain.set_power(-0.5,-0.5,-0.5,-0.5)
                    elif button == 'B':
                        self.robot.drivetrain.set_power(-0.5,-0.5,0.5,0.5)
                    elif button == 'X':
                        self.robot.drivetrain.set_power(0.5,0.5,-0.5,-0.5)
                    elif button == 'LB':
                        self.robot.drivetrain.set_power(-0.5,0.5,-0.5,0.5)
                    elif button == 'RB':
                        self.robot.drivetrain.set_power(0.5,-0.5,0.5,-0.5)
                else:
                    self.robot.drivetrain.set_power(0.0,0.0,0.0,0.0)
        elif button == 'DPAD_UP':
            if is_pressed:
                #Intake Auger
                print("Intake Auger")
                self.robot.auger.intake()
        elif button == 'DPAD_DOWN':
            if is_pressed:
                #Outtake Auger
                print("Outtake Auger")
                self.robot.auger.outtake()
        elif button == 'DPAD_LEFT':
            if is_pressed:
                #Off Drivetrain
                if robot_params.RobotConfig.useDrivetrain:
                    self.robot.drivetrain.set_power(0.0,0.0,0.0,0.0)
        elif button == 'DPAD_RIGHT':
            if is_pressed:
                #Off Auger
                self.robot.auger.stop()

    """Called at 50Hz — put all periodic tasks here."""
    def periodic_loop(self):
        if robot_params.RobotConfig.useDrivetrain and not self._button_drive_active:
            self.robot.drivetrain.drive_task(self.robot.controller.AxisValues.y, self.robot.controller.AxisValues.x, self.robot.controller.AxisValues.yaw_rate)

        # Check for joystick drift
        robot_params.Telemetry.print_t(f"Controller Y: {self.robot.controller.AxisValues.y:.2f}, X: {self.robot.controller.AxisValues.x:.2f}, Yaw: {self.robot.controller.AxisValues.yaw_rate:.2f}")

        # Print telemetry and log data
        self.robot.auger.log_data()

        # Update motor controller
        self.robot.motor_controller.update()

    def run_teleOp_step(self):

        # Update periodic loop
        now = time.monotonic()
        elapsed = now - self._last_update_time
        if elapsed >= robot_params.LoopConfig.UPDATE_PERIOD_S:
            self.periodic_loop()
            self._last_update_time = now
