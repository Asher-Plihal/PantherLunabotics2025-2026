from __future__ import annotations
import time
import robot_params
from typing import TYPE_CHECKING

# self.robot.drivetrain.set_power(0.5,0.5,-0.5,-0.5) fold out
if TYPE_CHECKING:
    import robot

class TeleOp:

    def __init__(self, robot: robot.Robot):
        self.robot = robot
        self._last_update_time = time.monotonic()
        self._button_drive_active = False

    # Called only when there is a button event
    # Drivetrain: DPAD_UP = forward, DPAD_DOWN = backward, DPAD_LEFT = strafe left, DPAD_RIGHT = strafe right
    # Drivetrain: LB = turn left, RB = turn right
    # Auger: Y = intake, A = outtake, B = stop auger
    def on_button_event(self, button, is_pressed):
        if button in ('DPAD_UP', 'DPAD_DOWN', 'DPAD_LEFT', 'DPAD_RIGHT', 'LB', 'RB'):
            if robot_params.RobotConfig.useDrivetrain:
                self._button_drive_active = is_pressed
                if is_pressed:
                    if button == 'DPAD_UP':
                        # Drive forward
                        self.robot.drivetrain.set_power(-0.5,-0.5,-0.5,-0.5)
                    elif button == 'DPAD_DOWN':
                        # Drive backward
                        self.robot.drivetrain.set_power(0.5,0.5,0.5,0.5)
                    elif button == 'DPAD_LEFT':
                        # Strafe left
                        self.robot.drivetrain.set_power(-0.5,0.5,0.5,-0.5)
                    elif button == 'DPAD_RIGHT':
                        # Strafe right
                        self.robot.drivetrain.set_power(-0.5,0.5,0.5,-0.5)
                    elif button == 'LB':
                        # Turn left
                        self.robot.drivetrain.set_power(-0.5,0.5,-0.5,0.5)
                    elif button == 'RB':
                        # Turn right
                        self.robot.drivetrain.set_power(0.5,-0.5,0.5,-0.5)
                else:
                    self.robot.drivetrain.set_power(0.0,0.0,0.0,0.0)
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
