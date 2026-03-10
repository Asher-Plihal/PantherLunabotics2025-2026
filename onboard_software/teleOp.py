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

    # Called only when there is a button event
    def on_button_event(self, button, is_pressed):
        if button == 'A':
            if is_pressed:
                #Forward
                if robot_params.RobotConfig.useDrivetrain:
                    self.robot.drivetrain.set_power(0.5,0.5,0.5,0.5)
        elif button == 'Y':
            if is_pressed:
                #Backward
                if robot_params.RobotConfig.useDrivetrain:
                    self.robot.drivetrain.set_power(-0.5,-0.5,-0.5,-0.5)
        elif button == 'B':
            if is_pressed:
                #Strafe right
                if robot_params.RobotConfig.useDrivetrain:
                    self.robot.drivetrain.set_power(-0.5,-0.5,0.5,0.5)
        elif button == 'X':
            if is_pressed:
                #Strafe left
                if robot_params.RobotConfig.useDrivetrain:
                    self.robot.drivetrain.set_power(0.5,0.5,-0.5,-0.5)
        elif button == 'LB':
            if is_pressed:
                #Turn left
                if robot_params.RobotConfig.useDrivetrain:
                    self.robot.drivetrain.set_power(-0.5,0.5,-0.5,0.5)
        elif button == 'RB':
            if is_pressed:
                #Turn right
                if robot_params.RobotConfig.useDrivetrain:
                    self.robot.drivetrain.set_power(0.5,-0.5,0.5,-0.5)
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
        if robot_params.RobotConfig.useDrivetrain:
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
