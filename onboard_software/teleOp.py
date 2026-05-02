from __future__ import annotations
import time
import robot_params
from typing import TYPE_CHECKING
from library.protocol import Button

if TYPE_CHECKING:
    import robot

_TRIGGER_THRESHOLD = -0.6  # axis value above which a trigger counts as "pressed" (~20% press)

class TeleOp:
    """Handles teleoperated robot control: maps gamepad buttons and sticks to subsystem commands."""

    def __init__(self, robot: robot.Robot):
        """Attach to the robot and initialize the loop timer and active drive button state."""
        self.robot = robot
        self._last_update_time = time.monotonic()
        self._active_drive_buttons: set = set()
        self._lt_was_pressed: bool = False
        self._rt_was_pressed: bool = False

    def on_button_event(self, button, is_pressed):
        """Dispatch a single gamepad button press or release to the appropriate subsystem."""
        if button == Button.DPAD_DOWN:
            if robot_params.RobotConfig.useDrivetrain:
                if is_pressed:
                    self._active_drive_buttons.add(button)
                    self.robot.drivetrain.fold_out()
                else:
                    self._active_drive_buttons.discard(button)
                    if not self._active_drive_buttons:
                        self.robot.drivetrain.stop()
        elif button == Button.Y:
            if is_pressed:
                self.robot.auger.set_auger_dump_angle()
        elif button == Button.B:
            if is_pressed:
                self.robot.auger.set_auger_transport_angle()
        elif button == Button.A:
            if is_pressed:
                self.robot.auger.set_auger_intake_angle()
        elif button == Button.X:
            if is_pressed:
                task = self.robot.excavation_task
                if task.is_running:
                    task.stop_auto_task()
                else:
                    task.start_auto_task()
        elif button == Button.LB:
            if is_pressed:
                if self.robot.auger.power > 0:
                    self.robot.auger.stop()
                else:
                    self.robot.auger.intake()
        elif button == Button.RB:
            if is_pressed:
                if self.robot.auger.power < 0:
                    self.robot.auger.stop()
                else:
                    self.robot.auger.outtake()

    def periodic_loop(self):
        """Called at 50Hz — put all periodic tasks here."""
        axes = self.robot.controller.axis_values

        if robot_params.RobotConfig.useDrivetrain and not self._active_drive_buttons:
            self.robot.drivetrain.drive_task(
                None,
                axes.left_stick_y, axes.left_stick_x,
                axes.right_stick_y, axes.right_stick_x
            )

        if robot_params.RobotConfig.useAuger:
            lt_pressed = axes.lt > _TRIGGER_THRESHOLD
            rt_pressed = axes.rt > _TRIGGER_THRESHOLD

            # LT held → retract actuator fully; release → freeze at current position
            if lt_pressed and not self._lt_was_pressed:
                self.robot.auger.set_auger_min_angle()
                self._lt_was_pressed = True
            elif not lt_pressed and self._lt_was_pressed:
                self.robot.auger.set_auger_angle(self.robot.auger.get_auger_angle())
                self._lt_was_pressed = False

            # RT held → extend actuator fully; release → freeze at current position
            if rt_pressed and not self._rt_was_pressed:
                self.robot.auger.set_auger_max_angle()
                self._rt_was_pressed = True
            elif not rt_pressed and self._rt_was_pressed:
                self.robot.auger.set_auger_angle(self.robot.auger.get_auger_angle())
                self._rt_was_pressed = False

        # Run auto task state machines
        self.robot.excavation_task.run_task_states()

        # Update motor controller (must come before telemetry reads)
        self.robot.motor_controller.update()
        self.robot.auger.update_actuator()

        #Test if current will work for auger trigger
        #self.robot.auger.is_full

        # Print telemetry and log data
        self.robot.auger.log_data()
        self.robot.drivetrain.log_data()

        if robot_params.RobotConfig.usePIDDrive and self.robot.pid_drive is not None:
            self.robot.pid_drive.update()

    def run_teleOp_step(self):
        """Call periodic_loop() when the 50 Hz period has elapsed."""
        # Update periodic loop
        now = time.monotonic()
        elapsed = now - self._last_update_time
        if elapsed >= robot_params.LoopConfig.UPDATE_PERIOD_S:
            self.periodic_loop()
            self._last_update_time = now
