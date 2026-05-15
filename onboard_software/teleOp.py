from __future__ import annotations
import time
import robot_params
from typing import TYPE_CHECKING
from library.protocol import Button

if TYPE_CHECKING:
    import robot

_TRIGGER_PRESS_THRESHOLD   = -0.5  # axis must exceed this to start a trigger press (~25% press)
_TRIGGER_RELEASE_THRESHOLD = -0.8  # axis must drop below this before a press is considered released (~10%)
# Hysteresis prevents chatter when the axis lingers at the transition point.

_FREEZE_TOLERANCE_IN  = 0.15  # freeze is complete once the actuator is within this many inches of the target
_FREEZE_MIN_DURATION_S = 2.0   # freeze must stay active at least this long before completion is allowed

_DPAD_STEP_IN     = 0.1  # auger position increment per D-pad up/down press
_ACTUATOR_MIN_IN  = 0.0
_ACTUATOR_MAX_IN  = 4.0
# The Arduino's SoftwareSerial drops bytes while transmitting POS. A freeze MOVE often
# needs several 250 ms resends to land between POS bursts, so we keep retrying for at
# least _FREEZE_MIN_DURATION_S even when the position momentarily equals the target
# (which is trivially true on the cycle the freeze is captured).

class TeleOp:
    """Handles teleoperated robot control: maps gamepad buttons and sticks to subsystem commands."""

    def __init__(self, robot: robot.Robot):
        """Attach to the robot and initialize the loop timer and active drive button state."""
        self.robot = robot
        self._last_update_time = time.monotonic()
        self._active_drive_buttons: set = set()
        self._lt_was_pressed: bool = False
        self._rt_was_pressed: bool = False
        self._lt_freeze_pos: float | None = None
        self._rt_freeze_pos: float | None = None
        self._lt_freeze_time: float = 0.0
        self._rt_freeze_time: float = 0.0

    def _nudge_auger(self, delta_in: float) -> None:
        """Step the auger target by delta_in, clamped to the actuator range.

        Clears any active LT/RT freeze so the freeze retry loop does not overwrite
        the new target on the next cycle.
        """
        target = self.robot.auger.get_auger_angle() + delta_in
        target = max(_ACTUATOR_MIN_IN, min(_ACTUATOR_MAX_IN, target))
        self._lt_freeze_pos = None
        self._rt_freeze_pos = None
        self.robot.auger.set_auger_angle(target)
        robot_params.Telemetry.print_t(
            f"[TeleOp] D-pad nudge {delta_in:+.2f} in → target={target:.2f} in (pos={self.robot.auger.get_auger_angle():.2f} in)",
            prints_per_second=50,
        )

    def on_button_event(self, button, is_pressed):
        """Dispatch a single gamepad button press or release to the appropriate subsystem."""
        if button == Button.DPAD_LEFT:
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
            pass
            # if is_pressed:
            #     task = self.robot.excavation_task
            #     if task.is_running:
            #         task.stop_auto_task()
            #     else:
            #         task.start_auto_task()
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
        elif button == Button.DPAD_DOWN:
            if is_pressed:
                self._nudge_auger(+_DPAD_STEP_IN)
        elif button == Button.DPAD_UP:
            if is_pressed:
                self._nudge_auger(-_DPAD_STEP_IN)

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
            lt_axis = axes.lt
            rt_axis = axes.rt

            # Hysteresis: press requires crossing PRESS threshold; release requires
            # dropping back below RELEASE threshold. Prevents chatter at the edge.
            lt_pressed = lt_axis > _TRIGGER_PRESS_THRESHOLD if not self._lt_was_pressed else lt_axis > _TRIGGER_RELEASE_THRESHOLD
            rt_pressed = rt_axis > _TRIGGER_PRESS_THRESHOLD if not self._rt_was_pressed else rt_axis > _TRIGGER_RELEASE_THRESHOLD

            # LT held → retract actuator fully; release → freeze at current position
            if lt_pressed and not self._lt_was_pressed:
                self.robot.auger.set_auger_min_angle()
                # Clear BOTH freeze states — an active RT freeze retry would
                # otherwise overwrite this MOVE on the next cycle.
                self._lt_freeze_pos = None
                self._rt_freeze_pos = None
                self._lt_was_pressed = True
                robot_params.Telemetry.print_t(f"[TeleOp] LT pressed — commanding min angle. Actuator pos={self.robot.auger.get_auger_angle():.2f} in", prints_per_second=50)
            elif not lt_pressed and self._lt_was_pressed:
                self._lt_freeze_pos = self.robot.auger.get_auger_angle()
                self._lt_freeze_time = time.monotonic()
                self.robot.auger.set_auger_angle(self._lt_freeze_pos)
                self._lt_was_pressed = False
                robot_params.Telemetry.print_t(f"[TeleOp] LT released — freeze at {self._lt_freeze_pos:.2f} in, actuator_moving={self.robot.auger.is_actuator_moving}", prints_per_second=50)

            # RT held → extend actuator fully; release → freeze at current position
            if rt_pressed and not self._rt_was_pressed:
                self.robot.auger.set_auger_max_angle()
                # Clear BOTH freeze states — an active LT freeze retry would
                # otherwise overwrite this MOVE on the next cycle.
                self._lt_freeze_pos = None
                self._rt_freeze_pos = None
                self._rt_was_pressed = True
                robot_params.Telemetry.print_t(f"[TeleOp] RT pressed — commanding max angle. Actuator pos={self.robot.auger.get_auger_angle():.2f} in", prints_per_second=50)
            elif not rt_pressed and self._rt_was_pressed:
                self._rt_freeze_pos = self.robot.auger.get_auger_angle()
                self._rt_freeze_time = time.monotonic()
                self.robot.auger.set_auger_angle(self._rt_freeze_pos)
                self._rt_was_pressed = False
                robot_params.Telemetry.print_t(f"[TeleOp] RT released — freeze at {self._rt_freeze_pos:.2f} in, actuator_moving={self.robot.auger.is_actuator_moving}", prints_per_second=50)

            # Re-stage freeze target each cycle until BOTH (a) it has been active
            # at least _FREEZE_MIN_DURATION_S, and (b) the actuator is within tolerance.
            # The duration gate prevents premature completion on the cycle the freeze
            # is captured (where pos == freeze_pos trivially) and gives the Arduino
            # multiple 250 ms resend windows to actually receive the MOVE command.
            now = time.monotonic()
            for freeze_attr, time_attr, label in (
                ("_lt_freeze_pos", "_lt_freeze_time", "LT"),
                ("_rt_freeze_pos", "_rt_freeze_time", "RT"),
            ):
                freeze_pos: float | None = getattr(self, freeze_attr)
                if freeze_pos is None:
                    continue
                pos = self.robot.auger.get_auger_angle()
                age = now - getattr(self, time_attr)
                within_tol = abs(pos - freeze_pos) <= _FREEZE_TOLERANCE_IN
                if age < _FREEZE_MIN_DURATION_S or not within_tol:
                    self.robot.auger.set_auger_angle(freeze_pos)
                    robot_params.Telemetry.print_t(
                        f"[TeleOp] {label} freeze retry → target={freeze_pos:.2f} in, pos={pos:.2f} in, err={pos - freeze_pos:+.2f}, age={age:.2f}s",
                        prints_per_second=4,
                    )
                else:
                    robot_params.Telemetry.print_t(
                        f"[TeleOp] {label} freeze complete → target={freeze_pos:.2f} in, pos={pos:.2f} in, age={age:.2f}s",
                        prints_per_second=50,
                    )
                    setattr(self, freeze_attr, None)

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
