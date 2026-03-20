import os
import sys
import time
import robot_params
import math

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../library/motor_controller/build'))
import motor_controller  # type: ignore

from library import telemetry_logger
from library.util import Util
from library.subsystem import Subsystem
from robot_params import DriveMode

# Note: if this is changed, update log_row data in print_telemetry as well
_LOG_COLUMNS = [
    "FL Duty Cycle", "FL Velocity (RPM)", "FL Position (ticks)", "FL Current (A)", "FL Temp (°C)", "FL Bus Voltage (V)",
    "FR Duty Cycle", "FR Velocity (RPM)", "FR Position (ticks)", "FR Current (A)", "FR Temp (°C)", "FR Bus Voltage (V)",
    "BL Duty Cycle", "BL Velocity (RPM)", "BL Position (ticks)", "BL Current (A)", "BL Temp (°C)", "BL Bus Voltage (V)",
    "BR Duty Cycle", "BR Velocity (RPM)", "BR Position (ticks)", "BR Current (A)", "BR Temp (°C)", "BR Bus Voltage (V)",
]

# Subsystem Parameters
DEAD_ZONE_THRESHOLD = 0.08

class Drivetrain(Subsystem):

    def __init__(self, mc):
        super().__init__()
        self.slow_turning = False
        self.max_speed = 0.3
        self._last_telemetry_time = 0.0

        self.mc = mc
        self._logger = telemetry_logger.TelemetryLogger("drivetrain")
        self.motor_ids = [7, 4, 1, 2] # FL, FR, BL, BR
        self.motor_labels = list(zip(["FL", "FR", "BL", "BR"], self.motor_ids))

        config = motor_controller.MotorConfig()
        config.idle_mode = motor_controller.IdleMode.BRAKE
        config.motor_type = motor_controller.MotorType.BRUSHLESS
        config.sensor_type = motor_controller.SensorType.HALL_SENSOR
        config.ramp_rate = 0.0
        config.inverted = False
        config.motor_kv = 480
        config.smart_current_free_limit = 20.0
        config.smart_current_stall_limit = 80.0

        self.mc.initialize_motors(self.motor_ids, config)
        for motor_id in self.motor_ids:
            self.mc.reset_motor_position(motor_id)

        if robot_params.RobotConfig.logDrivetrainTelemetry:
            self.start_logging()

    def set_slow_turning(self, slow_turning):
        self.slow_turning = slow_turning

    def set_max_speed(self, max_speed):
        self.max_speed = max_speed

    def set_power(self, owner, front_left_power, front_right_power, back_left_power, back_right_power):
        if not self.check_ownership(owner):
            return
        self.mc.set_motor_duty_cycle(self.motor_ids[0], Util.clip(front_left_power, -self.max_speed, self.max_speed))
        self.mc.set_motor_duty_cycle(self.motor_ids[1], Util.clip(front_right_power, -self.max_speed, self.max_speed))
        self.mc.set_motor_duty_cycle(self.motor_ids[2], Util.clip(back_left_power, -self.max_speed, self.max_speed))
        self.mc.set_motor_duty_cycle(self.motor_ids[3], Util.clip(back_right_power, -self.max_speed, self.max_speed))

    def stop(self):
        self.set_power(None, 0, 0, 0, 0)

    def drive_forward(self):
        self.set_power(None, 0.5, -0.5, 0.5, -0.5)

    def drive_backward(self):
        self.set_power(None, -0.5, 0.5, -0.5, 0.5)

    def strafe_left(self):
        self.set_power(None, -0.5, -0.5, 0.5, 0.5)

    def strafe_right(self):
        self.set_power(None, 0.5, 0.5, -0.5, -0.5)

    def turn_left(self):
        self.set_power(None, -0.5, -0.5, -0.5, -0.5)

    def turn_right(self):
        self.set_power(None, 0.5, 0.5, 0.5, 0.5)

    def fold_out(self):
        self.set_power(None, 0.5, 0.5, -0.5, -0.5)

    def drive_task(self, owner, left_forward, left_strafe, right_forward, right_strafe):  # right_strafe = turning in arcade, strafe in tank; right_forward unused in arcade
        if robot_params.RobotConfig.drivetrainMode == DriveMode.TANK:
            self._drive_task_tank(owner, left_forward, right_forward, left_strafe, right_strafe)
        else:
            self._drive_task_arcade(owner, left_forward, left_strafe, right_strafe)

    def shutdown(self):
        self.force_release()
        self.stop()
        self.stop_logging()

    def _drive_task_arcade(self, owner, y_axis, x_axis, turning_axis):
        y_axis = Util.apply_deadzone(y_axis, DEAD_ZONE_THRESHOLD)
        x_axis = Util.apply_deadzone(x_axis, DEAD_ZONE_THRESHOLD)
        turning_axis = Util.apply_deadzone(turning_axis, DEAD_ZONE_THRESHOLD)

        y = -(math.atan(5 * y_axis) / math.atan(5))
        x = (math.atan(5 * x_axis) / math.atan(5)) * 1.1 # Strafing compensation
        turning = (math.atan(5 * turning_axis) / math.atan(5))

        if self.slow_turning:
            turning *= 0.5 # Slow down turning

        denominator = max(abs(y) + abs(x) + abs(turning), 1)
        front_left_power = (y - x - turning) / denominator
        front_right_power = (y + x + turning) / denominator
        back_left_power = (y + x - turning) / denominator
        back_right_power = (y - x + turning) / denominator

        self.set_power(owner, front_left_power, front_right_power, back_left_power, back_right_power)

    def _drive_task_tank(self, owner, left_forward, right_forward, left_strafe, right_strafe):
        left_forward = Util.apply_deadzone(left_forward, DEAD_ZONE_THRESHOLD)
        right_forward = Util.apply_deadzone(right_forward, DEAD_ZONE_THRESHOLD)
        left_strafe = Util.apply_deadzone(left_strafe, DEAD_ZONE_THRESHOLD)
        right_strafe = Util.apply_deadzone(right_strafe, DEAD_ZONE_THRESHOLD)

        left = -(math.atan(5 * left_forward) / math.atan(5))
        right = -(math.atan(5 * right_forward) / math.atan(5))
        strafe = (math.atan(5 * ((left_strafe + right_strafe) / 2)) / math.atan(5)) * 1.1  # Strafing compensation

        denominator = max(abs(left) + abs(strafe), abs(right) + abs(strafe), 1)
        front_left_power = (left - strafe) / denominator
        front_right_power = (right + strafe) / denominator
        back_left_power = (left + strafe) / denominator
        back_right_power = (right - strafe) / denominator

        self.set_power(owner, front_left_power, front_right_power, back_left_power, back_right_power)

    def start_logging(self):
        self._logger.start_logging(_LOG_COLUMNS)

    def stop_logging(self):
        self._logger.stop_logging()

    def print_telemetry(self, duty_cycle=True, velocity=True, position=True, current=True, temperature=False, voltage=True, interval=0.1):
        if not self._check_telemetry_interval(interval):
            return
        for label, motor_id in self.motor_labels:
            feedback = self.mc.get_motor_feedback(motor_id)
            parts = self._format_motor_feedback(feedback, duty_cycle, velocity, position, current, temperature, voltage)
            if not parts or robot_params.robot_timer is None:
                continue
            print(f"{robot_params.robot_timer.timestamp()} [Drivetrain {label}] " + ", ".join(parts))

    def log_data(self):
        if robot_params.RobotConfig.useTelemetry:
            self.print_telemetry()
        if not self._logger.is_logging:
            return
        row = []
        for _, motor_id in self.motor_labels:
            fb = self.mc.get_motor_feedback(motor_id)
            row.extend([fb.duty_cycle, fb.velocity, fb.position, fb.current, fb.temperature, fb.voltage])
        self._logger.log_row(row)
