import os
import sys
import time
import robot_params
import math

sys.path.append(os.path.join(os.path.dirname(__file__), '../library/motor_controller/build'))
import motor_controller  # type: ignore

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from library import telemetry_logger
from library.util import Util

# Subsystem Parameters
logTelemetryData = False

# Note: if this is changed, update log_row data in print_telemetry as well
_LOG_COLUMNS = [
    "FL Duty Cycle", "FL Velocity (RPM)", "FL Position (ticks)", "FL Current (A)", "FL Temp (°C)", "FL Bus Voltage (V)",
    "BL Duty Cycle", "BL Velocity (RPM)", "BL Position (ticks)", "BL Current (A)", "BL Temp (°C)", "BL Bus Voltage (V)",
    "FR Duty Cycle", "FR Velocity (RPM)", "FR Position (ticks)", "FR Current (A)", "FR Temp (°C)", "FR Bus Voltage (V)",
    "BR Duty Cycle", "BR Velocity (RPM)", "BR Position (ticks)", "BR Current (A)", "BR Temp (°C)", "BR Bus Voltage (V)",
]

class Drivetrain:

    def __init__(self, mc):
        self.slow_turning = True
        self.max_speed = 0.2
        self._last_telemetry_time = 0.0

        self.mc = mc
        self._logger = telemetry_logger.TelemetryLogger("drivetrain")
        self.left_motor_ids = [7, 1] #front left, back left
        self.right_motor_ids = [4, 2] #front right, back right

        # This config is the same as default so technically it is not needed.
        configL = motor_controller.MotorConfig() # left motors
        configL.idle_mode = motor_controller.IdleMode.BRAKE
        configL.motor_type = motor_controller.MotorType.BRUSHLESS
        configL.sensor_type = motor_controller.SensorType.HALL_SENSOR
        configL.ramp_rate = 0.0
        configL.inverted = False
        configL.motor_kv = 480
        configL.smart_current_free_limit = 20.0
        configL.smart_current_stall_limit = 80.0

        configR = motor_controller.MotorConfig() #right motors
        configR.idle_mode = motor_controller.IdleMode.BRAKE
        configR.motor_type = motor_controller.MotorType.BRUSHLESS
        configR.sensor_type = motor_controller.SensorType.HALL_SENSOR
        configR.ramp_rate = 0.0
        configR.inverted = True
        configR.motor_kv = 480
        configR.smart_current_free_limit = 20.0
        configR.smart_current_stall_limit = 80.0

        self.mc.initialize_motors(self.left_motor_ids, configL)
        self.mc.initialize_motors(self.right_motor_ids, configR)
        for motor_id in self.left_motor_ids + self.right_motor_ids:
            self.mc.reset_motor_position(motor_id)

    def start_logging(self):
        self._logger.start_logging(_LOG_COLUMNS)

    def stop_logging(self):
        self._logger.stop_logging()

    def set_slow_turning(self, slow_turning):
        self.slow_turning = slow_turning

    def set_max_speed(self, max_speed):
        self.max_speed = max_speed

    def stop(self):
        self.set_power(0, 0, 0, 0)

    def shutdown(self):
        self.stop()
        self.stop_logging()

    def set_power(self, front_right_power, front_left_power, back_right_power, back_left_power):
        self.mc.set_motor_duty_cycle(self.right_motor_ids[0], Util.clip(front_right_power, -self.max_speed, self.max_speed))
        self.mc.set_motor_duty_cycle(self.left_motor_ids[0], Util.clip(front_left_power, -self.max_speed, self.max_speed))
        self.mc.set_motor_duty_cycle(self.right_motor_ids[1], Util.clip(back_right_power, -self.max_speed, self.max_speed))
        self.mc.set_motor_duty_cycle(self.left_motor_ids[1], Util.clip(back_left_power, -self.max_speed, self.max_speed))

    def drive_task(self, y_axis, x_axis, turning_axis):
        y_axis = Util.apply_deadzone(y_axis, 0.05)
        x_axis = Util.apply_deadzone(x_axis, 0.05)
        turning_axis = Util.apply_deadzone(turning_axis, 0.05)

        y = (math.atan(5 * y_axis) / math.atan(5))
        x = (math.atan(5 * x_axis) / math.atan(5)) * 1.1 # Strafing compensation
        turning = (math.atan(5 * turning_axis) / math.atan(5))

        if self.slow_turning:
            turning *= 0.5 # Slow down turning

        denominator = max(abs(y) + abs(x) + abs(turning), 1)
        front_right_power = (y - x + turning) / denominator
        front_left_power = (y - x - turning) / denominator
        back_right_power = (y + x + turning) / denominator
        back_left_power = (y + x - turning) / denominator

        self.set_power(front_right_power, front_left_power, back_right_power, back_left_power)

    def print_telemetry(self, duty_cycle=True, velocity=True, position=True, current=True, temperature=False, voltage=True, interval=1):
        now = time.monotonic()
        if now - self._last_telemetry_time < interval:
            return
        self._last_telemetry_time = now

        motors = [
            ("FL", self.left_motor_ids[0]),
            ("BL", self.left_motor_ids[1]),
            ("FR", self.right_motor_ids[0]),
            ("BR", self.right_motor_ids[1]),
        ]

        feedbacks = [(label, self.mc.get_motor_feedback(motor_id)) for label, motor_id in motors]

        for label, feedback in feedbacks:
            parts = []
            if duty_cycle:
                parts.append(f"Duty Cycle: {feedback.duty_cycle:.4f}")
            if velocity:
                parts.append(f"Velocity: {feedback.velocity:.2f} RPM")
            if position:
                parts.append(f"Position: {feedback.position:.1f} ticks")
            if current:
                parts.append(f"Current: {feedback.current:.2f} A")
            if temperature:
                parts.append(f"Temp: {feedback.temperature:.1f} °C")
            if voltage:
                parts.append(f"Bus: {feedback.voltage:.2f} V")
            if parts:
                print(f"{robot_params.robot_timer.timestamp()} [Drivetrain {label}] " + ", ".join(parts))

    def log_data(self):
        if not self._logger.is_logging:
            return
        motors = [
            ("FL", self.left_motor_ids[0]),
            ("BL", self.left_motor_ids[1]),
            ("FR", self.right_motor_ids[0]),
            ("BR", self.right_motor_ids[1]),
        ]
        row = []
        for _, motor_id in motors:
            fb = self.mc.get_motor_feedback(motor_id)
            row.extend([fb.duty_cycle, fb.velocity, fb.position, fb.current, fb.temperature, fb.voltage])
        self._logger.log_row(row)
