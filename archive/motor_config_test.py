"""
motor_config_test.py
Standalone motor configuration test — Motor ID 6.

Modify the CONFIG and TEST SEQUENCE sections below, then run this
script directly.  All telemetry is printed to stdout so you can
watch how config changes affect real motor behaviour.
"""

from __future__ import annotations
import os
import sys
import time
import signal

sys.path.append(os.path.join(os.path.dirname(__file__), 'library/motor_controller/build'))
import motor_controller as mc  # type: ignore
sys.path.append(os.path.join(os.path.dirname(__file__), 'library'))
from telemetry_logger import TelemetryLogger # type: ignore

# ==============================================================
# CONFIG — tweak these values to test different settings
# ==============================================================

MOTOR_ID = 6

# IdleMode.BRAKE  → motor actively resists movement when idle
# IdleMode.COAST  → motor freewheels when idle
IDLE_MODE = mc.IdleMode.BRAKE

# MotorType.BRUSHLESS or MotorType.BRUSHED
MOTOR_TYPE = mc.MotorType.BRUSHLESS

# SensorType.HALL_SENSOR (add others if your build exposes them)
SENSOR_TYPE = mc.SensorType.HALL_SENSOR

# How fast the controller ramps output (0.0 = instant, higher = slower)
RAMP_RATE = 0.0

# Flip the direction of positive power
INVERTED = False

# KV rating of the motor (RPM per volt)
MOTOR_KV = 473

# Encoder ticks per full mechanical revolution
ENCODER_COUNTS_PER_REV = 43

# Current limits (amps)
SMART_CURRENT_FREE_LIMIT  = 20.0
SMART_CURRENT_STALL_LIMIT = 80.0

# ==============================================================
# TEST SEQUENCE — list of (power, hold_seconds) tuples
# power is in [-1.0, 1.0]; adjust steps to suit your needs
# ==============================================================

TEST_STEPS: list[tuple[float, float]] = [
    ( 0.00, 0.5),   # start at neutral — confirm zero output
    ( 0.01, 2.0),   # light forward
    ( 0.00, 1.0),   # back to neutral — watch idle mode behaviour
    (-0.01, 2.0),   # light reverse
    ( 0.00, 0.5),   # final stop
]

# How often telemetry is printed (seconds)
TELEMETRY_INTERVAL = 0.05


# ==============================================================
# Test class
# ==============================================================

class MotorConfigTest:

    def __init__(self) -> None:
        self.motor_id = MOTOR_ID
        self._setpoint = 0.0
        self._running = True
        self._run_start = 0.0

        # Telemetry logger
        self._logger = TelemetryLogger(f"motor_{MOTOR_ID}")

        # Graceful shutdown on Ctrl-C
        signal.signal(signal.SIGINT, self._on_sigint)

        # Connect to CAN bus
        self.mc = mc.MotorController.get_instance("can0")

        # Build and apply config
        config = mc.MotorConfig()
        config.idle_mode               = IDLE_MODE
        config.motor_type              = MOTOR_TYPE
        config.sensor_type             = SENSOR_TYPE
        config.ramp_rate               = RAMP_RATE
        config.inverted                = INVERTED
        config.motor_kv                = MOTOR_KV
        config.encoder_counts_per_rev  = ENCODER_COUNTS_PER_REV
        config.smart_current_free_limit  = SMART_CURRENT_FREE_LIMIT
        config.smart_current_stall_limit = SMART_CURRENT_STALL_LIMIT

        self.mc.initialize_motor(self.motor_id, config)

        # BurnFlash causes the SPARK MAX to reboot — wait for it to come back online
        print("[Init] Waiting for SPARK MAX to reboot after BurnFlash...")
        time.sleep(4.0)

        # Zero the encoder position so ticks start from 0 each run
        self.mc.reset_motor_position(self.motor_id)
        print(f"[Init] Encoder position reset to 0 for motor {self.motor_id}.")

        self._verify_config(config)

    # ----------------------------------------------------------
    # Internal helpers
    # ----------------------------------------------------------

    def _on_sigint(self, sig, frame) -> None:
        print("\n\n[Test] Ctrl-C received — stopping motor.")
        self._running = False
        self.mc.set_motor_duty_cycle(self.motor_id, 0.0)
        self._logger.stop_logging()
        sys.exit(0)

    def _verify_config(self, intended: mc.MotorConfig) -> None:
        MAX_ATTEMPTS = 3
        checks_def = [
            ("Idle Mode",           "idle_mode",                None),
            ("Motor Type",          "motor_type",               None),
            ("Sensor Type",         "sensor_type",              None),
            ("Ramp Rate",           "ramp_rate",                0.001),
            ("Inverted",            "inverted",                 None),
            ("Motor KV",            "motor_kv",                 None),
            ("Encoder CPR",         "encoder_counts_per_rev",   None),
            ("Current Free Limit",  "smart_current_free_limit", 0.5),
            ("Current Stall Limit",  "smart_current_stall_limit", 0.5),
        ]

        for attempt in range(1, MAX_ATTEMPTS + 1):
            print(f"\n[Verify] Reading parameters from SPARK MAX (attempt {attempt}/{MAX_ATTEMPTS})...")
            try:
                actual = self.mc.read_motor_config(self.motor_id)
            except Exception as exc:
                print(f"[Verify] ERROR — could not read config: {exc}")
                return

            results = []
            all_pass = True
            for label, field, tol in checks_def:
                expected  = getattr(intended, field)
                read_back = getattr(actual,   field)
                ok = (abs(float(expected) - float(read_back)) <= tol
                      if tol is not None else expected == read_back)
                if not ok:
                    all_pass = False
                results.append((label, expected, read_back, ok))

            if all_pass or attempt == MAX_ATTEMPTS:
                break

            print(f"[Verify] {sum(1 for *_, ok in results if not ok)} parameter(s) failed"
                  f" — retrying in 1s...")
            time.sleep(1.0)

        sep = "=" * 70
        print(sep)
        print(f"  CONFIG VERIFICATION   |   Motor ID: {self.motor_id}")
        print(f"  {'Parameter':<28} {'Expected':<18} {'Actual':<18} Status")
        print(sep)
        for label, expected, read_back, ok in results:
            status = "PASS" if ok else "FAIL <<<<<"
            print(f"  {label:<28} {str(expected):<18} {str(read_back):<18} {status}")
        print(sep)
        if all_pass:
            print("  All parameters verified successfully.")
        else:
            print("  WARNING: One or more parameters did not match the flashed values!")
        print(sep)

    def _print_config(self) -> None:
        sep = "=" * 58
        print(sep)
        print(f"  MOTOR CONFIG TEST   |   Motor ID: {self.motor_id}")
        print(sep)
        print(f"  Idle Mode           : {IDLE_MODE}")
        print(f"  Motor Type          : {MOTOR_TYPE}")
        print(f"  Sensor Type         : {SENSOR_TYPE}")
        print(f"  Ramp Rate           : {RAMP_RATE}")
        print(f"  Inverted            : {INVERTED}")
        print(f"  Motor KV            : {MOTOR_KV} KV")
        print(f"  Encoder CPR         : {ENCODER_COUNTS_PER_REV} ticks/rev")
        print(f"  Current Free Limit  : {SMART_CURRENT_FREE_LIMIT} A")
        print(f"  Current Stall Limit : {SMART_CURRENT_STALL_LIMIT} A")
        print(sep)

    def _print_telemetry(self) -> None:
        try:
            fb = self.mc.get_motor_feedback(self.motor_id)
            print(
                f"[M{self.motor_id}] "
                f"Set: {self._setpoint:+.3f}  |  "
                f"Duty: {fb.duty_cycle:+.4f}  |  "
                f"Vel: {fb.velocity:8.2f} RPM  |  "
                f"Pos: {fb.position:10.1f} ticks  |  "
                f"Curr: {fb.current:5.2f} A  |  "
                f"Temp: {fb.temperature:5.1f} °C  |  "
                f"Bus: {fb.voltage:5.2f} V"
            )
            if self._logger.is_logging:
                elapsed = time.monotonic() - self._run_start
                mins, secs = divmod(elapsed, 60)
                ts = f"[T+{int(mins):02d}:{secs:05.2f}]"
                self._logger.log_row(ts, [
                    f"{self._setpoint:+.3f}",
                    f"{fb.duty_cycle:+.4f}",
                    f"{fb.velocity:.2f}",
                    f"{fb.position:.1f}",
                    f"{fb.current:.2f}",
                    f"{fb.temperature:.1f}",
                    f"{fb.voltage:.2f}",
                ])
        except Exception as exc:
            print(f"[M{self.motor_id}] Telemetry error: {exc}")

    # ----------------------------------------------------------
    # Public interface
    # ----------------------------------------------------------

    def set_power(self, power: float) -> None:
        self._setpoint = power
        self.mc.set_motor_duty_cycle(self.motor_id, power)

    def stop(self) -> None:
        self.set_power(0.0)

    def run(self) -> None:
        print("\n[Test] Starting sequence. Press Ctrl-C to abort.\n")

        self._run_start = time.monotonic()
        self._logger.start_logging([
            "Setpoint",
            "Duty Cycle",
            "Velocity (RPM)",
            "Position (ticks)",
            "Current (A)",
            "Temperature (°C)",
            "Bus Voltage (V)",
        ])

        for step_num, (power, duration) in enumerate(TEST_STEPS, start=1):
            if not self._running:
                break

            print(f"\n--- Step {step_num}/{len(TEST_STEPS)}: "
                  f"power={power:+.2f}  hold={duration:.1f}s ---")
            self.set_power(power)

            elapsed = 0.0
            while elapsed < duration and self._running:
                self.mc.update()
                self._print_telemetry()
                time.sleep(TELEMETRY_INTERVAL)
                elapsed += TELEMETRY_INTERVAL

        # Always end at zero
        print("\n[Test] Sequence complete — stopping motor.")
        self.stop()

        # One final snapshot after the motor settles
        time.sleep(0.3)
        print("\n--- Final telemetry (motor stopped) ---")
        self._print_telemetry()

        self._logger.stop_logging()
        print("\n[Test] Done.")


if __name__ == "__main__":
    MotorConfigTest().run()
