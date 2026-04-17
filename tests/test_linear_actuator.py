"""
Standalone tester for the linear actuator Arduino Nano over I2C.

Arduino address: 0x08, connected to Jetson I2C bus 1 (pins 3=SDA, 5=SCL).

Write: 2 bytes big-endian target position (0–1023)
Read:  2 bytes big-endian current position (0–1023)

Run on Jetson:
    python3 tests/test_linear_actuator.py
"""

import time
import smbus2


I2C_BUS     = 1       # /dev/i2c-1  (Jetson 40-pin header pins 3/5)
I2C_ADDRESS = 0x40
READ_DELAY  = 0.05    # seconds between write and read


class LinearActuatorTester:
    """Sends target positions to the actuator Nano and reads back position feedback."""

    def __init__(self, bus: int = I2C_BUS, address: int = I2C_ADDRESS) -> None:
        """Open the I2C bus."""
        self._address = address
        self._bus = smbus2.SMBus(bus)
        print(f"Opened I2C bus {bus}, device 0x{address:02X}")

    def send_target(self, target: int) -> None:
        """Send a target position (0–1023) to the Nano."""
        target = max(0, min(1023, target))
        high = (target >> 8) & 0xFF
        low  =  target       & 0xFF
        self._bus.write_i2c_block_data(self._address, high, [low])

    def read_position(self) -> int:
        """Read the current actuator position (0–1023) from the Nano."""
        data = self._bus.read_i2c_block_data(self._address, 0, 2)
        return (data[0] << 8) | data[1]
        # Note: smbus2 read sends a register byte (0x00) before reading.
        # Arduino's requestEvent ignores it and sends 2 bytes back.

    def move_to(self, target: int, timeout: float = 5.0) -> None:
        """Send target and poll until position is within ±5 raw counts or timeout."""
        print(f"\nMoving to target={target}")
        self.send_target(target)
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(READ_DELAY)
            pos = self.read_position()
            error = abs(target - pos)
            print(f"  position={pos:4d}  error={error:4d}")
            if error <= 5:
                print("  Reached target.")
                return
        print("  Timeout — target not reached.")

    def close(self) -> None:
        """Close the I2C bus."""
        self._bus.close()


def main() -> None:
    """Run a simple sweep test: retract → mid → extend → retract."""
    tester = LinearActuatorTester()
    try:
        for target in [0, 512, 1023, 0]:
            tester.move_to(target)
            time.sleep(0.5)
    finally:
        tester.close()
        print("\nDone.")


if __name__ == "__main__":
    main()
