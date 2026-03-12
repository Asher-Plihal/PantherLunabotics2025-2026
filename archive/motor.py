from library.util import Util


class Motor:

    def __init__(self):

        self.hz = 100 # Hz
        self.full_forward_pulse = 2000  # in microseconds
        self.full_reverse_pulse = 1000  # in microseconds
        self.neutral_pulse = 1500  # in microseconds

    def calculate_power(self, power):

        power = Util.clip(power, -1.0, 1.0)

        if power >= 0:
            pulse = self.neutral_pulse + (self.full_forward_pulse - self.neutral_pulse) * power
        else:
            pulse = self.neutral_pulse + (self.full_reverse_pulse - self.neutral_pulse) * abs(power)

        pulse_ms = pulse / 1000.0  # convert to milliseconds
        duty_cycle = pulse_ms * 0.001 * self.hz * 100 # duty cycle in %

        print(f"Power: {power:.2f} | Pulse: {pulse:.1f} µs | Duty: {duty_cycle:.2f}%")
        return duty_cycle
    
    def get_frequency(self):
        return self.hz