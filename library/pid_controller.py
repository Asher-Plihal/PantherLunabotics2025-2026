import math
import time
from dataclasses import dataclass
from library.util import Util

class PIDController:
    """PIDF controller with configurable output limits, integral zone, heading wrap, and optional sqrt power scaling."""

    DEFAULT_SETTLING_TIME = 0.2 # seconds

    @dataclass(frozen=True)
    class PIDCoefficients:
        """Immutable tuning coefficients for a PIDF controller."""

        kp: float = 0.0
        ki: float = 0.0
        kd: float = 0.0
        kf: float = 0.0
        iZone: float = 0.0

        def __post_init__(self):
            """Validate that all coefficients are non-negative."""
            for field, value in [('kp', self.kp), ('ki', self.ki), ('kd', self.kd),
                                  ('kf', self.kf), ('iZone', self.iZone)]:
                if value < 0:
                    raise ValueError(f"PIDCoefficients: '{field}' must be non-negative, got {value}")

        def __str__(self):
            """Human-readable coefficient summary."""
            return (f"PIDCoefficients(kp={self.kp}, ki={self.ki}, kd={self.kd}, kf={self.kf}, iZone={self.iZone})")

    def __init__(self, coefficients, debug: bool = False):
        """Initialize the controller from a PIDCoefficients tuple and configure default limits."""
        self.kp = coefficients.kp
        self.ki = coefficients.ki
        self.kd = coefficients.kd
        self.kf = coefficients.kf
        self.iZone = coefficients.iZone

        self.no_oscillation = False
        self.debug = debug
        self._squid = False  # Configuration — not cleared by reset()

        # Output limits (configuration — preserved through reset)
        self.output_min = -1.0
        self.output_max = 1.0
        self.output_limit = 1.0
        # Input limits (configuration — preserved through reset)
        self.input_min: float | None = None
        self.input_max: float | None = None
        self.input_modulus: float | None = None
        self.input_bound: float | None = None

        self.reset()

    def reset(self):
        """Reset all transient state (error, integral, timestamp) without touching configuration."""
        # State variables only — output/input limits are preserved
        self.timestamp = None
        self.previous_error = 0.0
        self.error = 0.0
        self.target_sign = 0.0
        self.output = 0.0
        self.integral_error = 0.0
        # OnTarget variables
        self.settlingStartTime = 0.0
        self.timeoutStartTime = 0.0
        self.timeout = 0.0

    def enable_squid(self):
        """Scale output by sqrt(|output|)*sign(output) — smoother low-speed response."""
        self._squid = True

    def disable_squid(self):
        """Disable sqrt power scaling."""
        self._squid = False

    def setNoOscillation(self, no_oscillation):
        """If True, consider on-target once error crosses zero rather than waiting for a settling window."""
        self.no_oscillation = no_oscillation

    def setOutputRange(self, min_output, max_output):
        """Set asymmetric output limits; also updates output_limit when |min| == |max|."""
        if abs(min_output) == abs(max_output):
            self.output_limit = max_output

        self.output_min = min_output
        self.output_max = max_output

    def setOutputLimit(self, limit):
        """Set a symmetric output clamp to ±limit."""
        self.setOutputRange(-limit, limit)

    def enableWrapTarget(self, min_input, max_input):
        """Enable input modulus so the shortest-path error is used (e.g. heading wrap at ±180°)."""
        self.input_min = min_input
        self.input_max = max_input
        modulus = max_input - min_input
        self.input_modulus = modulus
        self.input_bound = modulus / 2.0

    def disableWrapTarget(self):
        """Disable input modulus"""
        self.input_min = None
        self.input_max = None
        self.input_modulus = None
        self.input_bound = None

    def setTimeout(self, timeout):
        """Schedule a forced on-target after timeout seconds, starting now."""
        self.timeout = timeout
        self.timeoutStartTime = time.monotonic()

    def resetTimeout(self):
        """Clear any active timeout."""
        self.timeout = 0.0
        self.timeoutStartTime = 0.0

    def isOnTarget(self, tolerance, settlingTime):
        """Return True if error is within tolerance for the settling window, or if the timeout has fired."""
        onTarget = False

        current_time = time.monotonic()
        abs_error = abs(self.error)

        if (self.no_oscillation):
            if(self.error*self.target_sign <= tolerance):
                onTarget = True
        elif (self.timeout > 0.0 and  current_time - self.timeoutStartTime >= self.timeout):
            onTarget = True
        elif (abs_error > tolerance):
            self.settlingStartTime = time.monotonic()
        elif (settlingTime == 0.0 or current_time >= self.settlingStartTime + settlingTime):
            onTarget = True

        if self.debug:
            print(f"OnTarget: {onTarget} | Error: {abs_error} | Tolerance: {tolerance} | currentTime: {current_time}")
            print(f"SettlingTime: {settlingTime} | SettlingStartTime: {self.settlingStartTime}")
            print(f"timeout: {self.timeout} | timeoutStartTime: {self.timeoutStartTime}")
        if onTarget: self.resetTimeout()
        return onTarget

    def isOnTargetDefault(self, tolerance):
        """isOnTarget using DEFAULT_SETTLING_TIME."""
        return self.isOnTarget(tolerance, self.DEFAULT_SETTLING_TIME)

    def calculate(self, target, state):
        """Compute one PIDF output step for the given target and current state. Returns the clipped output."""
        # Delta time calculation
        prev_time = self.timestamp
        self.timestamp = time.monotonic()
        delta_time = self.timestamp - prev_time if prev_time is not None else 0.0

        # Position error calculation
        self.previous_error = self.error
        self.error = target - state
        if (self.input_bound is not None):
            self.error = self.inputMod(self.error, -self.input_bound, self.input_bound)
        self.target_sign = Util.signum(self.error)

        # Derivative (uses updated error vs previous)
        delta_error = (self.error - self.previous_error) / delta_time if delta_time > 0.0 else 0.0

        # Integral error calculation with iZone consideration
        if (abs(self.error) > self.iZone):
            self.integral_error = 0.0
        elif (self.ki != 0.0):
            self.integral_error += self.error * delta_time

        # PIDF output calculation
        P_Term = self.kp * self.error
        I_Term = self.ki * self.integral_error
        D_Term = self.kd * delta_error
        F_Term = self.kf * target

        clipped = Util.clip(P_Term + I_Term + D_Term + F_Term, -self.output_limit, self.output_limit)
        if self._squid and clipped != 0.0:
            normalized = clipped / self.output_limit
            clipped = math.copysign(math.sqrt(abs(normalized)), normalized) * self.output_limit
        self.output = clipped
        if self.debug:
            print(f"Output: {self.output} P: {P_Term:.3f} | I: {I_Term:.3f} | D: {D_Term:.3f} | F: {F_Term:.3f}")

        return self.output

    def inputMod(self, error, lower_bound, upper_bound):
        """Wrap error into [lower_bound, upper_bound] using the configured input modulus."""
        assert self.input_modulus is not None
        # Wrap input if its's above the minimum input
        numMax = int((error - lower_bound) / self.input_modulus)
        error -= numMax * self.input_modulus

        # Wrap input if it's below the minimum input
        numMin = int((error - upper_bound) / self.input_modulus)
        error -= numMin * self.input_modulus

        return error
