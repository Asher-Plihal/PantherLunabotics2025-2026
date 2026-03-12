import time
from dataclasses import dataclass
from library.util import Util

class PIDController:

    DEFAULT_SETTLING_TIME = 0.2 # seconds

    @dataclass(frozen=True)
    class PIDCoefficients:
        kp: float = 0.0
        ki: float = 0.0
        kd: float = 0.0
        kf: float = 0.0
        iZone: float = 0.0

        def __post_init__(self):
            object.__setattr__(self, 'kp', abs(self.kp))
            object.__setattr__(self, 'ki', abs(self.ki))
            object.__setattr__(self, 'kd', abs(self.kd))
            object.__setattr__(self, 'kf', abs(self.kf))
            object.__setattr__(self, 'iZone', abs(self.iZone))

        def __str__(self):
            return (f"PIDCoefficients(kp={self.kp}, ki={self.ki}, kd={self.kd}, kf={self.kf}, iZone={self.iZone})")

    def __init__(self, PIDCoefficients):
        self.kp = PIDCoefficients.kp
        self.ki = PIDCoefficients.ki
        self.kd = PIDCoefficients.kd
        self.kf = PIDCoefficients.kf
        self.iZone = PIDCoefficients.iZone

        self.no_oscillation = False
        
        # Output limits
        self.output_min = -1.0
        self.output_max = 1.0
        self.output_limit = 1.0

        # Input limits
        self.input_min = None
        self.input_max = None
        self.input_modulus = None
        self.input_bound = None

        # Initialize state variables   
        self.timestamp = 0.0
        self.prevTimestamp = None
        self.previous_error = 0.0
        self.error = 0.0
        self.target_sign = 0.0
        self.output = 0.0
        self.integral_error = 0.0

        # OnTarget variables
        self.settlingStartTime = 0.0
        self.timeoutStartTime = 0.0
        self.timeout = 0.0

    def reset(self):
        # Output limits
        self.output_min = -1.0
        self.output_max = 1.0
        self.output_limit = 1.0
        # Input limits
        self.input_min = None
        self.input_max = None
        self.input_modulus = None
        self.input_bound = None
        # Initialize state variables   
        self.timestamp = 0.0
        self.prevTimestamp = None
        self.previous_error = 0.0
        self.error = 0.0
        self.target_sign = 0.0
        self.output = 0.0
        self.integral_error = 0.0
        # OnTarget variables
        self.settlingStartTime = 0.0
        self.timeoutStartTime = 0.0
        self.timeout = 0.0

    def setNoOscillation(self, no_oscillation):
        self.no_oscillation = no_oscillation
    
    def setOutputRange(self, min_output, max_output):

        if abs(min_output) == abs(max_output):
            self.output_limit = max_output
        
        self.output_min = min_output
        self.output_max = max_output

    def setOutputLimit(self, limit):
        self.setOutputRange(-limit, limit)

    def enableWrapTarget(self, min_input, max_input):
        self.input_min = min_input
        self.input_max = max_input
        self.input_modulus = max_input - min_input
        self.input_bound = self.input_modulus / 2.0
    
    def disableWrapTarget(self):
        self.input_min = None
        self.input_max = None
        self.input_modulus = None
        self.input_bound = None

    def setTimeout(self, timeout):
        self.timeout = timeout
        self.timeoutStartTime = time.monotonic()

    def resetTimeout(self):
        self.timeout = 0.0
        self.timeoutStartTime = 0.0
    
    def isOnTarget(self, tolerance, settlingTime):
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

        print(f"OnTarget: {onTarget} | Error: {abs_error} | Tolerance: {tolerance} | currentTime: {current_time}")
        print(f"SettlingTime: {settlingTime} | SettlingStartTime: {self.settlingStartTime}")
        print(f"timeout: {self.timeout} | timeoutStartTime: {self.timeoutStartTime}")
        if onTarget: self.resetTimeout()
        return onTarget
    
    def isOnTargetDefault(self, tolerance):
        return self.isOnTarget(tolerance, self.DEFAULT_SETTLING_TIME)
    
    def calculate(self, target, state):

        # Delta time calculation
        self.prevTimestamp = self.timestamp
        self.timestamp = time.monotonic()
        delta_time = self.timestamp - self.prevTimestamp if self.prevTimestamp is not None else 0.0
        delta_error = (self.error - self.previous_error) / delta_time if delta_time > 0.0 else 0.0

        # Position error calculation
        self.previous_error = self.error
        self.error = target - state
        if (self.input_bound is not None):
            self.error = self.inputMod(self.error, -self.input_bound, self.input_bound)
        self.target_sign = Util.signum(self.error)

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

        self.output = Util.clip(P_Term + I_Term + D_Term + F_Term, -self.output_limit, self.output_limit)
        print(f"Output: {self.output} P: {P_Term:.3f} | I: {I_Term:.3f} | D: {D_Term:.3f} | F: {F_Term:.3f}")

        return self.output
    
    def inputMod(self, error, lower_bound, upper_bound):
        # Wrap input if its's above the minimum input
        numMax = int((error - lower_bound) / self.input_modulus)
        error -= numMax * self.input_modulus

        # Wrap input if it's below the minimum input
        numMin = int((error - upper_bound) / self.input_modulus)
        error -= numMin * self.input_modulus

        return error