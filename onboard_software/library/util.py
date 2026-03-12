import math

class Util:

    # Conversion constants
    MM_PER_INCH = 25.4
    METERS_PER_INCH = MM_PER_INCH / 1000.0
    INCHES_PER_MM = 1.0 / MM_PER_INCH
    INCHES_PER_CM = 10.0 / MM_PER_INCH
    INCHES_PER_METER = 1000.0 / MM_PER_INCH
    
    @staticmethod
    def clip(value, min_value, max_value):
        """
        Clips the input value to be within [min_value, max_value].
        """
        return max(min_value, min(value, max_value))
    
    @staticmethod
    def signum(x):
        """
        Returns the sign of the given number.
        """
        return (x > 0) - (x < 0)
    
    @staticmethod
    def apply_deadzone(value, deadzone=0.1):
        if abs(value) < deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - deadzone) / (1.0 - deadzone)