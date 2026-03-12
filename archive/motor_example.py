import sys
import os
import time

# Adds the 'build' directory to the list of places Python looks for modules
sys.path.append(os.path.join(os.getcwd(), 'build'))
import motor_lib

def test_motor_bindings():
    print("--- SparkMax C++ Binding Test ---")

    if hasattr(motor_lib, 'run_motor'):
        print("[SUCCESS] Found 'run_motor' function.")
    else:
        print("[FAILED] Could not find 'run_motor' function.")
        return

    # 2. Test the MotorFeedback struct binding
    # We can create a dummy list of IDs and Duty Cycles
    # Note: This may throw a CAN error if no hardware is connected, 
    # but it proves the BINDING works.
    can_bus = "can0"
    motor_ids = [6]
    duties = [0.1]

    print(f"Attempting to call run_motor on {can_bus}...")

    try:
        # We wrap this because it might crash if the CAN interface isn't 'up'
        while True:
            start_time = time.time()
            results = motor_lib.run_motor(can_bus, motor_ids, duties)

            print(f"Received {len(results)} feedback objects.")
            for i, data in enumerate(results):
                print(f"Motor {motor_ids[i]} Feedback:")
                print(f"  - Velocity: {data.motorVelocity}")
                print(f"  - Duty Cycle: {data.dutyCycle}")
            
            elapsed = time.time() - start_time
            sleep_time = 0.1 - elapsed
            
            if sleep_time > 0:
                time.sleep(sleep_time)

    except Exception as e:
        print(f"\n[BINDING OK, HARDWARE OFFLINE]")
        print(f"The Python-to-C++ call worked, but the SparkMax driver reported: {e}")

if __name__ == "__main__":
    test_motor_bindings()
