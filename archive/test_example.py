import sys
import os
import time

sys.path.append(os.path.join(os.path.dirname(__file__), 'library/motor_controller/build'))
import motor_controller # type: ignore

def test_example():
    
    mc = motor_controller.MotorController.get_instance("can0")
    motor_id = 6

    config = motor_controller.MotorConfig()
    config.idle_mode = motor_controller.IdleMode.BRAKE
    config.motor_type = motor_controller.MotorType.BRUSHLESS
    config.sensor_type = motor_controller.SensorType.HALL_SENSOR
    config.ramp_rate = 0.0
    config.inverted = False
    config.motor_kv = 480
    config.encoder_counts_per_rev = 4096
    config.smart_current_free_limit = 20.0
    config.smart_current_stall_limit = 80.0

    mc.initialize_motor(motor_id, config)

    print("Starting motor test...")
    start_time = time.time()
    while time.time() - start_time < 5:
        if time.time() - start_time < 2:
            mc.set_motor_duty_cycle(motor_id, 0.1)
        elif time.time() - start_time < 4:
            mc.set_motor_duty_cycle(motor_id, -0.1)
        else:
            mc.set_motor_duty_cycle(motor_id, 0.0)
        mc.update()

    print("Motor test complete.")

if __name__ == "__main__":
    test_example()
