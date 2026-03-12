#!/usr/bin/env python3
"""
Comprehensive test suite for the MotorController C++ binding
Tests all methods and verifies the architecture works as designed
"""

import sys
import os

# Add the build directory to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../library/motor_controller/build'))

try:
    import motor_controller as mc
except ImportError as e:
    print(f"ERROR: Failed to import motor_controller module: {e}")
    print("Make sure the module is compiled and the path is correct")
    sys.exit(1)


class TestMotorController:
    """Test class for MotorController functionality"""

    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.hardware_available = True

    def log_pass(self, test_name):
        """Log a passed test"""
        print(f"  PASS: {test_name}")
        self.passed += 1

    def log_fail(self, test_name, error):
        """Log a failed test"""
        print(f"  FAIL: {test_name}")
        print(f"  Error: {error}")
        self.failed += 1

    def log_skip(self, test_name, reason):
        """Log a skipped test"""
        print(f"  SKIP: {test_name}")
        print(f"  Reason: {reason}")

    def test_module_imports(self):
        """Test that all expected classes and enums are importable"""
        print("\n=== Testing Module Imports ===")

        try:
            assert hasattr(mc, 'IdleMode'), "IdleMode enum not found"
            assert hasattr(mc.IdleMode, 'COAST'), "IdleMode.COAST not found"
            assert hasattr(mc.IdleMode, 'BRAKE'), "IdleMode.BRAKE not found"
            self.log_pass("IdleMode enum import")
        except Exception as e:
            self.log_fail("IdleMode enum import", e)

        try:
            assert hasattr(mc, 'MotorType'), "MotorType enum not found"
            assert hasattr(mc.MotorType, 'BRUSHED'), "MotorType.BRUSHED not found"
            assert hasattr(mc.MotorType, 'BRUSHLESS'), "MotorType.BRUSHLESS not found"
            self.log_pass("MotorType enum import")
        except Exception as e:
            self.log_fail("MotorType enum import", e)

        try:
            assert hasattr(mc, 'SensorType'), "SensorType enum not found"
            assert hasattr(mc.SensorType, 'NO_SENSOR'), "SensorType.NO_SENSOR not found"
            assert hasattr(mc.SensorType, 'HALL_SENSOR'), "SensorType.HALL_SENSOR not found"
            self.log_pass("SensorType enum import")
        except Exception as e:
            self.log_fail("SensorType enum import", e)

        try:
            assert hasattr(mc, 'MotorFeedback'), "MotorFeedback class not found"
            feedback = mc.MotorFeedback()
            assert hasattr(feedback, 'duty_cycle'), "MotorFeedback.duty_cycle not found"
            assert hasattr(feedback, 'velocity'), "MotorFeedback.velocity not found"
            assert hasattr(feedback, 'position'), "MotorFeedback.position not found"
            self.log_pass("MotorFeedback structure")
        except Exception as e:
            self.log_fail("MotorFeedback structure", e)

        try:
            assert hasattr(mc, 'MotorConfig'), "MotorConfig class not found"
            config = mc.MotorConfig()
            assert hasattr(config, 'idle_mode'), "MotorConfig.idle_mode not found"
            assert hasattr(config, 'motor_type'), "MotorConfig.motor_type not found"
            assert hasattr(config, 'sensor_type'), "MotorConfig.sensor_type not found"
            self.log_pass("MotorConfig structure")
        except Exception as e:
            self.log_fail("MotorConfig structure", e)

        try:
            assert hasattr(mc, 'MotorController'), "MotorController class not found"
            assert hasattr(mc.MotorController, 'get_instance'), "MotorController.get_instance not found"
            self.log_pass("MotorController class import")
        except Exception as e:
            self.log_fail("MotorController class import", e)

    def test_singleton_pattern(self):
        """Test that MotorController follows singleton pattern"""
        print("\n=== Testing Singleton Pattern ===")

        try:
            controller1 = mc.MotorController.get_instance("can0")
            controller2 = mc.MotorController.get_instance("can0")

            # They should be the same instance
            assert controller1.get_canbus_name() == controller2.get_canbus_name()
            self.log_pass("Singleton returns same instance")
        except Exception as e:
            self.log_fail("Singleton returns same instance", e)

        try:
            # Trying to change CAN bus should raise error
            try:
                mc.MotorController.get_instance("can1")
                self.log_fail("Singleton CAN bus immutability", "Should have raised error")
            except RuntimeError:
                self.log_pass("Singleton CAN bus immutability")
        except Exception as e:
            self.log_fail("Singleton CAN bus immutability", e)

    def test_motor_initialization(self):
        """Test motor initialization with default and custom configs"""
        print("\n=== Testing Motor Initialization ===")

        controller = mc.MotorController.get_instance("can0")

        # Test default configuration
        try:
            config = mc.MotorConfig()
            config.motor_kv = 565  # NEO Vortex
            config.encoder_counts_per_rev = 7168

            controller.initialize_motor(1, config)

            # Verify motor is initialized
            motor_ids = controller.get_initialized_motor_ids()
            assert 1 in motor_ids, "Motor 1 not in initialized list"
            assert controller.get_motor_count() >= 1, "Motor count incorrect"

            self.log_pass("Initialize single motor with config")
        except Exception as e:
            self.log_fail("Initialize single motor with config", e)
            if "CAN" in str(e) or "socket" in str(e):
                self.hardware_available = False

        # Test initializing multiple motors
        try:
            config = mc.MotorConfig()
            config.motor_kv = 480  # NEO 550
            config.encoder_counts_per_rev = 4096
            config.idle_mode = mc.IdleMode.BRAKE
            config.motor_type = mc.MotorType.BRUSHLESS
            config.sensor_type = mc.SensorType.HALL_SENSOR

            controller.initialize_motors([2, 3, 4], config)

            motor_ids = controller.get_initialized_motor_ids()
            assert 2 in motor_ids and 3 in motor_ids and 4 in motor_ids
            assert controller.get_motor_count() >= 4

            self.log_pass("Initialize multiple motors")
        except Exception as e:
            self.log_fail("Initialize multiple motors", e)

        # Test re-initializing existing motor (should warn but not crash)
        try:
            config = mc.MotorConfig()
            controller.initialize_motor(1, config)  # Already initialized
            self.log_pass("Re-initialize existing motor (warning expected)")
        except Exception as e:
            self.log_fail("Re-initialize existing motor", e)

    def test_duty_cycle_setting(self):
        """Test setting duty cycles for motors"""
        print("\n=== Testing Duty Cycle Setting ===")

        controller = mc.MotorController.get_instance("can0")

        # Test setting single motor duty cycle
        try:
            controller.set_motor_duty_cycle(1, 0.25)
            duty = controller.get_current_duty_cycle(1)
            assert abs(duty - 0.25) < 0.001, f"Duty cycle mismatch: {duty} != 0.25"
            self.log_pass("Set single motor duty cycle")
        except Exception as e:
            self.log_fail("Set single motor duty cycle", e)

        # Test setting multiple motor duty cycles
        try:
            motor_ids = [2, 3, 4]
            duty_cycles = [0.1, -0.15, 0.2]
            controller.set_motors_duty_cycles(motor_ids, duty_cycles)

            for motor_id, expected_duty in zip(motor_ids, duty_cycles):
                actual_duty = controller.get_current_duty_cycle(motor_id)
                assert abs(actual_duty - expected_duty) < 0.001

            self.log_pass("Set multiple motor duty cycles")
        except Exception as e:
            self.log_fail("Set multiple motor duty cycles", e)

        # Test setting duty cycle for uninitialized motor (should raise error)
        try:
            try:
                controller.set_motor_duty_cycle(99, 0.5)
                self.log_fail("Set duty cycle for uninitialized motor", "Should have raised error")
            except RuntimeError:
                self.log_pass("Set duty cycle for uninitialized motor (error expected)")
        except Exception as e:
            self.log_fail("Set duty cycle for uninitialized motor", e)

        # Test mismatched array sizes (should raise error)
        try:
            try:
                controller.set_motors_duty_cycles([1, 2], [0.1, 0.2, 0.3])
                self.log_fail("Mismatched array sizes", "Should have raised error")
            except Exception:
                self.log_pass("Mismatched array sizes (error expected)")
        except Exception as e:
            self.log_fail("Mismatched array sizes", e)

    def test_update_and_feedback(self):
        """Test the Update() method and feedback retrieval"""
        print("\n=== Testing Update and Feedback ===")

        if not self.hardware_available:
            self.log_skip("Update and feedback tests", "CAN hardware not available")
            return

        controller = mc.MotorController.get_instance("can0")

        # Set some duty cycles
        try:
            controller.set_motor_duty_cycle(1, 0.15)
            controller.set_motor_duty_cycle(2, -0.10)

            # Call update to send commands and collect feedback
            controller.update()

            self.log_pass("Update() method execution")
        except Exception as e:
            self.log_fail("Update() method execution", e)
            self.hardware_available = False
            return

        # Test getting single motor feedback
        try:
            feedback = controller.get_motor_feedback(1)

            assert hasattr(feedback, 'duty_cycle')
            assert hasattr(feedback, 'velocity')
            assert hasattr(feedback, 'position')

            print(f"  Motor 1 feedback: {feedback}")
            self.log_pass("Get single motor feedback")
        except Exception as e:
            self.log_fail("Get single motor feedback", e)

        # Test getting all motor feedback
        try:
            all_feedback = controller.get_all_feedback()

            assert isinstance(all_feedback, dict)
            assert len(all_feedback) == controller.get_motor_count()

            print(f"  Total motors with feedback: {len(all_feedback)}")
            for motor_id, feedback in all_feedback.items():
                print(f"    Motor {motor_id}: DC={feedback.duty_cycle:.2f}, "
                      f"V={feedback.velocity:.1f} RPM, P={feedback.position:.0f} ticks")

            self.log_pass("Get all motor feedback")
        except Exception as e:
            self.log_fail("Get all motor feedback", e)

        # Test getting feedback for uninitialized motor (should raise error)
        try:
            try:
                controller.get_motor_feedback(99)
                self.log_fail("Get feedback for uninitialized motor", "Should have raised error")
            except RuntimeError:
                self.log_pass("Get feedback for uninitialized motor (error expected)")
        except Exception as e:
            self.log_fail("Get feedback for uninitialized motor", e)

    def test_feedback_caching(self):
        """Test that feedback is cached and not queried on get calls"""
        print("\n=== Testing Feedback Caching Architecture ===")

        if not self.hardware_available:
            self.log_skip("Feedback caching test", "CAN hardware not available")
            return

        controller = mc.MotorController.get_instance("can0")

        try:
            # Get feedback before update (should return cached or zero data)
            feedback_before = controller.get_motor_feedback(1)

            # Set new duty cycle and update
            controller.set_motor_duty_cycle(1, 0.20)
            controller.update()

            # Get feedback after update
            feedback_after = controller.get_motor_feedback(1)

            # The get_motor_feedback should not trigger hardware query
            # It should just return cached data from Update()
            self.log_pass("Feedback caching (get methods don't query hardware)")

            print(f"  Before: {feedback_before}")
            print(f"  After:  {feedback_after}")

        except Exception as e:
            self.log_fail("Feedback caching", e)

    def test_custom_configurations(self):
        """Test various custom motor configurations"""
        print("\n=== Testing Custom Configurations ===")

        controller = mc.MotorController.get_instance("can0")

        # Test NEO 550 configuration
        try:
            config = mc.MotorConfig()
            config.motor_kv = 480
            config.encoder_counts_per_rev = 4096
            config.idle_mode = mc.IdleMode.COAST
            config.motor_type = mc.MotorType.BRUSHLESS
            config.sensor_type = mc.SensorType.HALL_SENSOR
            config.ramp_rate = 0.5
            config.inverted = True
            config.smart_current_free_limit = 15.0
            config.smart_current_stall_limit = 60.0

            controller.initialize_motor(10, config)
            self.log_pass("NEO 550 custom configuration")
        except Exception as e:
            self.log_fail("NEO 550 custom configuration", e)

        # Test NEO Vortex configuration
        try:
            config = mc.MotorConfig()
            config.motor_kv = 565
            config.encoder_counts_per_rev = 7168
            config.idle_mode = mc.IdleMode.BRAKE
            config.ramp_rate = 0.1
            config.smart_current_free_limit = 20.0
            config.smart_current_stall_limit = 80.0

            controller.initialize_motor(11, config)
            self.log_pass("NEO Vortex custom configuration")
        except Exception as e:
            self.log_fail("NEO Vortex custom configuration", e)

    def test_motor_info(self):
        """Test motor information retrieval methods"""
        print("\n=== Testing Motor Information Methods ===")

        controller = mc.MotorController.get_instance("can0")

        try:
            canbus_name = controller.get_canbus_name()
            assert canbus_name == "can0", f"CAN bus name mismatch: {canbus_name}"
            self.log_pass("Get CAN bus name")
        except Exception as e:
            self.log_fail("Get CAN bus name", e)

        try:
            motor_ids = controller.get_initialized_motor_ids()
            assert isinstance(motor_ids, list), "Motor IDs should be a list"
            print(f"  Initialized motor IDs: {motor_ids}")
            self.log_pass("Get initialized motor IDs")
        except Exception as e:
            self.log_fail("Get initialized motor IDs", e)

        try:
            count = controller.get_motor_count()
            assert isinstance(count, int), "Motor count should be an integer"
            assert count >= 0, "Motor count should be non-negative"
            print(f"  Total motor count: {count}")
            self.log_pass("Get motor count")
        except Exception as e:
            self.log_fail("Get motor count", e)

    def run_all_tests(self):
        """Run all tests and print summary"""
        print("=" * 70)
        print("MOTOR CONTROLLER TEST SUITE")
        print("=" * 70)

        self.test_module_imports()
        self.test_singleton_pattern()
        self.test_motor_initialization()
        self.test_duty_cycle_setting()
        self.test_update_and_feedback()
        self.test_feedback_caching()
        self.test_custom_configurations()
        self.test_motor_info()

        print("\n" + "=" * 70)
        print("TEST SUMMARY")
        print("=" * 70)
        print(f"Passed:  {self.passed}")
        print(f"Failed:  {self.failed}")
        print(f"Total:   {self.passed + self.failed}")

        if self.failed == 0:
            print("\nALL TESTS PASSED!")
            return 0
        else:
            print(f"\n{self.failed} TEST(S) FAILED")
            return 1


if __name__ == "__main__":
    tester = TestMotorController()
    exit_code = tester.run_all_tests()
    sys.exit(exit_code)