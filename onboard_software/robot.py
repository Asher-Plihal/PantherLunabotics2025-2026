from __future__ import annotations
import os
import sys
import subprocess
import server
import threading
import time
import teleOp
import auto
from subsystems import drivetrain
from subsystems import auger
from library import controller
import robot_params

sys.path.append(os.path.join(os.path.dirname(__file__), '../library/motor_controller/build'))
import motor_controller as mc  # type: ignore

def print_network_info():
    """Print the Wi-Fi SSID at startup."""
    try:
        result = subprocess.run(["iwgetid", "-r"], capture_output=True, text=True)
        ssid = result.stdout.strip()
        if ssid:
            print(f"[Network] Connected to Wi-Fi: {ssid}")
        else:
            print("[Network] Not connected to Wi-Fi")
    except Exception as e:
        print(f"[Network] Could not determine Wi-Fi: {e}")

def init_can_bus(interface: str = "can1", bitrate: int = 1_000_000):
    """Bring up the CAN bus interface. Requires root privileges."""
    commands = [
        ["sudo", "ip", "link", "set", interface, "down"],
        ["sudo", "ip", "link", "set", interface, "type", "can", "bitrate", str(bitrate)],
        ["sudo", "ip", "link", "set", interface, "txqueuelen", "1000"],
        ["sudo", "ip", "link", "set", interface, "up"],
    ]
    for cmd in commands:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"[CAN] Failed: {' '.join(cmd)}\n  {result.stderr.strip()}")
            sys.exit(1)
    print(f"[CAN] {interface} is up at {bitrate} bps")

class Robot:
    def __init__(self):
        self.current_mode = None
        self.running = True

        # Print network info so we know where to connect
        print_network_info()

        # Initialize global timer
        robot_params.robot_timer = robot_params.RobotTimer()

        # Bring up CAN bus before accessing hardware
        init_can_bus("can1", 1_000_000)

        # Initialize hardware
        self.motor_controller = mc.MotorController.get_instance("can1")
        self.drivetrain = drivetrain.Drivetrain(self.motor_controller)
        self.auger = auger.Auger(self.motor_controller)

        # Initialize server
        self.server = server.Server()
        threading.Thread(target=self.server.start).start()

        # Initialize controller and run modes
        self.controller = controller.Controller(self)
        self.teleop = teleOp.TeleOp(self)
        self.auto = auto.Auto(self)

        startup_timeout = 60 # seconds
        startup_start = time.monotonic()
        while self.server.get_command() != "READY":
            if time.monotonic() - startup_start > startup_timeout:
                print("[Robot] Timed out waiting for READY from mission control")
                self.stop()
                return
            time.sleep(0.1)
        robot_params.robot_timer.start()
        print("[Robot] Startup complete!")

    def run(self):

        while self.running:

            cmd = self.server.get_command()
            if cmd == "SHUTDOWN":
                self.stop()
                break

            if cmd is not None:
                self.current_mode = cmd[0]
                self.controller.process_controller_inputs(cmd)

            if self.current_mode == "TELEOP":
                self.teleop.run_teleOp_step()
            elif self.current_mode == "AUTO":
                self.auto.run_auto_step()
    
    def stop(self):
        print("[Robot] Stopping robot")
        self.running = False
        self.server.stop()
        self.drivetrain.shutdown()
        self.auger.shutdown()
        
if __name__ == "__main__":
    Robot().run()
