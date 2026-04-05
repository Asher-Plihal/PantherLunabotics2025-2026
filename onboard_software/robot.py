from __future__ import annotations
import os
import sys
import subprocess
import threading
import time
import teleOp
import auto
import robot_params
import server
from subsystems import drivetrain
from subsystems import auger
from subsystems import perception
from subsystems import dashboard

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from library import controller
from library.protocol import Command, Mode
from library.pid_drive import PIDDrive

from library.uwb_localizer import UWBLocalizer

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'library', 'motor_controller', 'build'))
import motor_controller as mc  # type: ignore

class Robot:
    def __init__(self):
        self.current_mode = None
        self.running = True

        # Print network info so we know where to connect
        setup_network()

        # Initialize global timer
        robot_params.robot_timer = robot_params.RobotTimer()
        robot_params.print_config()

        # Bring up CAN bus before accessing hardware
        init_can_bus(robot_params.RobotConfig.Robot_CAN_Interface, 1_000_000)

        # Initialize hardware
        self.motor_controller = mc.MotorController.get_instance(robot_params.RobotConfig.Robot_CAN_Interface)
        self.drivetrain = drivetrain.Drivetrain(self.motor_controller)
        self.auger = auger.Auger(self.motor_controller)

        # Initialize perception (lidar/camera streams)
        self.perception = perception.Perception()
        self.perception.start()

        # Initialize field dashboard (no-op if RobotConfig.fieldDashboard is False)
        self.dashboard = dashboard.Dashboard(self.server)

        # Initialize server
        self.server = server.Server()
        threading.Thread(target=self.server.start, daemon=True).start()

        # Initialize PID drive (localizer start is handled inside PIDDrive.__init__)
        self.pid_drive: PIDDrive | None = None
        if robot_params.RobotConfig.usePIDDrive and robot_params.RobotConfig.useDrivetrain:
            cfg = robot_params.RobotConfig
            localizer = UWBLocalizer(
                ax=cfg.uwbAnchorAX, ay=cfg.uwbAnchorAY,
                bx=cfg.uwbAnchorBX, by=cfg.uwbAnchorBY,
                tag_sep=cfg.uwbTagSep, forward_offset=cfg.uwbForwardOffset,
                use_hardware=True,
                left_port=cfg.uwbLeftPort, right_port=cfg.uwbRightPort,
            )
            self.pid_drive = PIDDrive(
                self.drivetrain, localizer,
                x_coeffs=cfg.pidXCoeffs,
                y_coeffs=cfg.pidYCoeffs,
                h_coeffs=cfg.pidHCoeffs,
            )
            self.dashboard.set_uwb_source(localizer)

        # Initialize controller and run modes
        self.controller = controller.Controller(self)
        self.teleop = teleOp.TeleOp(self)
        self.auto = auto.Auto(self)

        startup_timeout = 60 # seconds
        startup_start = time.monotonic()
        while self.server.get_command() != Command.READY:
            if time.monotonic() - startup_start > startup_timeout:
                print("[Robot] Timed out waiting for READY from mission control")
                self.stop()
                return
            time.sleep(0.1)
        robot_params.robot_timer.start()
        print("[Robot] Startup complete!")

    def run(self):

        previous_mode = None

        while self.running:

            cmd = self.server.get_command()
            if cmd == Command.SHUTDOWN:
                self.stop()
                break

            if cmd is not None:
                # Skip plain string commands (READY, etc.) — only process list commands
                if isinstance(cmd, (list, tuple)):
                    self.current_mode = cmd[0]
                    self.controller.process_controller_inputs(cmd)

            # On mode transition, clear all subsystem ownership so the
            # incoming mode starts with a clean slate
            if self.current_mode != previous_mode:
                self.drivetrain.force_release()
                self.auger.force_release()
                previous_mode = self.current_mode

            if self.current_mode == Mode.TELEOP:
                self.teleop.run_teleOp_step()
            elif self.current_mode == Mode.AUTO:
                self.auto.run_auto_step()

            time.sleep(0.01)
    
    def stop(self):
        print("[Robot] Stopping robot")
        self.running = False
        if self.pid_drive is not None:
            self.pid_drive.shutdown()
        self.perception.stop()
        self.server.stop()
        self.drivetrain.shutdown()
        self.auger.shutdown()
        self.dashboard.shutdown()

def setup_network():
    """Connect to the configured network via nmcli (skips if already connected), then print the active SSID."""
    selected = robot_params.NetworkConfig.SELECTED_NETWORK
    connection_name = robot_params.NetworkConfig.NETWORKS.get(selected)

    if not connection_name:
        raise ValueError(f"[Network] Unknown network key '{selected}' — check NetworkConfig.NETWORKS in robot_params.py")

    # Check if the target profile is already active
    already_connected = False
    try:
        active = subprocess.run(
            ["nmcli", "-t", "-f", "NAME", "connection", "show", "--active"],
            capture_output=True, text=True
        )
        active_names = [line.strip() for line in active.stdout.splitlines()]
        already_connected = connection_name in active_names
    except Exception:
        pass

    if already_connected:
        print(f"[Network] Already connected to '{connection_name}'")
    else:
        print(f"[Network] Connecting to '{selected}' ({connection_name})...")
        try:
            result = subprocess.run(
                ["sudo", "nmcli", "connection", "up", connection_name],
                capture_output=True, text=True, timeout=20
            )
            if result.returncode == 0:
                print(f"[Network] Connected to '{connection_name}'")
            else:
                print(f"[Network] ERROR: Failed to connect to '{selected}' ({connection_name}): {result.stderr.strip()}")
                sys.exit(1)
        except subprocess.TimeoutExpired:
            print(f"[Network] ERROR: Connection attempt timed out for '{connection_name}'")
            sys.exit(1)
        except Exception as e:
            print(f"[Network] ERROR: Could not run nmcli: {e}")
            sys.exit(1)

    try:
        result = subprocess.run(["iwgetid", "-r"], capture_output=True, text=True)
        ssid = result.stdout.strip()
        if ssid:
            print(f"[Network] Active Wi-Fi SSID: {ssid}")
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
        
if __name__ == "__main__":
    robot = None
    try:
        robot = Robot()
        robot.run()
    except KeyboardInterrupt:
        print("\n[Robot] Interrupted by user")
    except Exception as e:
        print(f"[Robot] Fatal error: {e}")
    finally:
        if robot is not None:
            robot.stop()
