import multiprocessing
import os
import sys
import threading
import time
import pygame
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "onboard_software"))
import client
from library.protocol import Command, Mode, Button, ButtonAction
from library.streaming import StreamMode
from library.panther_dashboard import DashboardView
from robot_params import RobotConfig

'''
Axis 0 - Left Stick (L = -1, R = 1)
Axis 1 - Left Stick (T = -1, D = 1)
Axis 2 - Right Stick (L = -1, R = 1)
Axis 3 - Right Stick (T = -1, D = 1)

Axis 4 - Left Trigger (unpressed = -1, pressed = 1)
Axis 5 - Right Trigger (unpressed = -1, pressed = 1)

Button 0 - A
Button 1 - B
Button 2 - X
Button 3 - Y
Button 4 - LB
Button 5 - RB
Button 6 - Options
Button 7 - Start

Hat 0 - D-Pad (x: L = -1, R = 1 | y: D = -1, U = 1)
'''

class Control:
    def __init__(self, server_ip):
        self.running = True
        self.mode = None
        self.viewer_procs: list[multiprocessing.Process] = []
        self._dashboard_queue: multiprocessing.Queue | None = None
        self.client = client.Client(server_ip) # Connect to the robot's TCP server

        # Initialize the controller
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            print("❌ No joystick detected.")
            sys.exit(1)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print("[Control] 🎮 Controller connected!")

    def run(self):
        self.client_t = threading.Thread(target=self.client.connect)
        self.client_t.start()
        if not self.client.connected.wait(timeout=10):
            print("[Control] Failed to connect to robot within 10 seconds")
            return
        self.client.send_command(Command.READY) # Notify robot that client is ready

        # Auto-launch stream viewers based on RobotConfig
        self._launch_viewers()

        print("[Control] Waiting for mode selection: \n Press A for TELEOP \n Press B for AUTO\n")
        while self.mode is None:
            if not self.client.is_connected:
                print("[Control] Lost connection to robot.")
                self.stop()
                return
            pygame.event.pump()
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    button = event.button
                    if button == 0:
                        print("[Control] Starting in TELEOP mode")
                        self.mode = Mode.TELEOP
                    elif button == 1:
                        print("[Control] Starting in AUTO mode")
                        self.mode = Mode.AUTO
                    elif button == 7:
                        self.client.send_command(Command.SHUTDOWN)
                        self.stop()
                        return
            time.sleep(0.05) # 20 Hz loop

        # Prevent A/B release events from leaking into control loop
        drain_start = time.monotonic()
        while self.joystick.get_button(0) or self.joystick.get_button(1):
            if time.monotonic() - drain_start > 2.0:
                break
            pygame.event.pump()
            time.sleep(0.05) # 20 Hz loop
        pygame.event.clear()
        
        button_map = {
            Button.A: 0,
            Button.B: 1,
            Button.X: 2,
            Button.Y: 3,
            Button.LB: 4,
            Button.RB: 5,
        }

        # D-pad hat state: maps (x, y) offset -> button name
        dpad_map = {
            ( 0,  1): Button.DPAD_UP,
            ( 0, -1): Button.DPAD_DOWN,
            (-1,  0): Button.DPAD_LEFT,
            ( 1,  0): Button.DPAD_RIGHT,
        }
        prev_hat = (0, 0)

        last_command = None

        while self.running:
            if not self.client.is_connected:
                print("[Control] Lost connection to robot.")
                self.stop()
                return
            pygame.event.pump() # Update joystick states

            # Read joystick axes
            left_stick_x = self.joystick.get_axis(0)          # Left Stick left and right
            left_stick_y = self.joystick.get_axis(1)          # Left Stick up and down
            right_stick_x = self.joystick.get_axis(2)   # Right Stick left and right
            right_stick_y = self.joystick.get_axis(3) # Right Stick up and down

            lt = self.joystick.get_axis(4)  # Left Trigger
            rt = self.joystick.get_axis(5)  # Right Trigger

            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:

                    if event.button == 7:
                        self.client.send_command(Command.SHUTDOWN)
                        self.stop()
                        return

                    if event.button == 6:
                        self.mode = Mode.TELEOP if self.mode != Mode.TELEOP else Mode.AUTO
                        print(f"[Control] Switching to {self.mode} mode")

                    for name, btn in button_map.items():
                        if event.button == btn:
                            button_command = (self.mode, name, ButtonAction.PRESSED)
                            self.client.send_command(button_command)

                elif event.type == pygame.JOYBUTTONUP:
                    for name, btn in button_map.items():
                        if event.button == btn:
                            button_command = (self.mode, name, ButtonAction.RELEASED)
                            self.client.send_command(button_command)

                elif event.type == pygame.JOYHATMOTION:
                    curr_hat = event.value
                    # Check each direction independently to support diagonals
                    for (ox, oy), name in dpad_map.items():
                        was_active = (ox != 0 and prev_hat[0] == ox) or (oy != 0 and prev_hat[1] == oy)
                        is_active  = (ox != 0 and curr_hat[0] == ox) or (oy != 0 and curr_hat[1] == oy)
                        if was_active and not is_active:
                            self.client.send_command((self.mode, name, ButtonAction.RELEASED))
                        elif is_active and not was_active:
                            self.client.send_command((self.mode, name, ButtonAction.PRESSED))
                    prev_hat = curr_hat

            commands = (self.mode, round(left_stick_x, 2), round(left_stick_y, 2), round(right_stick_x, 2), round(right_stick_y, 2), round(lt, 2), round(rt, 2))
            if commands != last_command and self.mode == Mode.TELEOP: # For now only TELEOP uses axes
                self.client.send_command(commands)
                last_command = commands

            telemetry = self.client.get_telemetry()
            if telemetry is not None and self._dashboard_queue is not None:
                self._dashboard_queue.put(telemetry)

            time.sleep(0.05) # 20 Hz loop

    def _launch_viewers(self):
        if RobotConfig.useLidar and RobotConfig.lidarStream == StreamMode.REMOTE:
            from subsystems.perception import Lidar
            proc = multiprocessing.Process(target=Lidar.run_viewer, daemon=True)
            proc.start()
            self.viewer_procs.append(proc)
            print("[Control] Launched lidar viewer")

        if RobotConfig.fieldDashboard:
            self._dashboard_queue = multiprocessing.Queue()
            proc = multiprocessing.Process(
                target=DashboardView.run,
                args=(self._dashboard_queue,),
                daemon=True)
            proc.start()
            self.viewer_procs.append(proc)
            print("[Control] Launched field dashboard")

    def _stop_viewers(self):
        for proc in self.viewer_procs:
            proc.terminate()
        for proc in self.viewer_procs:
            proc.join(timeout=3)
            if proc.is_alive():
                proc.kill()
        self.viewer_procs.clear()

    def stop(self):
        print("[Control] Starting shut down")
        self.running = False
        self._stop_viewers()
        pygame.quit()
        self.client.stop()

if __name__ == "__main__":
    Control(RobotConfig.Robot_IP).run()
