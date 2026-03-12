from __future__ import annotations
import time
import robot_params
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import robot

class Auto:

    def __init__(self, robot: robot.Robot):
        self.robot = robot
        self._last_update_time = time.monotonic()

    # Called only when there is a button event
    def on_button_event(self, button, is_pressed):
        if button == 'A':
            if is_pressed:
                pass
            else:
                pass
        elif button == 'B':
            if is_pressed:
                pass
            else:
                pass
        elif button == 'X':
            if is_pressed:
                pass
            else:
                pass
        elif button == 'Y':
            if is_pressed:
                pass
            else:
                pass
        elif button == 'LB':
            if is_pressed:
                pass
            else:
                pass
        elif button == 'RB':
            if is_pressed:
                pass
            else:
                pass
        elif button == 'DPAD_UP':
            if is_pressed:
                pass
            else:
                pass
        elif button == 'DPAD_DOWN':
            if is_pressed:
                pass
            else:
                pass
        elif button == 'DPAD_LEFT':
            if is_pressed:
                pass
            else:
                pass
        elif button == 'DPAD_RIGHT':
            if is_pressed:
                pass
            else:
                pass

    """Called at 50Hz — put all periodic tasks here."""
    def periodic_loop(self):

        # Update motor controller
        self.robot.motor_controller.update()

    def run_auto_step(self):

        # Update periodic loop
        now = time.monotonic()
        elapsed = now - self._last_update_time
        if elapsed >= robot_params.LoopConfig.UPDATE_PERIOD_S:
            self.periodic_loop()
            self._last_update_time = now
