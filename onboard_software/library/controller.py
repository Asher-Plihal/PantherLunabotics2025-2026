from __future__ import annotations
import os
import sys
from dataclasses import dataclass
from typing import TYPE_CHECKING

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from shared.protocol import Mode, ButtonAction

if TYPE_CHECKING:
    from onboard_software.robot import Robot

@dataclass
class AxisValues:
    x: float = 0.0
    y: float = 0.0
    yaw_rate: float = 0.0
    pitch_rate: float = 0.0
    lt: float = 0.0
    rt: float = 0.0

    def update(self, cmd):
        self.x = cmd[1]
        self.y = cmd[2]
        self.yaw_rate = cmd[3]
        self.pitch_rate = cmd[4]
        self.lt = cmd[5]
        self.rt = cmd[6]

    def __str__(self):
        return (f"Axis State:"
                f"  X: {self.x},"
                f"  Y: {self.y},"
                f"  Yaw Rate: {self.yaw_rate},"
                f"  Pitch Rate: {self.pitch_rate},"
                f"  Left Trigger: {self.lt},"
                f"  Right Trigger: {self.rt}")

class Controller:
    def __init__(self, robot: Robot):
        self.robot = robot
        self.AxisValues = AxisValues()


    def process_axes(self, cmd):
        self.AxisValues.update(cmd)
        #print(f"[Controller] {self.AxisValues.__str__()}")

    def process_buttons(self, cmd):

        mode, button, action = cmd
        is_pressed = (action == ButtonAction.PRESSED)

        if self.robot.current_mode == Mode.TELEOP:
            self.robot.teleop.on_button_event(button, is_pressed)
        elif self.robot.current_mode == Mode.AUTO:
            self.robot.auto.on_button_event(button, is_pressed)

    def process_controller_inputs(self, cmd):
        if len(cmd) > 4 and cmd[0] == Mode.TELEOP: # For now only TELEOP uses axes
            self.process_axes(cmd)
        else:
            self.process_buttons(cmd)
