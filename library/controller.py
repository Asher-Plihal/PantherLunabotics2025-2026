from __future__ import annotations
from dataclasses import dataclass
from typing import TYPE_CHECKING

from library.protocol import Mode, ButtonAction

if TYPE_CHECKING:
    from onboard_software.robot import Robot

@dataclass
class AxisValues:
    """Snapshot of all six gamepad axis values for one control cycle."""

    left_stick_x: float = 0.0
    left_stick_y: float = 0.0
    right_stick_x: float = 0.0
    right_stick_y: float = 0.0
    lt: float = 0.0
    rt: float = 0.0

    def update(self, cmd):
        """Unpack axis values from a raw command tuple."""
        self.left_stick_x = cmd[1]
        self.left_stick_y = cmd[2]
        self.right_stick_x = cmd[3]
        self.right_stick_y = cmd[4]
        self.lt = cmd[5]
        self.rt = cmd[6]

    def __str__(self):
        """Human-readable axis state for logging."""
        return (f"Axis State:"
                f"  Left Stick X: {self.left_stick_x},"
                f"  Left Stick Y: {self.left_stick_y},"
                f"  Right Stick X: {self.right_stick_x},"
                f"  Right Stick Y: {self.right_stick_y},"
                f"  Left Trigger: {self.lt},"
                f"  Right Trigger: {self.rt}")

class Controller:
    """Translates raw command tuples received from the network into robot actions."""

    def __init__(self, robot: Robot):
        """Attach to the robot instance and initialize axis state."""
        self.robot = robot
        self.axis_values = AxisValues()

    def process_axes(self, cmd):
        """Update stored axis values from an axis command tuple."""
        self.axis_values.update(cmd)

    def process_buttons(self, cmd):
        """Dispatch a button event to the handler for the current mode."""
        if len(cmd) != 3:
            print(f"[Controller] Unexpected button command length {len(cmd)}: {cmd}")
            return
        mode, button, action = cmd
        is_pressed = (action == ButtonAction.PRESSED)

        if self.robot.current_mode == Mode.TELEOP:
            self.robot.teleop.on_button_event(button, is_pressed)
        elif self.robot.current_mode == Mode.AUTO:
            self.robot.auto.on_button_event(button, is_pressed)

    def process_controller_inputs(self, cmd):
        """Route an incoming command to axis or button processing based on its shape."""
        if len(cmd) > 4 and cmd[0] == Mode.TELEOP: # For now only TELEOP uses axes
            self.process_axes(cmd)
        else:
            self.process_buttons(cmd)
