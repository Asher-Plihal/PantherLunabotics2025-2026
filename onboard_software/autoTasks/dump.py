from __future__ import annotations
import os
import sys
from enum import Enum, auto
from typing import TYPE_CHECKING

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from library.auto_task import AutoTask

if TYPE_CHECKING:
    import robot


class DumpState(Enum):
    IDLE = auto()
    DRIVING_TO_BERM = auto()
    DUMPING = auto()
    DONE = auto()


# Tunable durations (seconds)
_DRIVE_DURATION_S = 3.0
_DUMP_DURATION_S = 5.0


class DumpTask(AutoTask):
    """
    Auto task: drive to the construction zone and outtake regolith onto the berm.

    Ownership is claimed explicitly in start_auto_task() and released when the
    task finishes or is stopped.

    State machine:
      IDLE → DRIVING_TO_BERM → DUMPING → DONE
    """

    def __init__(self, robot: robot.Robot):
        super().__init__()
        self.robot = robot
        self._state = DumpState.IDLE

    # ------------------------------------------------------------------
    # AutoTask interface
    # ------------------------------------------------------------------

    def start_auto_task(self) -> None:
        self.claim_subsystem_ownership()
        self.transition_to(DumpState.DRIVING_TO_BERM)

    def stop_auto_task(self) -> None:
        self.robot.drivetrain.set_power(self, 0, 0, 0, 0)
        self.robot.auger.set_power(self, 0.0)
        self.release_subsystem_ownership()
        self._state = DumpState.IDLE

    @property
    def is_finished(self) -> bool:
        return self._state == DumpState.DONE

    def run_task_states(self) -> None:
        """Step the state machine. Called once per 50 Hz cycle."""
        match self._state:
            case DumpState.IDLE:
                pass

            case DumpState.DRIVING_TO_BERM:
                self.robot.drivetrain.set_power(self, -0.5, -0.5, -0.5, -0.5)
                if self.wait_for_event(DumpState.DUMPING, timeout=_DRIVE_DURATION_S):
                    self.robot.drivetrain.set_power(self, 0, 0, 0, 0)

            case DumpState.DUMPING:
                self.robot.auger.outtake()
                if self.wait_for_event(DumpState.DONE, timeout=_DUMP_DURATION_S):
                    self.robot.auger.set_power(self, 0.0)
                    self.release_subsystem_ownership()

            case DumpState.DONE:
                pass

    def claim_subsystem_ownership(self) -> None:
        self.robot.drivetrain.claim_ownership(self)
        self.robot.auger.claim_ownership(self)
        if self.robot.pid_drive is not None:
            self.robot.pid_drive.claim_ownership(self)

    def release_subsystem_ownership(self) -> None:
        self.robot.drivetrain.release_ownership(self)
        self.robot.auger.release_ownership(self)
        if self.robot.pid_drive is not None:
            self.robot.pid_drive.release_ownership()
