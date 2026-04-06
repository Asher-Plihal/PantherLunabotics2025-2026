from __future__ import annotations
import os
import sys
from enum import Enum, auto
from typing import TYPE_CHECKING

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from library.auto_task import AutoTask

if TYPE_CHECKING:
    import robot


class ExcavationState(Enum):
    """State machine states for the excavation autonomous task."""
    IDLE = auto()
    DRIVING_TO_ZONE = auto()
    EXCAVATING = auto()
    DONE = auto()


# Tunable durations (seconds)
_DRIVE_DURATION_S = 3.0
_EXCAVATE_DURATION_S = 10.0


class ExcavationTask(AutoTask):
    """
    Auto task: run the auger intake, optionally preceded by autonomous driving.

    Ownership is claimed explicitly in start_auto_task() and released when the
    task finishes or is stopped.

    State machine (drive=True):
      IDLE → DRIVING_TO_ZONE → EXCAVATING → DONE

    State machine (drive=False):
      IDLE → EXCAVATING → DONE
    """

    def __init__(self, robot: robot.Robot):
        """Attach to the robot and initialize the task in IDLE state."""
        super().__init__()
        self.robot = robot
        self._state = ExcavationState.IDLE

    # ------------------------------------------------------------------
    # AutoTask interface
    # ------------------------------------------------------------------

    def start_auto_task(self, drive: bool = True) -> None:
        """
        Start the excavation task.

        Args:
            drive: If True the robot drives autonomously to the excavation zone
                   before running the auger. If False the drivetrain is left to
                   the driver and only the auger sequence runs.
        """
        self.claim_subsystem_ownership()
        if drive:
            self.transition_to(ExcavationState.DRIVING_TO_ZONE)
        else:
            self.transition_to(ExcavationState.EXCAVATING)

    def stop_auto_task(self) -> None:
        """Stop all actuators, release ownership, and return to IDLE."""
        self.robot.drivetrain.set_power(self, 0, 0, 0, 0)
        self.robot.auger.set_power(self, 0.0)
        self.release_subsystem_ownership()
        self._state = ExcavationState.IDLE

    @property
    def is_finished(self) -> bool:
        """True when the task has reached DONE."""
        return self._state == ExcavationState.DONE

    def run_task_states(self) -> None:
        """Step the state machine. Called once per 50 Hz cycle."""
        match self._state:
            case ExcavationState.IDLE:
                pass

            case ExcavationState.DRIVING_TO_ZONE:
                self.robot.drivetrain.set_power(self, -0.5, -0.5, -0.5, -0.5)
                if self.wait_for_event(ExcavationState.EXCAVATING, timeout=_DRIVE_DURATION_S):
                    self.robot.drivetrain.set_power(self, 0, 0, 0, 0)

            case ExcavationState.EXCAVATING:
                self.robot.auger.set_power(self, 0.5)
                if self.wait_for_event(ExcavationState.DONE, timeout=_EXCAVATE_DURATION_S):
                    self.robot.auger.set_power(self, 0.0)
                    self.release_subsystem_ownership()

            case ExcavationState.DONE:
                pass

    def claim_subsystem_ownership(self) -> None:
        """Claim drivetrain, auger, and PID drive (if present)."""
        self.robot.drivetrain.claim_ownership(self)
        self.robot.auger.claim_ownership(self)
        if self.robot.pid_drive is not None:
            self.robot.pid_drive.claim_ownership(self)

    def release_subsystem_ownership(self) -> None:
        """Release drivetrain, auger, and PID drive (if present)."""
        self.robot.drivetrain.release_ownership(self)
        self.robot.auger.release_ownership(self)
        if self.robot.pid_drive is not None:
            self.robot.pid_drive.release_ownership()

