from __future__ import annotations
import os
import sys
from enum import Enum, auto
from typing import TYPE_CHECKING

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from library.auto_task import AutoTask

from robot_params import Positions as pos

if TYPE_CHECKING:
    import robot


class DumpState(Enum):
    """State machine states for the dump autonomous task."""
    IDLE = auto()
    DRIVING_TO_BERM = auto()
    DUMPING = auto()
    DONE = auto()


# Tunable durations (seconds)
_DRIVE_TIMEOUT_S = 15.0
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
        """Attach to the robot and initialize the task in IDLE state."""
        super().__init__()
        self.robot = robot
        self._is_finished = False
        self._state = DumpState.IDLE

    # ------------------------------------------------------------------
    # AutoTask interface
    # ------------------------------------------------------------------

    def start_auto_task(self, drive: bool = False) -> None:
        """Claim subsystem ownership and enter DRIVING_TO_BERM."""
        self._is_finished = False
        self._is_running = True
        self.claim_subsystem_ownership()
        if drive and self.robot.pid_drive is not None:
            self.transition_to(DumpState.DRIVING_TO_BERM)
        else:
            self.transition_to(DumpState.DUMPING)


    def stop_auto_task(self) -> None:
        """Stop all actuators, release ownership, and return to IDLE."""
        self.robot.drivetrain.stop(self)
        self.robot.auger.set_power(self, 0.0)
        if self.robot.pid_drive is not None:
            self.robot.pid_drive.reset()
        self.release_subsystem_ownership()
        self._is_running = False
        self._state = DumpState.IDLE

    @property
    def is_finished(self) -> bool:
        """True once the task has reached DONE. Reset to False at the start of each run."""
        return self._is_finished

    def run_task_states(self) -> None:
        """Step the state machine. Called once per 50 Hz cycle."""
        match self._state:
            case DumpState.IDLE:
                pass

            case DumpState.DRIVING_TO_BERM:
                assert self.robot.pid_drive is not None
                self.robot.auger.set_auger_transport_angle()
                self.robot.pid_drive.set_target(pos.dumpPos)
                self.wait_for_event(DumpState.DUMPING, self.robot.pid_drive.on_target, timeout=_DRIVE_TIMEOUT_S)

            case DumpState.DUMPING:
                self.robot.auger.set_auger_dump_angle()
                self.robot.auger.outtake(self)
                self.robot.drivetrain.drive_backward(0.02, self)
                if self.wait_for_event(DumpState.DONE, timeout=_DUMP_DURATION_S):
                    self.robot.auger.set_auger_transport_angle()

            case DumpState.DONE:
                self._is_finished = True
                self.stop_auto_task()

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
            self.robot.pid_drive.release_ownership(self)
