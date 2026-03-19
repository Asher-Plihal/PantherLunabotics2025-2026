from abc import ABC, abstractmethod


class AutoTask(ABC):
    """
    Abstract base class for all autonomous tasks.

    Ownership model
    ---------------
    Tasks may claim subsystems explicitly via claim_subsystem_ownership(),
    called at the start of start_auto_task(), The subsystem blocks any caller 
    that is not the current owner.

    """

    @abstractmethod
    def start_auto_task(self) -> None:
        """Transition from IDLE into the first active state."""

    @abstractmethod
    def stop_auto_task(self) -> None:
        """Immediately stop all actuators, release subsystem ownership, and return to IDLE."""

    @property
    @abstractmethod
    def is_finished(self) -> bool:
        """True once the task has reached its DONE state."""

    @abstractmethod
    def run_task_states(self) -> None:
        """Step the state machine. Called once per 50 Hz cycle."""

    @abstractmethod
    def claim_subsystem_ownership(self) -> None:
        """Claim ownership of all subsystems used by this task. Call from start_auto_task()."""

    @abstractmethod
    def release_subsystem_ownership(self) -> None:
        """Release ownership of all subsystems used by this task."""
