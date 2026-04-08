import time
from abc import ABC, abstractmethod


class AutoTask(ABC):
    """
    Abstract base class for all autonomous tasks.

    Ownership model
    ---------------
    Tasks may claim subsystems explicitly via claim_subsystem_ownership(),
    called at the start of start_auto_task(). The subsystem blocks any caller
    that is not the current owner.

    State timing
    ------------
    Call transition_to(state) to immediately enter a new state and reset the timer.
    Call wait_for_event(next_state, ...) inside run_task_states()
    to advance to the next state once a condition or timeout is met.

    condition (optional positional):
      bool       — direct boolean value              e.g. sensor.is_ready
      callable   — called with *args, returns bool   e.g. sensor.above, threshold

    timeout (keyword, optional):
      float — seconds since last _transition()       e.g. timeout=3.0

    If both are supplied the state advances when either fires.
    If only timeout is supplied it acts as pure time-based triggering.
    """

    def __init__(self):
        """Initialize the state timer and running flag."""
        self._state_start: float = 0.0
        self._is_running: bool = False

    @property
    def is_running(self) -> bool:
        """True while the task is active (between start_auto_task and stop_auto_task)."""
        return self._is_running

    def transition_to(self, new_state) -> None:
        """Immediately enter new_state and reset the state timer. Prints the transition."""
        old_state = getattr(self, '_state', None)
        self._state = new_state
        self._state_start = time.monotonic()
        task_name = type(self).__name__
        if old_state is not None:
            print(f"[{task_name}] {old_state.name} → {new_state.name}")
        else:
            print(f"[{task_name}] → {new_state.name}")

    def wait_for_event(self, next_state, condition=None, condition_args: tuple = (), *, timeout: float | None = None) -> bool:
        """
        Transition to next_state when a condition or timeout is met.

        Args:
            next_state:      The state to transition to.
            condition:       Optional bool or callable. If callable, called with
                             *condition_args and must return bool.
            condition_args:  Tuple of arguments forwarded to condition if it is callable.
            timeout:         Optional seconds since last _transition(). If only
                             timeout is supplied it acts as pure time-based triggering.

        Returns True and transitions if condition or timeout fires, False otherwise.
        """
        if condition is None:
            cond_met = False
        elif callable(condition):
            cond_met = condition(*condition_args)
        else:
            cond_met = bool(condition)

        timeout_met = timeout is not None and time.monotonic() - self._state_start >= timeout

        if cond_met or timeout_met:
            self.transition_to(next_state)
            return True
        return False

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

