"""The one-line answer to "why is it not moving", shared by the GUI and the
dashboard so the two can never tell different stories."""

from typing import Optional

from input.mission import MissionCommand
from input.sign_policy import SignConstraint


def stop_cause(stop_reason: Optional[str], holding: float, hold: bool,
               constraint: SignConstraint, mission_command: MissionCommand) -> Optional[tuple[str, str]]:
    """(where the stop comes from, its text): the controller's own stop
    first, then the sign policy's or the mission's hold (they stop the car
    without the controller knowing why), then a lane-loss hold in progress.
    The dashboard words each source itself; the text is the tuner's."""
    if stop_reason is not None:
        return "controller", stop_reason
    if hold:
        if constraint.hold:
            return "sign", constraint.reason or constraint.state.value
        return "mission", mission_command.reason or mission_command.state.value
    if holding > 0:
        return "holding", f"holding {holding:.1f}s"
    return None


def stop_text(stop_reason: Optional[str], holding: float, hold: bool,
              constraint: SignConstraint, mission_command: MissionCommand) -> Optional[str]:
    cause = stop_cause(stop_reason, holding, hold, constraint, mission_command)
    return None if cause is None else cause[1]
