from dataclasses import dataclass
from enum import Enum


class TurnSignal(Enum):
    OFF = "off"
    LEFT = "left"
    RIGHT = "right"
    HAZARD = "hazard"


@dataclass
class LightsOutput:
    # Combined front units: turn signal + DRL share one housing but are separate lamps.
    front_left_signal: bool = False
    front_right_signal: bool = False
    front_left_drl: bool = False
    front_right_drl: bool = False

    # Rear turn signals (dedicated lamps).
    back_left_signal: bool = False
    back_right_signal: bool = False

    # Low beam projector.
    low_beam: bool = False

    # Parking + tail + brake (combined rear lamps with 3 brightness levels).
    # 0 = off, 1 = dim (parking/tail), 2 = bright (brake).
    tail_left: int = 0
    tail_right: int = 0

    # Logo projectors.
    logo_left: bool = False
    logo_right: bool = False


class LightsController:
    def __init__(self):
        self.turn_signal = TurnSignal.OFF
        self.drl = False
        self.low_beam = False
        self.logo = False
        self.brake = False
        self.stopped = False

    def compute(self) -> LightsOutput:
        left_active = self.turn_signal in (TurnSignal.LEFT, TurnSignal.HAZARD)
        right_active = self.turn_signal in (TurnSignal.RIGHT, TurnSignal.HAZARD)

        running = self.drl or self.low_beam
        braking = self.brake or self.stopped

        if braking:
            tail = 2
        elif running:
            tail = 1
        else:
            tail = 0

        return LightsOutput(
            front_left_signal=left_active,
            front_right_signal=right_active,
            front_left_drl=self.drl,
            front_right_drl=self.drl,
            back_left_signal=left_active,
            back_right_signal=right_active,
            low_beam=self.low_beam,
            tail_left=tail,
            tail_right=tail,
            logo_left=self.logo,
            logo_right=self.logo,
        )
