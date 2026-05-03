from dataclasses import dataclass
from enum import Enum


class TurnSignal(Enum):
    OFF = "off"
    LEFT = "left"
    RIGHT = "right"
    HAZARD = "hazard"


@dataclass
class LightsOutput:
    # Front combined housings: turn signal + DRL share a housing but are separate lamps.
    front_left_signal: bool = False
    front_right_signal: bool = False
    front_left_drl: bool = False
    front_right_drl: bool = False

    # Rear 4-in-1: signal wire.
    back_left_signal: bool = False
    back_right_signal: bool = False

    # Rear 4-in-1: dim "running" wire (the lamp's always-on position when running
    # lights are engaged). Driven by the same relay as the front DRLs.
    back_left_position: bool = False
    back_right_position: bool = False

    # Rear 4-in-1: brake wire. Independent of position — adds brightness on top
    # of the position level, or lights the lamp directly from off to bright.
    back_left_brake: bool = False
    back_right_brake: bool = False

    # Rear 4-in-1: reverse wire.
    reverse_left: bool = False
    reverse_right: bool = False

    # Front low beam.
    low_beam: bool = False

    # Logo projectors.
    logo_left: bool = False
    logo_right: bool = False


class LightsController:
    _BRAKE_THROTTLE_THRESHOLD = 0.05
    _BRAKE_DECEL_THRESHOLD = 0.15

    def __init__(self):
        self.turn_signal = TurnSignal.OFF
        self.drl = False
        self.low_beam = False
        self.logo = False
        self.reverse = False
        self._braking = False
        self._prev_throttle = 0.0

    def update(self, throttle: float, *, estopped: bool) -> None:
        decelerating = (self._prev_throttle - throttle) > self._BRAKE_DECEL_THRESHOLD
        coasting = throttle < self._BRAKE_THROTTLE_THRESHOLD
        self._braking = estopped or coasting or decelerating
        self._prev_throttle = throttle

    def compute(self) -> LightsOutput:
        left_active = self.turn_signal in (TurnSignal.LEFT, TurnSignal.HAZARD)
        right_active = self.turn_signal in (TurnSignal.RIGHT, TurnSignal.HAZARD)

        return LightsOutput(
            front_left_signal=left_active,
            front_right_signal=right_active,
            front_left_drl=self.drl,
            front_right_drl=self.drl,
            back_left_signal=left_active,
            back_right_signal=right_active,
            back_left_position=self.drl,
            back_right_position=self.drl,
            back_left_brake=self._braking,
            back_right_brake=self._braking,
            reverse_left=self.reverse,
            reverse_right=self.reverse,
            low_beam=self.low_beam,
            logo_left=self.logo,
            logo_right=self.logo,
        )
