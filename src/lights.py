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
    def __init__(self):
        self.turn_signal = TurnSignal.OFF
        self.drl = False
        self.low_beam = False
        self.logo = False
        self.brake = False
        self.reverse = False
        self.stopped = False

    def compute(self) -> LightsOutput:
        left_active = self.turn_signal in (TurnSignal.LEFT, TurnSignal.HAZARD)
        right_active = self.turn_signal in (TurnSignal.RIGHT, TurnSignal.HAZARD)
        braking = self.brake or self.stopped

        return LightsOutput(
            front_left_signal=left_active,
            front_right_signal=right_active,
            front_left_drl=self.drl,
            front_right_drl=self.drl,
            back_left_signal=left_active,
            back_right_signal=right_active,
            back_left_position=self.drl,
            back_right_position=self.drl,
            back_left_brake=braking,
            back_right_brake=braking,
            reverse_left=self.reverse,
            reverse_right=self.reverse,
            low_beam=self.low_beam,
            logo_left=self.logo,
            logo_right=self.logo,
        )
