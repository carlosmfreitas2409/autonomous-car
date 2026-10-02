from dataclasses import dataclass
from enum import Enum


class TurnSignal(Enum):
    OFF = "off"
    LEFT = "left"
    RIGHT = "right"
    HAZARD = "hazard"


class StripEffect(Enum):
    OFF = 0
    RED = 1
    GREEN = 2
    BLUE = 3
    STROBE = 4
    RAINBOW = 5
    CYAN = 6
    RED_PULSE = 7


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

    logo: bool = False

    # Doors projector
    projector: bool = False

    strip_effect: StripEffect = StripEffect.OFF


class LightsController:
    _BRAKE_THROTTLE_THRESHOLD = 0.05
    _BRAKE_DECEL_THRESHOLD = 0.15

    def __init__(self):
        self.turn_signal = TurnSignal.OFF
        self.signal_override: TurnSignal | None = None
        self.drl = False
        self.low_beam = False
        self.logo = False
        self.projector = False
        self.reverse = False
        self.strip_effect: StripEffect | None = None
        self._estopped = False
        self._autonomous = True
        self._obstacle_blocked = False
        self._braking = False
        self._prev_throttle = 0.0

    def update(
        self,
        throttle: float,
        *,
        estopped: bool,
        autonomous: bool,
        obstacle_blocked: bool = False,
    ) -> None:
        speed = abs(throttle)
        prev_speed = abs(self._prev_throttle)
        decelerating = (prev_speed - speed) > self._BRAKE_DECEL_THRESHOLD
        coasting = speed < self._BRAKE_THROTTLE_THRESHOLD
        self._braking = estopped or coasting or decelerating
        self.reverse = (not estopped) and throttle < -self._BRAKE_THROTTLE_THRESHOLD
        self._prev_throttle = throttle
        self._estopped = estopped
        self._autonomous = autonomous
        self._obstacle_blocked = obstacle_blocked

    def _resolve_strip(self) -> StripEffect:
        if self.strip_effect is not None:
            return self.strip_effect
        if self._estopped:
            return StripEffect.RED_PULSE
        if self._obstacle_blocked:
            return StripEffect.RED
        return StripEffect.CYAN if self._autonomous else StripEffect.GREEN

    def compute(self) -> LightsOutput:
        signal = self.turn_signal if self.signal_override is None else self.signal_override
        left_active = signal in (TurnSignal.LEFT, TurnSignal.HAZARD)
        right_active = signal in (TurnSignal.RIGHT, TurnSignal.HAZARD)

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
            logo=self.logo,
            projector=self.projector,
            strip_effect=self._resolve_strip(),
        )
