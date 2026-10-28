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
    CYAN = 6            # drawn as SAE J3134 / DRIVE PILOT turquoise (firmware strip.cpp)
    RED_PULSE = 7
    POLICE = 8
    BLUE_BREATHE = 9
    CYAN_SWEEP = 10
    YELLOW_PULSE = 11


class DockScene(Enum):
    """What the lamps tell the people at a dock (mission.py decides which)."""
    WAITING = "waiting"         # docked: take your time, the loading zone is lit
    CHECKING = "checking"       # released: about to move, the loading zone closes
    DELIVERED = "delivered"     # unloaded, for a moment
    PARKED = "parked"           # delivered, ready for the next order


_SCENE_STRIP = {
    DockScene.WAITING: StripEffect.BLUE_BREATHE,
    DockScene.CHECKING: StripEffect.CYAN_SWEEP,
    DockScene.DELIVERED: StripEffect.GREEN,
    DockScene.PARKED: StripEffect.BLUE_BREATHE,
}
# The door projectors light the ground where the attendant works, and the
# logo with them, while the car stands at a dock.
_SCENE_LIT = (DockScene.WAITING, DockScene.DELIVERED, DockScene.PARKED)


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
        self._obstacle_stopped = False
        self._sign_stopped = False
        self._speed_test = False
        self._scene: DockScene | None = None
        self._braking = False
        self._prev_throttle = 0.0

    def update(
        self,
        throttle: float,
        *,
        estopped: bool,
        autonomous: bool,
        obstacle_stopped: bool = False,
        sign_stopped: bool = False,
        speed_test: bool = False,
        scene: DockScene | None = None,
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
        self._obstacle_stopped = obstacle_stopped
        self._sign_stopped = sign_stopped
        self._speed_test = speed_test
        self._scene = scene

    def _resolve_strip(self) -> StripEffect:
        if self.strip_effect is not None:
            return self.strip_effect
        if self._estopped:
            return StripEffect.RED_PULSE
        if self._obstacle_stopped or self._sign_stopped:
            return StripEffect.RED
        if self._scene is not None:
            return _SCENE_STRIP[self._scene]
        if not self._autonomous:
            return StripEffect.YELLOW_PULSE
        return StripEffect.POLICE if self._speed_test else StripEffect.CYAN

    def compute(self) -> LightsOutput:
        # Stopped for an obstacle: hazards, over the loop's override (Detour,
        # mission) and the operator's choice; those return once it clears.
        # A stop sign or red light is a normal stop, so no hazards there.
        if self._obstacle_stopped:
            signal = TurnSignal.HAZARD
        elif self.signal_override is not None:
            signal = self.signal_override
        else:
            signal = self.turn_signal
        left_active = signal in (TurnSignal.LEFT, TurnSignal.HAZARD)
        right_active = signal in (TurnSignal.RIGHT, TurnSignal.HAZARD)
        # The speed test is a show: every lamp on, over the operator's choice.
        show = self._speed_test
        drl = self.drl or show
        lit = show or self._scene in _SCENE_LIT

        return LightsOutput(
            front_left_signal=left_active,
            front_right_signal=right_active,
            front_left_drl=drl,
            front_right_drl=drl,
            back_left_signal=left_active,
            back_right_signal=right_active,
            back_left_position=drl,
            back_right_position=drl,
            back_left_brake=self._braking,
            back_right_brake=self._braking,
            reverse_left=self.reverse,
            reverse_right=self.reverse,
            low_beam=self.low_beam or show,
            logo=self.logo or lit,
            projector=self.projector or lit,
            strip_effect=self._resolve_strip(),
        )
