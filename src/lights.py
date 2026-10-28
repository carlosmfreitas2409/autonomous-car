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
    POLICE = 8
    BLUE_BREATHE = 9
    CYAN_SWEEP = 10
    YELLOW_PULSE = 11


class DockScene(Enum):
    WAITING = "waiting"
    CHECKING = "checking"
    DELIVERED = "delivered"
    PARKED = "parked"


_SCENE_STRIP = {
    DockScene.WAITING: StripEffect.BLUE_BREATHE,
    DockScene.CHECKING: StripEffect.CYAN_SWEEP,
    DockScene.DELIVERED: StripEffect.GREEN,
    DockScene.PARKED: StripEffect.BLUE_BREATHE,
}
_SCENE_LIT = (DockScene.WAITING, DockScene.DELIVERED, DockScene.PARKED)


@dataclass
class LightsOutput:
    front_left_signal: bool = False
    front_right_signal: bool = False
    front_left_drl: bool = False
    front_right_drl: bool = False

    back_left_signal: bool = False
    back_right_signal: bool = False

    back_left_position: bool = False
    back_right_position: bool = False

    back_left_brake: bool = False
    back_right_brake: bool = False

    reverse_left: bool = False
    reverse_right: bool = False

    low_beam: bool = False

    logo: bool = False

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
        if self._obstacle_stopped:
            signal = TurnSignal.HAZARD
        elif self.signal_override is not None:
            signal = self.signal_override
        else:
            signal = self.turn_signal
        left_active = signal in (TurnSignal.LEFT, TurnSignal.HAZARD)
        right_active = signal in (TurnSignal.RIGHT, TurnSignal.HAZARD)
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
