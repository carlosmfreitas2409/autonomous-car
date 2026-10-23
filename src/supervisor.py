"""Who is driving: e-stop, mode and the manual driver, owned by the loop.

Three input devices can change these: the tuner GUI and the PS5 (on the car
laptop) and the operator dashboard (over Wi-Fi). None of them holds the state;
each one asks, and the loop applies the answer, so they can never disagree.

The rules:

- E-stop from any source always wins. The GUI and PS5 keep their toggles; the
  dashboard sends the state it wants, so a retransmitted "stop" can never
  resume the car.
- Manual is latched. One action enters it and only an explicit "autonomous"
  leaves it. Nothing automatic ever returns the car to autonomous.
- In manual the `driver` is whoever took it. The pad and the GUI drive with the
  PS5 exactly as before. The dashboard drives with its own stick, under a
  lease: when its drive messages stop arriving for `LEASE_S`, the output is
  zero throttle (steering held) until they come back and the stick has
  returned to neutral, so a reconnect can never lurch the car. This is a
  dead-man on the network, not on the operator's hand.
- The pad in hand next to the car takes over from the dashboard the moment its
  stick or triggers move.

The car boots e-stopped in autonomous mode, as the GUI did.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional
import logging

logger = logging.getLogger(__name__)


LEASE_S = 0.3
_NEUTRAL = 0.05


class Mode(Enum):
    AUTONOMOUS = "autonomous"
    MANUAL = "manual"


class Source(Enum):
    GUI = "gui"
    PAD = "pad"
    DASHBOARD = "dashboard"


@dataclass(frozen=True)
class DriveInput:
    """The dashboard's latest stick, stamped with the loop clock on arrival."""
    steering: float
    throttle: float
    received_at: float


@dataclass(frozen=True)
class ManualOutput:
    steering: float
    throttle: float
    lease_lost: bool = False     # dashboard drive messages stopped arriving
    waiting_neutral: bool = False  # fresh again, but the stick has not returned to 0


class Supervisor:
    def __init__(self):
        self.estopped = True
        self.mode = Mode.AUTONOMOUS
        # The GUI and the PS5 both drive with the pad.
        self.driver = Source.PAD
        self._armed = False
        self._held_steering = 0.0
        self.lease_lost = False

    @property
    def autonomous(self) -> bool:
        return self.mode is Mode.AUTONOMOUS

    # --- requests ------------------------------------------------------

    def toggle_estop(self, source: Source) -> None:
        self.request_estop(not self.estopped, source)

    def request_estop(self, engaged: bool, source: Source) -> None:
        if engaged == self.estopped:
            return
        self.estopped = engaged
        # Resuming with a dashboard trigger held must not launch the car either.
        self._armed = False
        logger.info("e-stop %s (%s)", "engaged" if engaged else "released", source.value)

    def toggle_mode(self, source: Source) -> None:
        self.request_mode(Mode.MANUAL if self.autonomous else Mode.AUTONOMOUS, source)

    def request_mode(self, mode: Mode, source: Source) -> None:
        driver = Source.DASHBOARD if source is Source.DASHBOARD else Source.PAD
        if mode is self.mode and (mode is Mode.AUTONOMOUS or driver is self.driver):
            return
        self.mode = mode
        if mode is Mode.MANUAL:
            self._take(driver, source)
        else:
            self.lease_lost = False
            logger.info("mode: autonomous (%s)", source.value)

    def _take(self, driver: Source, source: Source) -> None:
        self.driver = driver
        # A dashboard driver starts disarmed: a trigger already held when the
        # operator clicked must not launch the car.
        self._armed = False
        self.lease_lost = False
        logger.info("mode: manual, %s drives (%s)", driver.value, source.value)

    # --- manual output -------------------------------------------------

    def manual(self, pad: tuple[float, float], drive: Optional[DriveInput], now: float) -> ManualOutput:
        """The (steering, raw throttle) to send in manual mode. `pad` is
        `ps5.poll()`; the caller scales throttle by base speed as before."""
        if self.driver is Source.DASHBOARD and _moved(pad):
            self._take(Source.PAD, Source.PAD)
        if self.driver is Source.PAD:
            return ManualOutput(*pad)

        fresh = drive is not None and now - drive.received_at <= LEASE_S
        if not fresh:
            if not self.lease_lost:
                logger.warning("dashboard drive lease lost: stopping, still manual")
            self.lease_lost = True
            self._armed = False
            return ManualOutput(self._held_steering, 0.0, lease_lost=True)
        if self.lease_lost:
            logger.info("dashboard drive link back")
        self.lease_lost = False
        self._held_steering = drive.steering
        if not self._armed:
            if abs(drive.throttle) > _NEUTRAL:
                return ManualOutput(drive.steering, 0.0, waiting_neutral=True)
            self._armed = True
        return ManualOutput(drive.steering, drive.throttle)


def _moved(pad: tuple[float, float]) -> bool:
    steering, throttle = pad
    return abs(steering) > _NEUTRAL or abs(throttle) > _NEUTRAL
