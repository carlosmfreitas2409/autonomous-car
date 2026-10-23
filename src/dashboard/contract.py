"""What the cart tells the dashboard, and what the dashboard may ask.

`build_state` turns the loop's own objects into one JSON-ready dict. It is a
pure function of what the loop already computed: nothing here perceives,
decides or estimates anything the car does not already know. A slice whose
source is absent is None (or reports itself offline), never a made-up value.

`parse_command` validates one browser message. Mirror of both lives in
`web/src/types.ts`; bump CONTRACT_VERSION on any breaking change.
"""

from dataclasses import dataclass, field
from typing import Any, Optional

from cargo import CargoReading
from config import Config
from input.mission import LETTERS, LETTER_EXIT, Mission, MissionCommand, Stage, _exit_after
from input.sign_policy import SignConstraint
from lights import LightsController, LightsOutput, StripEffect, TurnSignal
from obstacle import _SLOW_CM, _STOP_CM
from path import HOME_PATH, Path
from supervisor import Mode, Supervisor

CONTRACT_VERSION = 2

# The loop does not measure speed: there is no encoder on the car. Once a
# straight has been timed at a known PWM, put the m/s at full PWM here and
# the dashboard shows an estimate, labelled as one. None shows output only.
EST_MPS_AT_FULL_PWM: Optional[float] = None

_EXITS = (1, 2)


def layout() -> dict:
    """The track as the mission understands it: the exits in driving order
    and where each letter is. Letters outside every detour sit on the main
    road after the last exit (see mission.py). The map draws only this."""
    return {
        "exits": list(_EXITS),
        "after": {str(k): _exit_after(k, False) for k in _EXITS},
        "letters": {letter: LETTER_EXIT[letter] for letter in LETTERS},
    }


def segment(mission: Mission, path: Path, carrying: bool) -> Optional[dict]:
    """Where the car is, as far as the mission's exit count says. Only known
    while a mission is running: an idle mission counts nothing.

    While the Detour is armed, or its profile is still carried through the
    merge, the car is in (or entering) the detour of the exit just decided,
    or of the next one if none has been yet; otherwise it is on the main road
    heading for `next_exit`.
    """
    if mission.stage is Stage.IDLE:
        return None
    if path is Path.DETOUR or carrying:
        last = mission.last_decision[1] if mission.last_decision else mission.next_exit
        return {"kind": "detour", "exit": last}
    return {"kind": "main", "towards": mission.next_exit}


def last_decision(mission: Mission) -> Optional[dict]:
    """The choice made at the latest detour sign, with that sign's track id:
    `next` has already moved on to the following exit by then."""
    if mission.last_decision is None:
        return None
    sign, exit_number, wanted = mission.last_decision
    return {"sign": sign, "exit": exit_number, "take": "detour" if wanted is Path.DETOUR else "straight"}


def next_decision(mission: Mission) -> Optional[dict]:
    """The choice the mission will make at the next detour sign."""
    target = mission.target
    if target is None:
        return None
    exit_number = mission.next_exit
    enter = LETTER_EXIT[target] == exit_number
    return {"exit": exit_number, "take": "detour" if enter else "straight", "target": target}


def route_progress(mission: Mission) -> int:
    """Index of the current step in `mission.route`: exit steps are passed
    one per decision, the pickup step once the stage moved on to delivery."""
    decisions = mission.decisions
    for index, step in enumerate(mission.route):
        if step.startswith("exit "):
            if decisions <= 0:
                return index
            decisions -= 1
        elif step.endswith(": pickup"):
            if mission.stage is Stage.PICKUP:
                return index
        elif mission.stage is not Stage.DONE:
            return index
    return len(mission.route)


def dock(mission: Mission) -> Optional[dict]:
    """Where the car stands waiting for a release, as the mission holds it.
    `since` is on the loop's clock, like the state's `t`."""
    current = mission.dock
    if current is None:
        return None
    return {
        "letter": current.letter,
        "role": None if current.role is None else current.role.value,
        "since": round(current.since, 3),
        "released": current.released_at is not None,
        "lid_opened": current.lid_opened,
        "check_failed": current.check_failed,
        "override": current.override,
    }


def cargo_state(cargo: Optional[CargoReading]) -> Optional[dict]:
    """The compartment as the cargo node reports it; None with no node."""
    if cargo is None:
        return None
    return {"grams": cargo.grams, "stable": cargo.stable, "lid_closed": cargo.lid_closed,
            "unlocked": cargo.unlocked, "fault": cargo.fault}


def _round(value: Optional[float], digits: int = 3) -> Optional[float]:
    return None if value is None else round(float(value), digits)


def _track(track) -> dict:
    return {
        "id": track.id,
        "label": track.label,
        "distance_m": _round(track.distance_m, 2),
        "bearing_deg": _round(track.bearing_deg, 1),
        "confirmed": track.confirmed,
        "rejected": track.rejected,
        "variant": track.variant,
        "light": track.light_state.name.lower(),
        "reason": track.light_reason if track.rejected else "",
    }


def _signal(output: Optional[LightsOutput]) -> Optional[str]:
    if output is None:
        return None
    left = output.front_left_signal or output.back_left_signal
    right = output.front_right_signal or output.back_right_signal
    if left and right:
        return "hazard"
    if left:
        return "left"
    if right:
        return "right"
    return "off"


def light_settings(controller: LightsController) -> dict:
    """What the operator chose, as opposed to what the lamps show: the car
    overrides the signal and the strip while it needs them."""
    return {
        "turn_signal": controller.turn_signal.value,
        "drl": controller.drl,
        "low_beam": controller.low_beam,
        "logo": controller.logo,
        "projector": controller.projector,
        "strip": "auto" if controller.strip_effect is None else controller.strip_effect.name.lower(),
    }


def build_state(*, now: float, fps: float, supervisor: Supervisor, manual_note: Optional[str],
                steering: float, throttle: float, stop: Optional[tuple[str, str]],
                path: Path, detour_stage: str, carrying: bool, profile_label: str, holding: float,
                perc, roi: Optional[tuple[float, float]],
                snapshot, constraint: SignConstraint, mission: Mission, mission_command: MissionCommand,
                applied: bool, guard, lights_output: Optional[LightsOutput], lights_signal: Optional[TurnSignal],
                telemetry, serial_open: bool, battery=None,
                lights_controller: Optional[LightsController] = None,
                tuning: Optional[dict] = None, cargo: Optional[CargoReading] = None) -> dict:
    """One state message. `perc` is None while e-stopped (the drive camera is
    not read then); `roi` is (x0, x1) as fractions of the frame width. `stop`
    is status.stop_cause's (source, text), None while nothing stops the car."""
    fw = None
    if telemetry is not None:
        fw = {
            "state": telemetry.state,
            "ms_since_command": telemetry.ms_since_command,
            "throttle_pwm": telemetry.throttle_pwm,
            "steer_pwm": telemetry.steer_pwm,
            "angle_raw": telemetry.angle_raw,
            "fault": telemetry.fault,
            "age_s": _round(now - telemetry.received_at, 2),
        }

    applied_throttle = throttle if fw is None else fw["throttle_pwm"] / 255.0
    speed = None
    if EST_MPS_AT_FULL_PWM is not None:
        speed = round(abs(applied_throttle) * EST_MPS_AT_FULL_PWM, 2)

    lane = None
    if perc is not None:
        height, width = perc.display_frame.shape[:2]
        roi_center = None if roi is None else int(round((roi[0] + roi[1]) / 2 * width))
        lane = {
            "state": perc.lane_state,
            "offset_px": perc.lane_offset,
            "curvature": _round(perc.lane_curvature, 4),
            "anchor": perc.anchor.value if hasattr(perc.anchor, "value") else str(perc.anchor),
            "profile_ready": perc.profile_ready,
            "provisional": perc.provisional,
            "width_ratio": _round(perc.width_ratio, 3),
            "rows_found": perc.rows_found,
            "both_rows": perc.both_rows,
            "rows": [[int(y), int(x), bool(both)] for y, x, both in perc.row_centers],
            "frame": [int(width), int(height)],
            "roi_center_x": roi_center,
            # What the camera sees further up the frame, for drawing only:
            # (row, left x, right x) per band row and the width profile.
            "band": [[int(y), int(left), int(right)] for y, left, right in getattr(perc, "band_edges", ())],
            "width_profile": None if getattr(perc, "width_profile", None) is None
            else [_round(perc.width_profile[0], 5), _round(perc.width_profile[1], 2)],
        }

    signs = {
        "camera_open": snapshot.camera_open,
        "detector": snapshot.detector_available,
        "status": snapshot.status,
        "fps": _round(snapshot.fps, 1),
        "age_s": None if snapshot.timestamp <= 0 else _round(snapshot.age(now), 2),
        "tracks": [_track(t) for t in snapshot.tracks],
    }

    obstacle = {
        "distances_cm": [None if d is None else round(d) for d in guard.distances],
        "label": guard.label,
        "stopped": guard.stopped,
        "scale": _round(guard.scale, 2),
        "stale": guard.stale,
        "slow_cm": list(_SLOW_CM),
        "stop_cm": _STOP_CM,
    }

    lights = None
    if lights_output is not None:
        lights = {
            "signal": _signal(lights_output),
            "brake": bool(lights_output.back_left_brake or lights_output.back_right_brake),
            "strip": lights_output.strip_effect.name.lower(),
            "requested": None if lights_signal is None else lights_signal.name.lower(),
            "settings": None if lights_controller is None else light_settings(lights_controller),
        }

    return {
        "type": "state",
        "version": CONTRACT_VERSION,
        "t": round(now, 3),
        "drive": {
            "mode": supervisor.mode.value,
            "estopped": supervisor.estopped,
            "driver": supervisor.driver.value if supervisor.mode is Mode.MANUAL else None,
            "lease_lost": supervisor.lease_lost,
            "manual_note": manual_note,
            "steering": _round(steering),
            "throttle": _round(throttle),
            "applied_throttle": _round(applied_throttle),
            "speed_est_mps": speed,
            "stop_text": "e-stop" if supervisor.estopped else None if stop is None else stop[1],
            "stop_source": "estop" if supervisor.estopped else None if stop is None else stop[0],
            "fw": fw,
        },
        "path": {
            "path": path.value,
            "home": HOME_PATH.value,
            "detour_stage": detour_stage,
            "carrying": carrying,
            "profile": profile_label,
            "holding_s": _round(holding, 2),
        },
        "lane": lane,
        "signs": signs,
        "policy": {
            "state": constraint.state.value,
            "reason": constraint.reason,
            "scale": _round(constraint.scale, 2),
            "hold": constraint.hold,
            "stale": constraint.stale,
            "applied": applied,
        },
        "mission": {
            "stage": mission.stage.value,
            "pickup": mission.pickup,
            "delivery": mission.delivery,
            "target": mission.target,
            "next_exit": mission.next_exit,
            "route": list(mission.route),
            "step": route_progress(mission),
            "state": mission_command.state.value,
            "reason": mission_command.reason,
            "scale": _round(mission_command.scale, 2),
            "hold": mission_command.hold,
            "applied": applied,
            "next": next_decision(mission),
            "decided": last_decision(mission),
            "segment": segment(mission, path, carrying),
            "check": mission_command.check,
            "dock": dock(mission),
            "parked_at": mission.parked_at,
            "unlock": mission.unlock,
            "manifest_g": mission.manifest_g,
        },
        "cargo": cargo_state(cargo),
        "obstacle": obstacle,
        "lights": lights,
        "health": {
            "loop_fps": _round(fps, 1),
            "serial_open": serial_open,
            "drive_camera": perc is not None or supervisor.estopped,
        },
        # The throttle on straights (also the manual stick's full scale) and
        # on the sharpest curves, as the loop holds them.
        "speed": None if tuning is None else {
            "base": _round(tuning["base_speed"]),
            "min": _round(tuning["min_speed"]),
            "defaults": {"base": Config.base_speed, "min": Config.min_speed},
        },
        "pid": None if tuning is None else {
            **{gain: _round(tuning[gain], 5) for gain in PID_MAX},
            "defaults": {gain: getattr(Config, gain) for gain in PID_MAX},
            "max": PID_MAX,
        },
        # The firmware's battery gauge; None until a `B` line has arrived.
        "battery": None if battery is None else {
            "percent": battery.percent,
            "volts": _round(battery.volts, 2),
            "age_s": _round(now - battery.received_at, 1),
        },
    }


# --- commands --------------------------------------------------------------

_PATHS = {p.value: p for p in Path}
_MODES = {m.value: m for m in Mode}
_SIGNALS = {s.value: s for s in TurnSignal}
_STRIPS = {"auto": None, **{e.name.lower(): e for e in StripEffect}}
_LIGHT_SWITCHES = ("drl", "low_beam", "logo", "projector")
_SPEEDS = {"base": "base_speed", "min": "min_speed"}
# The tuner's slider ranges.
PID_MAX = {"kp": 0.1, "ki": 0.05, "kd": 0.1}


@dataclass
class Command:
    type: str
    client: int = 0
    id: Optional[int] = None
    args: dict[str, Any] = field(default_factory=dict)


def _clamp(value: Any, low: float = -1.0) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if value != value:  # NaN
        return None
    return max(low, min(1.0, float(value)))


def parse_command(message: Any) -> Optional[Command]:
    """A validated command, or None for anything malformed. Unknown types are
    rejected rather than ignored, so a typo shows up as a nack."""
    if not isinstance(message, dict):
        return None
    kind = message.get("type")
    command_id = message.get("id") if isinstance(message.get("id"), int) else None
    if kind == "estop":
        engaged = message.get("engaged")
        if not isinstance(engaged, bool):
            return None
        return Command("estop", id=command_id, args={"engaged": engaged})
    if kind == "mode":
        mode = _MODES.get(message.get("mode"))
        if mode is None:
            return None
        return Command("mode", id=command_id, args={"mode": mode})
    if kind == "drive":
        steering, throttle = _clamp(message.get("steering")), _clamp(message.get("throttle"))
        if steering is None or throttle is None:
            return None
        return Command("drive", id=command_id, args={"steering": steering, "throttle": throttle})
    if kind == "path":
        path = _PATHS.get(message.get("path"))
        if path is None:
            return None
        return Command("path", id=command_id, args={"path": path})
    if kind == "mission":
        pickup, delivery = message.get("pickup"), message.get("delivery")
        if pickup not in LETTERS + (None,) or delivery not in LETTERS + (None,):
            return None
        return Command("mission", id=command_id, args={"pickup": pickup, "delivery": delivery})
    if kind == "dock":
        # Only "wait": the release is given at the dock (the lid), or by the
        # tuner or the PS5, never from the dashboard.
        action = message.get("action")
        if action != "wait":
            return None
        return Command("dock", id=command_id, args={"action": action})
    if kind == "open_cargo":
        return Command("open_cargo", id=command_id)
    if kind == "lights":
        return _parse_lights(message, command_id)
    if kind == "speed":
        args = {}
        for key, name in _SPEEDS.items():
            if key in message:
                value = _clamp(message[key], low=0.0)
                if value is None:
                    return None
                args[name] = value
        return Command("speed", id=command_id, args=args) if args else None
    if kind == "pid":
        args = {}
        for gain, high in PID_MAX.items():
            if gain in message:
                value = _clamp(message[gain], low=0.0)
                if value is None:
                    return None
                args[gain] = min(value, high)
        return Command("pid", id=command_id, args=args) if args else None
    if kind == "claim":
        return Command("claim", id=command_id, args={"force": message.get("force") is True})
    if kind == "release":
        return Command("release", id=command_id)
    if kind == "ping":
        t = message.get("t")
        return Command("ping", id=command_id, args={"t": t if isinstance(t, (int, float)) else None})
    return None


def _parse_lights(message: dict, command_id: Optional[int]) -> Optional[Command]:
    """Any subset of the operator's light settings, keyed by the
    LightsController attribute each one sets."""
    args: dict[str, Any] = {}
    if "turn_signal" in message:
        if message["turn_signal"] not in _SIGNALS:
            return None
        args["turn_signal"] = _SIGNALS[message["turn_signal"]]
    for name in _LIGHT_SWITCHES:
        if name in message:
            if not isinstance(message[name], bool):
                return None
            args[name] = message[name]
    if "strip" in message:
        if message["strip"] not in _STRIPS:
            return None
        args["strip_effect"] = _STRIPS[message["strip"]]
    if not args:
        return None
    return Command("lights", id=command_id, args=args)


# Commands any viewer may send; everything else needs the control claim.
OPEN_COMMANDS = ("claim", "release", "ping")
