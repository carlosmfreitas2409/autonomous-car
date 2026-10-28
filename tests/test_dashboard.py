"""The dashboard contract, hub and server, without a camera or a car."""

import asyncio
import json
import math

import numpy as np
import pytest

from arduino import parse_battery, parse_telemetry
from cargo import CargoReading
from challenge import Challenge
from config import SPEED_TEST, Config
from dashboard.contract import (PID_MAX, build_state, layout, light_settings, next_decision, parse_command,
                                route_progress, segment)
from dashboard.hub import Hub
from input.mission import Dock, Mission, MissionCommand, MissionState, Stage
from input.sign_policy import SignConstraint
from lights import LightsController, StripEffect, TurnSignal
from obstacle import ObstacleGuard
from path import Path
from perception.line_tracking import LaneAnchor, LineTrackingResult
from perception.sign_tracker import Track, bearing_deg
from perception.sign_worker import SignSnapshot
from perception.traffic_light import LightState
from status import stop_cause, stop_text


def _track(**overrides):
    fields = dict(id=7, label="detour", bbox=(600, 100, 700, 340), confidence=0.7,
                  distance_m=2.345, confirmed=True, light_state=LightState.UNKNOWN,
                  light_reason="", rejected=False, variant=None, variant_reason="",
                  first_seen=0.0, last_seen=1.0, bearing_deg=-3.21)
    fields.update(overrides)
    return Track(**fields)


def _perc():
    return LineTrackingResult(
        display_frame=np.zeros((240, 320, 3), np.uint8), debug_frame=np.zeros((240, 320, 3), np.uint8),
        lane_offset=-12, lane_curvature=0.0312, lane_state="both 3/5", anchor=LaneAnchor.LEFT,
        rows_found=3, both_rows=2, profile_ready=True, row_centers=((170, 150, True), (200, 155, False)),
    )


def _state(estopped=True, manual=False, perc=None, mission=None, telemetry=None, battery=None, lights_controller=None,
           tuning=None, mission_command=None, cargo=None, stop=None, challenge=Challenge.DELIVERY):
    lights = LightsController()
    lights.update(0.4, estopped=False, autonomous=True)
    return build_state(
        now=50.0, fps=23.4, estopped=estopped, manual=manual,
        steering=0.12345, throttle=0.5, stop=stop,
        path=Path.STRAIGHT, detour_stage="", carrying=False, profile_label="normal", holding=0.0,
        perc=perc, roi=(0.03, 0.98),
        snapshot=SignSnapshot(tracks=[_track()], timestamp=49.9, camera_open=True, detector_available=True, fps=9.8),
        constraint=SignConstraint(), mission=mission or Mission(), mission_command=mission_command or MissionCommand(),
        applied=True, guard=ObstacleGuard(), lights_output=lights.compute(), lights_signal=None,
        telemetry=telemetry, serial_open=False, battery=battery, lights_controller=lights_controller,
        tuning=tuning, cargo=cargo, challenge=challenge,
    )


# --- contract --------------------------------------------------------------

def test_state_is_json_and_carries_every_slice():
    state = _state(perc=_perc())
    text = json.dumps(state)
    assert json.loads(text)["version"] == 3
    for key in ("drive", "path", "lane", "signs", "policy", "mission", "obstacle", "lights", "health", "cargo"):
        assert key in state
    assert state["lane"]["rows"] == [[170, 150, True], [200, 155, False]]
    assert state["lane"]["frame"] == [320, 240]
    assert state["lane"]["roi_center_x"] == round(0.505 * 320)
    assert state["signs"]["tracks"][0] == {
        "id": 7, "label": "detour", "distance_m": 2.35, "bearing_deg": -3.2, "confirmed": True,
        "rejected": False, "variant": None, "light": "unknown", "reason": "",
    }


def test_lane_carries_the_band_and_width_profile_for_drawing():
    perc = _perc()
    perc.band_edges = ((90, 40, 280), (102, 30, 290))
    perc.width_profile = (1.2345678, -12.345)
    lane = _state(perc=perc)["lane"]
    assert lane["band"] == [[90, 40, 280], [102, 30, 290]]
    assert lane["width_profile"] == [1.23457, -12.35]
    plain = _state(perc=_perc())["lane"]
    assert plain["band"] == [] and plain["width_profile"] is None


def test_estopped_state_has_no_lane_and_says_why():
    state = _state()
    assert state["lane"] is None
    assert state["drive"]["estopped"] and state["drive"]["stop_text"] == "e-stop"
    assert state["drive"]["stop_source"] == "estop"


def test_stop_names_its_source_once_driving():
    drive = _state(estopped=False, stop=("mission", "loading at A, waiting for release"))["drive"]
    assert drive["stop_source"] == "mission" and drive["stop_text"] == "loading at A, waiting for release"
    drive = _state(estopped=False)["drive"]
    assert drive["stop_source"] is None and drive["stop_text"] is None


def test_no_cargo_node_means_no_cargo_and_no_dock():
    mission = _state()["mission"]
    assert _state()["cargo"] is None
    assert mission["dock"] is None and mission["check"] is None
    assert mission["parked_at"] is None and not mission["unlock"] and mission["manifest_g"] is None


def test_dock_and_cargo_are_reflected_as_the_mission_holds_them():
    m = Mission()
    m.configure("C", "A")
    m.dock = Dock("C", 3, Stage.PICKUP, 42.5)
    m.dock.check_failed = "empty"
    reading = CargoReading(grams=12, stable=False, lid_closed=True, unlocked=True, fault=False)
    state = _state(mission=m, cargo=reading,
                   mission_command=MissionCommand(state=MissionState.DOCKED, reason="loading at C"))
    json.dumps(state)
    assert state["mission"]["dock"] == {
        "letter": "C", "role": "pickup", "since": 42.5, "released": False, "lid_opened": False,
        "check_failed": "empty", "override": False,
    }
    assert state["mission"]["unlock"] and state["mission"]["state"] == "docked"
    assert state["cargo"] == {"grams": 12, "stable": False, "lid_closed": True, "unlocked": True, "fault": False}

    assert m.release(43.0) is None
    state = _state(mission=m, mission_command=MissionCommand(state=MissionState.CHECKING, check="weighing"))
    assert state["mission"]["dock"]["released"] and state["mission"]["dock"]["override"]
    assert state["mission"]["check"] == "weighing" and not state["mission"]["unlock"]


def test_a_dock_left_from_where_it_parked_has_no_role():
    m = Mission()
    m._spot = ("C", 4)
    m.configure("A", "B")
    m.update(SignSnapshot(tracks=[], timestamp=1.0, camera_open=True, detector_available=True), Path.STRAIGHT, 1.0)
    mission = _state(mission=m)["mission"]
    assert mission["dock"]["letter"] == "C" and mission["dock"]["role"] is None
    assert mission["parked_at"] == "C"


def test_no_encoder_means_no_speed_but_the_applied_output():
    telemetry = parse_telemetry("T 512 500 -128 30 R 12", 49.5)
    state = _state(telemetry=telemetry)
    assert state["drive"]["speed_est_mps"] is None
    assert state["drive"]["applied_throttle"] == pytest.approx(-128 / 255, abs=1e-3)
    assert state["drive"]["fw"]["state"] == "R" and state["drive"]["fw"]["age_s"] == 0.5


def test_mode_is_reported():
    assert _state(manual=True)["drive"]["mode"] == "manual"
    assert _state()["drive"]["mode"] == "autonomous"


def test_layout_is_the_missions_own_table():
    from input.mission import LETTER_EXIT
    assert layout()["letters"] == LETTER_EXIT
    assert layout()["exits"] == [1, 2]


def test_segment_next_decision_and_progress_follow_the_mission():
    m = Mission()
    assert segment(m, Path.STRAIGHT, False) is None
    m.configure("A", "C")
    assert segment(m, Path.STRAIGHT, False) == {"kind": "main", "towards": 1}
    assert next_decision(m) == {"exit": 1, "take": "detour", "target": "A"}
    assert route_progress(m) == 0
    # The exit-1 sign was decided: the car is entering detour 1, which comes
    # back before exit 1.
    m.next_exit, m.decisions, m.last_decision = 1, 1, (7, 1, Path.DETOUR)
    assert segment(m, Path.DETOUR, False) == {"kind": "detour", "exit": 1}
    assert segment(m, Path.STRAIGHT, True) == {"kind": "detour", "exit": 1}
    assert segment(m, Path.STRAIGHT, False) == {"kind": "main", "towards": 1}
    assert route_progress(m) == 1  # "A: pickup"


def test_decided_names_the_sign_the_mission_just_decided():
    m = Mission()
    m.configure("A", "C")
    assert _state(mission=m)["mission"]["decided"] is None
    sign = SignSnapshot(tracks=[_track(id=7, distance_m=2.5)], timestamp=50.0, camera_open=True,
                        detector_available=True, fps=9.8)
    assert m.update(sign, Path.STRAIGHT, 50.0).path is Path.DETOUR
    mission = _state(mission=m)["mission"]
    # `next` has moved on (detour 1 comes back before exit 1); `decided` still says what sign 7 meant.
    assert mission["decided"] == {"sign": 7, "exit": 1, "take": "detour"}
    assert mission["next"]["exit"] == 1


def test_lane_band_marks_a_side_without_a_line():
    perc = _perc()
    perc.band_edges = ((90, 40, 280), (102, 30, -1))
    assert _state(perc=perc)["lane"]["band"] == [[90, 40, 280], [102, 30, -1]]


def test_stop_text_priority_matches_the_old_inline_logic():
    hold = SignConstraint(scale=0.0, hold=True, reason="stop sign")
    mission = MissionCommand(scale=0.0, hold=True, state=MissionState.DOCKED, reason="pickup at A")
    assert stop_text("obstacle", 1.0, True, hold, mission) == "obstacle"
    assert stop_text(None, 0.0, True, hold, mission) == "stop sign"
    assert stop_text(None, 0.0, True, SignConstraint(), mission) == "pickup at A"
    assert stop_text(None, 0.42, False, SignConstraint(), MissionCommand()) == "holding 0.4s"
    assert stop_text(None, 0.0, False, SignConstraint(), MissionCommand()) is None
    assert stop_cause("obstacle", 1.0, True, hold, mission) == ("controller", "obstacle")
    assert stop_cause(None, 0.0, True, hold, mission) == ("sign", "stop sign")
    assert stop_cause(None, 0.0, True, SignConstraint(), mission) == ("mission", "pickup at A")
    assert stop_cause(None, 0.42, False, SignConstraint(), MissionCommand()) == ("holding", "holding 0.4s")


@pytest.mark.parametrize("message", [
    None, [], {}, {"type": "nope"}, {"type": "estop"}, {"type": "estop", "engaged": 1},
    {"type": "mode", "mode": "turbo"}, {"type": "drive", "steering": 0, "throttle": 0}, {"type": "claim"},
    {"type": "path", "path": "left"}, {"type": "mission", "pickup": "D", "delivery": "A"},
    {"type": "lights"}, {"type": "lights", "drl": 1}, {"type": "lights", "turn_signal": "up"},
    {"type": "lights", "strip": "disco"}, {"type": "lights", "low_beam": True, "strip": "disco"},
    {"type": "speed"}, {"type": "speed", "base": "fast"}, {"type": "speed", "min": True},
    {"type": "speed", "base": 0.5, "min": float("nan")},
    {"type": "pid"}, {"type": "pid", "kp": "x"}, {"type": "pid", "kd": None},
    {"type": "dock"}, {"type": "dock", "action": "leave"}, {"type": "dock", "action": "release"},
    {"type": "challenge"}, {"type": "challenge", "challenge": "drift"},
])
def test_malformed_commands_are_rejected(message):
    assert parse_command(message) is None


def test_commands_parse_and_clamp():
    assert parse_command({"type": "estop", "engaged": True}).args == {"engaged": True}
    mode = parse_command({"type": "mode", "mode": "manual", "id": 4})
    assert mode.args == {"manual": True} and mode.id == 4
    assert parse_command({"type": "mode", "mode": "autonomous"}).args == {"manual": False}
    assert parse_command({"type": "path", "path": "detour"}).args["path"] is Path.DETOUR
    assert parse_command({"type": "mission", "pickup": "A", "delivery": None}).args == {"pickup": "A", "delivery": None}
    assert parse_command({"type": "dock", "action": "wait", "id": 9}).args == {"action": "wait"}
    assert parse_command({"type": "open_cargo"}).type == "open_cargo"
    assert parse_command({"type": "challenge", "challenge": "speed"}).args == {"challenge": Challenge.SPEED}


def test_lights_command_names_the_controller_attributes_it_sets():
    command = parse_command({"type": "lights", "turn_signal": "hazard", "drl": True, "strip": "police"})
    assert command.args == {"turn_signal": TurnSignal.HAZARD, "drl": True, "strip_effect": StripEffect.POLICE}
    assert parse_command({"type": "lights", "strip": "auto"}).args == {"strip_effect": None}

    lights = LightsController()
    for name, value in command.args.items():
        setattr(lights, name, value)
    assert light_settings(lights) == {
        "turn_signal": "hazard", "drl": True, "low_beam": False, "logo": False, "projector": False, "strip": "police",
    }


def test_speed_command_is_keyed_like_config_and_clamped_to_a_fraction():
    assert parse_command({"type": "speed", "base": 0.55}).args == {"base_speed": 0.55}
    assert parse_command({"type": "speed", "base": 1.4, "min": -0.2}).args == {"base_speed": 1.0, "min_speed": 0.0}


def test_pid_command_is_keyed_like_config_and_clamped_to_the_tuners_range():
    assert parse_command({"type": "pid", "kp": 0.025}).args == {"kp": 0.025}
    assert parse_command({"type": "pid", "ki": 0.9, "kd": -1}).args == {"ki": 0.05, "kd": 0.0}


def test_state_carries_the_loops_tuning_and_its_defaults():
    tuning = {"kp": 0.03, "ki": 0.001, "kd": 0.0027, "base_speed": 0.4, "min_speed": 0.3}
    state = _state(tuning=tuning)
    defaults = Config()
    assert state["speed"] == {"base": 0.4, "min": 0.3,
                              "defaults": {"base": defaults.base_speed, "min": defaults.min_speed}}
    assert state["pid"] == {"kp": 0.03, "ki": 0.001, "kd": 0.0027,
                            "defaults": {"kp": defaults.kp, "ki": defaults.ki, "kd": defaults.kd}, "max": PID_MAX}
    assert _state()["speed"] is None and _state()["pid"] is None


def test_state_names_the_stage_and_its_own_defaults():
    tuning = {"kp": 0.03, "ki": 0.001, "kd": 0.0027, "base_speed": 0.4, "min_speed": 0.3}
    assert _state(tuning=tuning)["challenge"] == "delivery"
    state = _state(tuning=tuning, challenge=Challenge.SPEED)
    assert state["challenge"] == "speed"
    assert state["speed"]["defaults"] == {"base": SPEED_TEST.base_speed, "min": SPEED_TEST.min_speed}
    assert state["pid"]["defaults"] == {"kp": SPEED_TEST.kp, "ki": SPEED_TEST.ki, "kd": SPEED_TEST.kd}


def test_state_carries_the_operators_light_settings_apart_from_the_lamps():
    lights = LightsController()
    lights.strip_effect = StripEffect.RAINBOW
    state = _state(lights_controller=lights)
    assert state["lights"]["settings"]["strip"] == "rainbow"
    assert state["lights"]["strip"] == "cyan"
    assert _state()["lights"]["settings"] is None


# --- firmware line, bearing --------------------------------------------------

def test_t_line_parses_seven_and_eight_fields():
    seven = parse_telemetry("T 512 500 120 -30 W 640", 1.0)
    assert (seven.throttle_pwm, seven.state, seven.ms_since_command, seven.fault) == (120, "W", 640, None)
    assert parse_telemetry("T 512 500 120 -30 R 10 D", 1.0).fault == "D"
    assert parse_telemetry("T 512 500 120", 1.0) is None
    assert parse_telemetry("T a b c d R 1", 1.0) is None
    assert parse_telemetry("S C 255 255 255 255", 1.0) is None


def test_b_line_parses_millivolts_and_percent():
    reading = parse_battery("B 12430 78", 1.0)
    assert (reading.volts, reading.percent) == (12.43, 78)
    assert parse_battery("B 12430 104", 1.0).percent == 100
    assert parse_battery("B 12430", 1.0) is None
    assert parse_battery("B x 78", 1.0) is None
    assert parse_battery("T 512 500 120 -30 W 640", 1.0) is None


def test_battery_is_reported_once_the_firmware_sends_it():
    assert _state()["battery"] is None
    state = _state(battery=parse_battery("B 12430 78", 48.0))
    assert state["battery"] == {"percent": 78, "volts": 12.43, "age_s": 2.0}


def test_bearing_is_zero_on_axis_and_half_fov_at_the_edge():
    hfov = math.radians(63.3)
    assert bearing_deg(640, 1280, hfov) == pytest.approx(0.0)
    assert bearing_deg(1280, 1280, hfov) == pytest.approx(31.65)
    assert bearing_deg(0, 1280, hfov) == pytest.approx(-31.65)


# --- hub -------------------------------------------------------------------

def test_hub_never_drops_an_estop():
    hub = Hub()
    for _ in range(64):
        assert hub.submit(parse_command({"type": "path", "path": "normal"}))
    assert hub.submit(parse_command({"type": "estop", "engaged": True}))
    commands = hub.drain_commands()
    assert commands[-1].type == "estop" and len(commands) == 64


# --- server ----------------------------------------------------------------

aiohttp = pytest.importorskip("aiohttp")


def test_server_streams_state_video_and_routes_commands(tmp_path):
    from dashboard.server import DashboardServer

    hub = Hub()
    server = DashboardServer(hub, host="127.0.0.1", port=0, dist=tmp_path)
    assert server.start()
    hub.publish(_state(perc=_perc()), np.full((240, 320, 3), 128, np.uint8), None)

    async def scenario():
        base = f"http://127.0.0.1:{server.port}"
        async with aiohttp.ClientSession() as session:
            async with session.get(base + "/") as response:
                assert response.status == 200 and "not built" in await response.text()

            viewer = await session.ws_connect(base + "/ws/control")
            hello = await viewer.receive_json(timeout=2)
            assert hello["type"] == "hello" and hello["layout"]["exits"] == [1, 2]
            state = await _next(viewer, "state")
            assert state["lane"]["state"] == "both 3/5"

            # Every console commands; the loop acks what it applies.
            other = await session.ws_connect(base + "/ws/control")
            await viewer.send_json({"type": "mode", "mode": "manual", "id": 1})
            await other.send_json({"type": "estop", "engaged": True, "id": 2})
            await viewer.send_json({"type": "drive", "steering": 0.2, "throttle": 0.0, "id": 3})
            nack = await _next(viewer, "nack")
            assert nack["id"] == 3 and nack["reason"] == "invalid command"
            await asyncio.sleep(0.1)
            assert sorted(c.type for c in hub.drain_commands()) == ["estop", "mode"]

            await viewer.send_json({"type": "ping", "t": 123})
            assert (await _next(viewer, "pong"))["t"] == 123

            video = await session.ws_connect(base + "/ws/video")
            await video.send_json({"source": "drive", "next": True})
            frame = await video.receive_bytes(timeout=3)
            assert frame[0] == 0 and frame[1:3] == b"\xff\xd8"

            await viewer.close()
            await other.close()
            await video.close()

    try:
        asyncio.run(scenario())
    finally:
        server.stop()


async def _next(ws, kind):
    for _ in range(50):
        message = await ws.receive_json(timeout=2)
        if message.get("type") == kind:
            return message
    raise AssertionError(f"no {kind}")
