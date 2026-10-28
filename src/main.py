import atexit
import cv2 as cv
import logging
import signal
import time

from arduino import Arduino

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("main")

from dataclasses import replace

from cargo import ManualOpen
from challenge import Challenge
from config import SPEED_TEST, Config
from frame_source import FrameSource
from lights import LightsController, TurnSignal
from obstacle import ObstacleGuard
from perception.line_tracking import LineTracker
from input.autonomous import Autonomous
from input.ps5 import BUTTON_NAMES, PS5Controller
from input.sign_policy import SignConstraint, SignPolicy
from input.mission import Mission, MissionCommand
from path import HOME_PATH, DetourReturn, ForkWatch, Path, anchor_for, profile_for
from perception.sign_worker import SignWorker
from gui import TunerGUI
from status import stop_cause
from voice import Voice


SIGN_CAM_INDICES = (0, 1, 2, 3)
SIGN_MODEL_PATH = "models/signs.onnx"
SIGN_CONFIDENCE = 0.5
CAMERA_INDEX = 0
DASHBOARD_PORT = 80


def draw_steering_bar(frame, steering):
    h, w = frame.shape[:2]
    bar_y = h - 30
    bar_w = w - 40
    bar_cx = w // 2
    cv.line(frame, (20, bar_y), (20 + bar_w, bar_y), (100, 100, 100), 2)
    cv.line(frame, (bar_cx, bar_y - 8), (bar_cx, bar_y + 8), (255, 255, 255), 1)
    indicator_x = int(bar_cx + steering * (bar_w // 2))
    indicator_x = max(20, min(20 + bar_w, indicator_x))
    color = (0, 0, 255) if abs(steering) > 0.5 else (0, 255, 255) if abs(steering) > 0.2 else (0, 255, 0)
    cv.circle(frame, (indicator_x, bar_y), 6, color, -1)


def draw_throttle_bar(frame, throttle):
    h, w = frame.shape[:2]
    bar_h = h - 80
    bar_y0 = 40
    bar_y1 = bar_y0 + bar_h
    bar_cy = (bar_y0 + bar_y1) // 2
    bar_x = w - 30
    t = max(-1.0, min(1.0, throttle))
    cv.rectangle(frame, (bar_x - 6, bar_y0), (bar_x + 6, bar_y1), (100, 100, 100), 1)
    cv.line(frame, (bar_x - 6, bar_cy), (bar_x + 6, bar_cy), (180, 180, 180), 1)
    mag = abs(t)
    fill_h = int((bar_h // 2) * mag)
    if fill_h > 0:
        if t >= 0:
            color = (0, int(255 * (1.0 - mag)), int(255 * mag))
            cv.rectangle(frame, (bar_x - 5, bar_cy - fill_h), (bar_x + 5, bar_cy), color, -1)
        else:
            color = (int(255 * mag), int(160 * mag), 0)
            cv.rectangle(frame, (bar_x - 5, bar_cy), (bar_x + 5, bar_cy + fill_h), color, -1)


cap = cv.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

ret, first_frame = cap.read()
if not ret:
    print("Error: Could not read first frame.")
    exit()


def _exit_on_signal(signum, _frame):
    raise SystemExit(128 + signum)


for _signum in (signal.SIGTERM, signal.SIGHUP):
    signal.signal(_signum, _exit_on_signal)

config = Config()
ratio = config.downscale_ratio
frame_w = round(first_frame.shape[1] / ratio)
frame_h = round(first_frame.shape[0] / ratio)
source = FrameSource(cap, live=True)
atexit.register(source.close)

gui = TunerGUI(config, frame_w, frame_h)
arduino = Arduino()
tracker = LineTracker()
autonomous = Autonomous()
detour_return = DetourReturn()
fork_watch = ForkWatch()
ps5 = PS5Controller()
lights = LightsController()
obstacle_guard = ObstacleGuard()
sign_cam_indices = tuple(i for i in SIGN_CAM_INDICES if i != CAMERA_INDEX)
sign_worker = SignWorker(sign_cam_indices, SIGN_MODEL_PATH, SIGN_CONFIDENCE)
sign_policy = SignPolicy()
mission = Mission()
voice = Voice()
manual_open = ManualOpen()
sign_worker.start()

hub = None
dashboard = None
try:
    from dashboard.hub import EventLogHandler, Hub
    from dashboard.server import DashboardServer
    from dashboard.contract import build_state
except ImportError as error:
    logger.warning(f"Dashboard disabled: {error} (pip install aiohttp)")
else:
    hub = Hub()
    logging.getLogger().addHandler(EventLogHandler(hub))
    dashboard = DashboardServer(hub, port=DASHBOARD_PORT)
    if not dashboard.start():
        hub = None
        dashboard = None

gui.set_pid_reset_callback(autonomous.reset)
gui.set_master_port_callback(arduino.set_master_port)
gui.set_lights_controller(lights)

challenge = Challenge.DELIVERY
if challenge is Challenge.SPEED:
    sign_worker.set_paused(True)
tunings = {stage: {name: getattr(stage_config, name) for name in ("kp", "ki", "kd", "base_speed", "min_speed")}
           for stage, stage_config in ((Challenge.DELIVERY, config), (Challenge.SPEED, SPEED_TEST))}
tuning = tunings[challenge]

fps = 0.0
prev_time = time.monotonic()
prev_stopped = False
last_sonar = None
held_steering = None
prev_hold = False
prev_stop_reason = None
path = HOME_PATH
prev_path = path
prev_lane_state = None
warned_calibrating = False
fork_passed = False

estopped = True
manual = False


def set_estop(engaged):
    global estopped
    if engaged != estopped:
        estopped = engaged
        logger.info("E-stop engaged" if engaged else "E-stop released")


def set_manual(on):
    global manual
    if on != manual:
        manual = on
        logger.info("Mode: manual" if on else "Mode: autonomous")


def set_challenge(requested):
    global challenge, tuning, path, detour_return, fork_watch, fork_passed
    if requested is challenge:
        return None
    if not estopped:
        return "the stage can only be changed while e-stopped"
    challenge = requested
    tuning = tunings[challenge]
    path = HOME_PATH
    detour_return = DetourReturn()
    fork_watch = ForkWatch()
    fork_passed = False
    sign_worker.set_paused(challenge is Challenge.SPEED)
    logger.info(f"Stage: {challenge.value}")
    return None


def letters_refusal(letters):
    if estopped or (mission.parked and letters != (None, None)):
        return None
    return "pickup/delivery can only be changed while e-stopped or parked at a delivery"


def dock_request(request, now):
    if estopped:
        return "e-stopped, resume first"
    if challenge is Challenge.SPEED:
        return "no docks in the speed test"
    return mission.release(now) if request == "release" else mission.hold_departure()


def open_request(now):
    if not estopped:
        return "e-stop first"
    manual_open.request(now)
    return None


def apply_dashboard_commands(now):
    requested_path = None
    if hub is None:
        return requested_path
    for command in hub.drain_commands():
        reason = None
        if command.type == "estop":
            set_estop(command.args["engaged"])
        elif command.type == "mode":
            set_manual(command.args["manual"])
        elif command.type == "challenge":
            reason = set_challenge(command.args["challenge"])
        elif command.type == "path":
            if challenge is Challenge.SPEED:
                reason = "the speed test drives Straight only"
            else:
                requested_path = command.args["path"]
        elif command.type == "lights":
            for name, value in command.args.items():
                setattr(lights, name, value)
        elif command.type in ("speed", "pid"):
            tuning.update(command.args)
        elif command.type == "mission":
            letters = (command.args["pickup"], command.args["delivery"])
            reason = letters_refusal(letters)
            if reason is None:
                mission.configure(*letters)
        elif command.type == "dock":
            reason = dock_request(command.args["action"], now)
        elif command.type == "open_cargo":
            reason = open_request(now)
        if reason:
            logger.warning(f"Dashboard {command.type} refused: {reason}")
        hub.reply(command.client, {"type": "nack" if reason else "ack", "id": command.id,
                                   "cmd": command.type, "reason": reason or ""})
    return requested_path


def publish(perc, steering, throttle, stop, constraint, mission_command, lights_output, applied, cargo):
    if hub is None:
        return
    state = build_state(
        now=now, fps=fps, estopped=estopped, manual=manual,
        steering=steering, throttle=throttle, stop=stop,
        path=path, detour_stage=detour_return.stage, carrying=detour_return.profile_for(path) is not profile_for(path),
        profile_label=autonomous.profile_label, holding=autonomous.holding,
        perc=perc, roi=None if perc is None else (params.roi_x0, params.roi_x1),
        snapshot=snapshot, constraint=constraint, mission=mission, mission_command=mission_command,
        applied=applied, guard=obstacle_guard, lights_output=lights_output, lights_signal=lights.signal_override,
        telemetry=arduino.telemetry, serial_open=arduino.is_open, battery=arduino.battery,
        lights_controller=lights, tuning=tuning, cargo=cargo, challenge=challenge,
    )
    hub.publish(state, None if perc is None else perc.display_frame, snapshot.frame)


while gui.is_running():
    now = time.monotonic()
    dt = now - prev_time
    prev_time = now

    if dt > 0:
        fps = 0.9 * fps + 0.1 / dt

    for index in ps5.poll_button_presses():
        logger.info(f"PS5 button {index} ({BUTTON_NAMES.get(index, '?')})")

    for toggled in (ps5.poll_estop_toggle(), gui.poll_estop_toggle()):
        if toggled:
            set_estop(not estopped)
    for toggled in (ps5.poll_mode_toggle(), gui.poll_mode_toggle()):
        if toggled:
            set_manual(not manual)

    tuning.update(gui.poll_tuning_request())

    requested_challenge = gui.poll_challenge_request()
    if requested_challenge is not None:
        refused = set_challenge(requested_challenge)
        if refused:
            logger.warning(f"Stage change refused: {refused}")
    dashboard_path = apply_dashboard_commands(now)
    speed_test = challenge is Challenge.SPEED

    for requested_path in (ps5.poll_path_buttons(), gui.poll_path_request(), dashboard_path):
        if requested_path is not None and not speed_test:
            path = requested_path

    letters = gui.poll_mission_request()
    if letters is not None:
        refused = letters_refusal(letters)
        if refused:
            logger.warning(f"Pickup/delivery refused: {refused}")
        else:
            mission.configure(*letters)

    for request in (gui.poll_dock_request(), "release" if ps5.poll_release() else None):
        if request is None:
            continue
        refused = dock_request(request, now)
        if refused:
            logger.warning(f"Dock {request} refused: {refused}")
    voice.update()

    sonar = arduino.read_sonar()
    if sonar is not None:
        last_sonar = sonar
        obstacle_guard.observe(sonar)
    obstacle_guard.update(dt)
    cargo = arduino.read_cargo(now)
    if gui.poll_open_request():
        refused = open_request(now)
        if refused:
            logger.warning(f"Open compartment refused: {refused}")
    open_by_hand = manual_open.update(now, cargo, stopped=estopped)

    gui.update_drive(estopped, manual)
    gui.update_lights(lights)
    gui.update_tuning(tuning)

    if estopped:
        if not prev_stopped:
            arduino.send_estop()
        prev_stopped = True
        if cargo is not None:
            arduino.send_cargo((mission.unlock and not speed_test) or open_by_hand)
        gui.update_sonar(last_sonar, obstacle_guard)
        gui.update_cargo(cargo)
        autonomous.reset()
        sign_policy.reset()
        mission.reset()
        held_steering = None
        lights.signal_override = TurnSignal.RIGHT if path is Path.DETOUR else None
        lights.update(0.0, estopped=True, autonomous=not manual, speed_test=speed_test,
                      scene=None if speed_test else mission.scene)
        lights_output = lights.compute()
        arduino.send_lights(lights_output)
        snapshot = sign_worker.get_snapshot()
        constraint = SignConstraint(reason="e-stop")
        mission_command = MissionCommand(reason="e-stop")
        gui.update_challenge(challenge)
        gui.update_signs(snapshot, constraint, applied=False)
        gui.update_mission(mission, mission_command, applied=False)
        gui.update_path_status(path, tracker.profile_ready, "e-stop")
        publish(None, 0.0, 0.0, ("estop", "e-stop"), constraint, mission_command, lights_output, False, cargo)
        gui.render_frame()
        continue
    prev_stopped = False

    ret, frame = source.read()
    if not ret:
        logger.warning("No camera frame for 0.5 s")
        continue

    params = replace(gui.read_params(), **tuning)

    snapshot = sign_worker.get_snapshot()
    if speed_test:
        mission_command = MissionCommand(reason="speed test")
    else:
        way_clear = not obstacle_guard.stopped and obstacle_guard.scale >= 1.0
        mission_command = mission.update(snapshot, path, now, fork_passed=fork_passed, clear=way_clear, cargo=cargo,
                                         quiet=not voice.speaking)
    if mission_command.path is not None:
        path = mission_command.path
    if not manual:
        voice.announce(mission_command.announce)

    guard_stopped = obstacle_guard.stopped and not speed_test
    guard_scale = 1.0 if speed_test else obstacle_guard.scale

    standing = not manual and (guard_stopped or prev_hold)
    perc = tracker.process(frame, params, anchor_for(path), learn=not standing)

    obstacle_stopped = not manual and guard_stopped
    returned = detour_return.update(path, perc, dt, standing=standing)
    fork_passed = not standing and fork_watch.update(path, perc, dt)
    if returned is not None:
        logger.info(f"Detour entrance passed, returning to {returned.value} (lane {perc.lane_state})")
        path = returned

    if path is not prev_path:
        autonomous.seed_derivative()
        logger.info(f"Path: {path.value} (lane {perc.lane_state})")
        prev_path = path
        warned_calibrating = False
    if path is not Path.NORMAL and not perc.profile_ready and not warned_calibrating:
        logger.warning(f"Path {path.value} selected while the tracker is still {perc.lane_state}; steering on the two-edge scan until it has a lane width")
        warned_calibrating = True

    if not manual:
        steering, throttle = autonomous.step(
            perc.lane_offset, perc.lane_curvature, params, dt, guard_scale,
            profile=detour_return.profile_for(path), provisional=perc.provisional, rows_found=perc.rows_found,
            far_offset=perc.far_offset, far_curvature=perc.far_curvature,
        )
        detour_return.observe_hold(autonomous.holding, dt, standing=standing)
    else:
        autonomous.reset()
        steering, throttle = ps5.poll()
        throttle *= params.base_speed

    if autonomous.stop_reason != prev_stop_reason:
        logger.info(f"Stop reason: {autonomous.stop_reason or 'none'} (lane {perc.lane_state})")
        prev_stop_reason = autonomous.stop_reason

    constraint = SignConstraint(reason="speed test") if speed_test else sign_policy.update(snapshot, now)
    scale = min(constraint.scale, mission_command.scale)
    hold = constraint.hold or mission_command.hold
    prev_hold = hold
    if not manual:
        if hold or throttle <= 0.0:
            throttle *= scale
        elif scale < 1.0:
            throttle = max(throttle * scale, autonomous.crawl_speed(params))
        if hold:
            if held_steering is None:
                held_steering = steering
            steering = held_steering
        else:
            held_steering = None

    draw_steering_bar(perc.display_frame, steering)
    draw_throttle_bar(perc.display_frame, throttle)

    if not manual and mission_command.turn_signal is not None:
        lights.signal_override = mission_command.turn_signal
    else:
        lights.signal_override = TurnSignal.RIGHT if path is Path.DETOUR else None

    sign_stopped = not manual and constraint.hold
    lights.update(throttle, estopped=False, autonomous=not manual,
                  obstacle_stopped=obstacle_stopped, sign_stopped=sign_stopped, speed_test=speed_test,
                  scene=None if manual else mission_command.scene)
    lights_output = lights.compute()
    arduino.send_lights(lights_output)
    arduino.send_motor(throttle, steering)
    if cargo is not None:
        arduino.send_cargo(mission.unlock and not manual and not speed_test)

    curvature = perc.lane_curvature if perc.lane_curvature is not None else 0.0
    error = (perc.lane_offset or 0) + 60.0 * curvature

    gui.update_textures(perc)
    lane_text = perc.lane_state if perc.width_ratio is None else f"{perc.lane_state} w{perc.width_ratio:.2f}"
    if detour_return.stage:
        lane_text = f"{lane_text} | {detour_return.stage}"
    stop = None if manual else stop_cause(autonomous.stop_reason, autonomous.holding, hold, constraint, mission_command)
    gui.update_telemetry(fps, int(round(error)), steering, throttle, lane_text, None if stop is None else stop[1])
    gui.update_path_status(path, perc.profile_ready, perc.lane_state)
    gui.update_sonar(last_sonar, obstacle_guard)
    applied = not manual and not speed_test
    why = "speed test" if speed_test else "manual"
    gui.update_challenge(challenge)
    gui.update_cargo(cargo)
    gui.update_signs(snapshot, constraint, applied=applied, why=why)
    gui.update_mission(mission, mission_command, applied=applied, why=why)
    publish(perc, steering, throttle, stop, constraint, mission_command, lights_output, applied, cargo)
    gui.render_frame()

source.close()
if dashboard is not None:
    dashboard.stop()
sign_worker.close()
voice.close()
ps5.shutdown()
arduino.close()
gui.shutdown()
