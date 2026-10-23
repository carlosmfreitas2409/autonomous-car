import argparse
import atexit
import csv
import cv2 as cv
import logging
import signal
import threading
import time

from arduino import Arduino

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("main")

from dataclasses import replace

from cargo import ManualOpen
from challenge import Challenge
from config import SPEED_TEST, Config
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
from supervisor import Source, Supervisor
from voice import Voice


# Sign perception runs on its own thread with its own camera: the worker takes
# the first of these indices that opens, minus whichever one the drive camera
# already holds. No model at SIGN_MODEL_PATH means no sign detection and an
# unchanged car. Confidence is the value measured in sign-detection.md.
SIGN_CAM_INDICES = (0, 1, 2, 3)
SIGN_MODEL_PATH = "models/signs.onnx"
SIGN_CONFIDENCE = 0.5


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


class FrameSource:
    """The newest camera frame, read on its own thread.

    The camera delivers 30 fps and the control loop runs slower, so a frame
    read in the loop is the oldest one in the driver's buffer: several frames
    (100-170 ms) stale, on top of the loop period itself. On the straight-lap
    recording of 2026-10-01 the car weaved with a 2.5-3 s period and the
    wheel at full lock half the time, the signature of a bang-bang loop with
    too much delay. Reading on a thread keeps the loop fed with the latest
    frame and takes the read's own cost out of the loop. Video replay stays
    sequential so a recording replays frame for frame.
    """

    def __init__(self, cap, live: bool):
        self._cap = cap
        self._live = live
        self._lock = threading.Condition()
        self._frame = None
        self._seq = 0
        self._failed = False
        self._thread = None
        if live:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def _run(self):
        while True:
            ok, frame = self._cap.read()
            with self._lock:
                if ok:
                    self._frame = frame
                    self._seq += 1
                    self._failed = False
                else:
                    self._failed = True
                self._lock.notify_all()
            if not ok:
                time.sleep(0.01)

    def read(self, timeout: float = 0.5):
        """(ok, frame): the newest frame not yet handed out."""
        if not self._live:
            return self._cap.read()
        with self._lock:
            seen = getattr(self, "_handed", 0)
            deadline = time.monotonic() + timeout
            while self._seq == seen and not self._failed:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False, None
                self._lock.wait(remaining)
            if self._seq == seen:
                return False, None
            self._handed = self._seq
            return True, self._frame


_LOG_FIELDS = ("t", "frame", "path", "lane_state", "rows", "both_rows", "offset", "curvature", "error", "steering", "throttle", "holding", "profile", "stop_reason", "sonar", "sonar_lr", "obstacle", "mission_stage", "target", "next_exit", "mission_state", "fork", "kp", "kd", "ema", "roi", "base_speed", "min_speed", "far_offset", "far_curvature", "cargo", "challenge")


parser = argparse.ArgumentParser(description="Autonomous car tuner")
parser.add_argument("--video", help="replay a recorded video instead of the camera")
parser.add_argument("--record", help="write every camera frame to this file, for replaying later")
parser.add_argument("--flip", action="store_true", help="mirror frames left-right (for a clip recorded flipped)")
parser.add_argument("--camera", type=int, default=3, help="camera index when not replaying a video (default 2)")
parser.add_argument("--log", help="write per-frame telemetry to this CSV")
parser.add_argument("--no-voice", action="store_true", help="no spoken prompts at the docks")
parser.add_argument("--voice", help='dock prompt voice: a `say` voice name, or "system" for the Mac\'s system voice (e.g. a Siri voice); default Luciana')
parser.add_argument("--dashboard-port", type=int, default=8000, help="operator dashboard port (default 8000)")
parser.add_argument("--no-dashboard", action="store_true", help="do not start the operator dashboard server")
args = parser.parse_args()

cap = cv.VideoCapture(args.video if args.video else args.camera)
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

ret, first_frame = cap.read()
if not ret:
    print("Error: Could not read first frame.")
    exit()

recorder = None
if args.record:
    capture_fps = cap.get(cv.CAP_PROP_FPS)
    if not capture_fps or capture_fps != capture_fps or capture_fps <= 0:
        capture_fps = 30.0
    frame_size = (first_frame.shape[1], first_frame.shape[0])
    recorder = cv.VideoWriter(args.record, cv.VideoWriter_fourcc(*"mp4v"), capture_fps, frame_size)
    logger.info(f"Recording {frame_size[0]}x{frame_size[1]} at {capture_fps:.0f} fps to {args.record}")

log_file = None
log_writer = None
if args.log:
    # Line-buffered: a run that ends in a crash or a kill keeps its last rows.
    log_file = open(args.log, "w", newline="", buffering=1)
    log_writer = csv.writer(log_file)
    log_writer.writerow(_LOG_FIELDS)


def _close_recording():
    """Finish the video and the log before anything else shuts down.

    An mp4 is unreadable until `release()` writes its index; day11/day12
    (2026-10-06) lost theirs when the run ended without reaching it. Runs at
    normal exit, on an exception, on Ctrl+C and on SIGTERM/SIGHUP.
    """
    global recorder, log_file
    if recorder is not None:
        recorder.release()
        recorder = None
    if log_file is not None:
        log_file.close()
        log_file = None


def _exit_on_signal(signum, _frame):
    raise SystemExit(128 + signum)


atexit.register(_close_recording)
for _signum in (signal.SIGTERM, signal.SIGHUP):
    signal.signal(_signum, _exit_on_signal)

config = Config()
ratio = config.downscale_ratio
frame_w = round(first_frame.shape[1] / ratio)
frame_h = round(first_frame.shape[0] / ratio)
cap.set(cv.CAP_PROP_POS_FRAMES, 0)
source = FrameSource(cap, live=not args.video)

gui = TunerGUI(config, frame_w, frame_h)
arduino = Arduino()
tracker = LineTracker()
autonomous = Autonomous()
detour_return = DetourReturn()
fork_watch = ForkWatch()
ps5 = PS5Controller()
lights = LightsController()
obstacle_guard = ObstacleGuard()
sign_cam_indices = tuple(i for i in SIGN_CAM_INDICES if args.video or i != args.camera)
sign_worker = SignWorker(sign_cam_indices, SIGN_MODEL_PATH, SIGN_CONFIDENCE)
sign_policy = SignPolicy()
mission = Mission()
voice = Voice(enabled=not args.no_voice, voice=args.voice)
manual_open = ManualOpen()
supervisor = Supervisor()
sign_worker.start()

# The operator dashboard: a server thread that only ever meets the loop in the
# hub. Without aiohttp, or with the port taken, the car runs exactly as before.
hub = None
dashboard = None
if not args.no_dashboard:
    try:
        from dashboard.hub import EventLogHandler, Hub
        from dashboard.server import DashboardServer
        from dashboard.contract import build_state
    except ImportError as error:
        logger.warning(f"Dashboard disabled: {error} (pip install aiohttp)")
    else:
        hub = Hub()
        logging.getLogger().addHandler(EventLogHandler(hub))
        dashboard = DashboardServer(hub, port=args.dashboard_port)
        if not dashboard.start():
            hub = None
            dashboard = None

gui.set_pid_reset_callback(autonomous.reset)
gui.set_master_port_callback(arduino.set_master_port)
gui.set_lights_controller(lights)

challenge = Challenge.DELIVERY
# The PID gains and speed limits belong to the loop, one set per stage: the
# tuner's sliders and the dashboard both ask for changes, and both show what
# is set here.
tunings = {stage: {name: getattr(stage_config, name) for name in ("kp", "ki", "kd", "base_speed", "min_speed")}
           for stage, stage_config in ((Challenge.DELIVERY, config), (Challenge.SPEED, SPEED_TEST))}
tuning = tunings[challenge]

fps = 0.0
start_time = time.monotonic()
prev_time = start_time
prev_stopped = False
last_sonar = None
held_steering = None
# The car was held still last frame by a sign or the mission (the tracker
# learns nothing from a car standing still).
prev_hold = False
prev_stop_reason = None
path = HOME_PATH
prev_path = path
prev_lane_state = None
frame_index = -1
warned_calibrating = False
# Set by ForkWatch on the frame it sees a detour's mouth go by; the mission
# reads it on the next frame (it decides before the tracker runs).
fork_passed = False


def letters_refusal(letters):
    """Why a new (pickup, delivery) is refused, or None. Parked after a
    delivery, the next order is given where it stands; clearing both letters
    forgets that dock, so that needs an e-stop."""
    if supervisor.estopped or (mission.parked and letters != (None, None)):
        return None
    return "pickup/delivery can only be changed while e-stopped or parked at a delivery"


def dock_request(request, now):
    """The person at the dock: loading or unloading is done ("release"), or
    not yet after all ("wait"), from any console. Returns why it was refused,
    or None."""
    if supervisor.estopped:
        return "e-stopped, resume first"
    if speed_test:
        return "no docks in the speed test"
    return mission.release(now) if request == "release" else mission.hold_departure()


def open_request(now):
    """The operator's "open the compartment", only while e-stopped. Returns
    why it was refused, or None."""
    if not supervisor.estopped:
        return "e-stop first"
    manual_open.request(now)
    return None


def apply_dashboard_commands(now):
    """Apply what the dashboard asked since the last pass, and ack it, through
    the same gates as the tuner and the pad. Returns the path it requested, if
    any, for the loop to treat exactly like the GUI's."""
    requested_path = None
    if hub is None:
        return requested_path
    for command in hub.drain_commands():
        reason = None
        if command.type == "estop":
            supervisor.request_estop(command.args["engaged"], Source.DASHBOARD)
        elif command.type == "mode":
            supervisor.request_mode(command.args["mode"], Source.DASHBOARD)
        elif command.type == "path":
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


def publish(perc, steering, throttle, stop, constraint, mission_command, lights_output, applied, manual_note, cargo):
    if hub is None:
        return
    state = build_state(
        now=now, fps=fps, supervisor=supervisor, manual_note=manual_note,
        steering=steering, throttle=throttle, stop=stop,
        path=path, detour_stage=detour_return.stage, carrying=detour_return.profile_for(path) is not profile_for(path),
        profile_label=autonomous.profile_label, holding=autonomous.holding,
        perc=perc, roi=None if perc is None else (params.roi_x0, params.roi_x1),
        snapshot=snapshot, constraint=constraint, mission=mission, mission_command=mission_command,
        applied=applied, guard=obstacle_guard, lights_output=lights_output, lights_signal=lights.signal_override,
        telemetry=arduino.telemetry, serial_open=arduino.is_open, battery=arduino.battery,
        lights_controller=lights, tuning=tuning, cargo=cargo,
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

    if ps5.poll_estop_toggle():
        supervisor.toggle_estop(Source.PAD)
    if gui.poll_estop_toggle():
        supervisor.toggle_estop(Source.GUI)

    if ps5.poll_mode_toggle():
        supervisor.toggle_mode(Source.PAD)
    if gui.poll_mode_toggle():
        supervisor.toggle_mode(Source.GUI)

    tuning.update(gui.poll_tuning_request())

    requested_challenge = gui.poll_challenge_request()
    if requested_challenge is not None and requested_challenge is not challenge:
        if supervisor.estopped:
            challenge = requested_challenge
            tuning = tunings[challenge]
            path = HOME_PATH
            detour_return = DetourReturn()
            fork_watch = ForkWatch()
            fork_passed = False
            sign_worker.set_paused(challenge is Challenge.SPEED)
            logger.info(f"Stage: {challenge.value}")
        else:
            logger.warning("The stage can only be changed while e-stopped")
    speed_test = challenge is Challenge.SPEED
    dashboard_path = apply_dashboard_commands(now)

    # Path buttons are read before the stopped branch so the Detour can be
    # armed while the car waits at the sign.
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

    # The person at the dock: loading or unloading is done ("release"), or
    # not yet after all ("wait"). Triangle's e-stop is the PS5's wait.
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
    # The cargo compartment, None with no cargo node on the bus (and then no
    # `K` line is ever sent).
    cargo = arduino.read_cargo(now)
    if gui.poll_open_request():
        refused = open_request(now)
        if refused:
            logger.warning(f"Open compartment refused: {refused}")
    open_by_hand = manual_open.update(now, cargo, stopped=supervisor.estopped)

    gui.update_supervisor(supervisor)
    gui.update_lights(lights)
    gui.update_tuning(tuning)

    if supervisor.estopped:
        if not prev_stopped:
            arduino.send_estop()
        prev_stopped = True
        if cargo is not None:
            # E-stopped the car cannot move: the dock's compartment stays
            # open, and the operator may open it by hand.
            arduino.send_cargo((mission.unlock and not speed_test) or open_by_hand)
        gui.update_sonar(last_sonar, obstacle_guard)
        gui.update_cargo(cargo)
        autonomous.reset()
        sign_policy.reset()
        mission.reset()
        held_steering = None
        lights.signal_override = TurnSignal.RIGHT if path is Path.DETOUR else None
        lights.update(0.0, estopped=True, autonomous=supervisor.autonomous, speed_test=speed_test,
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
        publish(None, 0.0, 0.0, ("estop", "e-stop"), constraint, mission_command, lights_output, False, None, cargo)
        gui.render_frame()
        continue
    prev_stopped = False

    ret, frame = source.read()
    if not ret:
        if args.video:
            logger.warning("End of video, rewinding")
            cap.set(cv.CAP_PROP_POS_FRAMES, 0)
            tracker.reset()
            autonomous.reset()
        else:
            logger.warning("No camera frame for 0.5 s")
        continue

    if args.flip:
        frame = cv.flip(frame, 1)

    # The recording is written frame for frame but tagged with the camera's
    # fps, while the loop runs slower; the frame index is what lines the CSV
    # up with the video, not `t`.
    frame_index += 1
    if recorder is not None:
        recorder.write(frame)

    params = replace(gui.read_params(), **tuning)

    # One lock, then pure arithmetic: the control loop never waits on the
    # sign camera or the model. The mission decides before the tracker runs,
    # so a detour sign's decision anchors this very frame.
    snapshot = sign_worker.get_snapshot()
    if speed_test:
        mission_command = MissionCommand(reason="speed test")
    else:
        # A released car pulls away only once the sonar sees nothing ahead.
        way_clear = not obstacle_guard.stopped and obstacle_guard.scale >= 1.0
        # A released car also waits for its "leaving" prompt to finish.
        mission_command = mission.update(snapshot, path, now, fork_passed=fork_passed, clear=way_clear, cargo=cargo,
                                         quiet=not voice.speaking)
    if mission_command.path is not None:
        path = mission_command.path
    if supervisor.autonomous:
        voice.announce(mission_command.announce)

    # The speed test runs without the sonar guard: it still observes, for
    # the GUI, but never slows or stops the car.
    guard_stopped = obstacle_guard.stopped and not speed_test
    guard_scale = 1.0 if speed_test else obstacle_guard.scale

    standing = supervisor.autonomous and (guard_stopped or prev_hold)
    perc = tracker.process(frame, params, anchor_for(path), learn=not standing)

    # Stopped for an obstacle the car is not moving: the detour's gap and
    # steady-lane timers must not count a car standing in the gap.
    obstacle_stopped = supervisor.autonomous and guard_stopped
    returned = None if obstacle_stopped else detour_return.update(path, perc, dt)
    fork_passed = not obstacle_stopped and fork_watch.update(path, perc, dt)
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

    manual = None
    if supervisor.autonomous:
        steering, throttle = autonomous.step(
            perc.lane_offset, perc.lane_curvature, params, dt, guard_scale,
            profile=detour_return.profile_for(path), provisional=perc.provisional, rows_found=perc.rows_found,
            far_offset=perc.far_offset, far_curvature=perc.far_curvature,
        )
        if not obstacle_stopped:
            detour_return.observe_hold(autonomous.holding, dt)
    else:
        autonomous.reset()
        # The pad (or the GUI) drives exactly as before; the dashboard's stick
        # drives under the supervisor's lease.
        manual = supervisor.manual(ps5.poll(), None if hub is None else hub.latest_drive(), now)
        steering, throttle = manual.steering, manual.throttle
        throttle *= params.base_speed

    if autonomous.stop_reason != prev_stop_reason:
        logger.info(f"Stop reason: {autonomous.stop_reason or 'none'} (lane {perc.lane_state})")
        prev_stop_reason = autonomous.stop_reason

    # Stop signs, lights and the mission's stops: the most restrictive wins.
    constraint = SignConstraint(reason="speed test") if speed_test else sign_policy.update(snapshot, now)
    scale = min(constraint.scale, mission_command.scale)
    hold = constraint.hold or mission_command.hold
    prev_hold = hold
    if supervisor.autonomous:
        if hold or throttle <= 0.0:
            throttle *= scale
        elif scale < 1.0:
            # Slow down, but never below the speed the car still moves at:
            # a ramp that bottoms out under the motors' minimum PWM parks
            # the car above the stop line, where the policy never reaches
            # its hold and so never releases.
            throttle = max(throttle * scale, autonomous.crawl_speed(params))
        if hold:
            # Stopped: keep sending the same steering so the PID's chase of
            # threshold jitter on a static frame cannot kick the wheel around.
            # The PID keeps running with its output discarded; no reset on
            # release, so there is no derivative spike.
            if held_steering is None:
                held_steering = steering
            steering = held_steering
        else:
            held_steering = None

    draw_steering_bar(perc.display_frame, steering)
    draw_throttle_bar(perc.display_frame, throttle)

    # The mission's signals only mean something when it is driving.
    if supervisor.autonomous and mission_command.turn_signal is not None:
        lights.signal_override = mission_command.turn_signal
    else:
        lights.signal_override = TurnSignal.RIGHT if path is Path.DETOUR else None

    # Only an obstacle or a dropped dashboard link forces hazards; a sign/light
    # stop just turns the strip red. The mission's own stops are left out: a
    # pickup signals left while still stopped before it pulls away.
    sign_stopped = supervisor.autonomous and constraint.hold
    link_stopped = manual is not None and manual.lease_lost
    lights.update(throttle, estopped=False, autonomous=supervisor.autonomous,
                  obstacle_stopped=obstacle_stopped or link_stopped, sign_stopped=sign_stopped, speed_test=speed_test,
                  scene=mission_command.scene if supervisor.autonomous else None)
    lights_output = lights.compute()
    arduino.send_lights(lights_output)
    arduino.send_motor(throttle, steering)
    if cargo is not None:
        # Only the mission holds the car at a dock: driven by hand it stays
        # locked. The master also refuses it while the wheels are driven.
        arduino.send_cargo(mission.unlock and supervisor.autonomous and not speed_test)

    curvature = perc.lane_curvature if perc.lane_curvature is not None else 0.0
    error = (perc.lane_offset or 0) + 60.0 * curvature

    if log_writer is not None:
        log_writer.writerow((
            f"{now - start_time:.3f}", frame_index, path.value, perc.lane_state, perc.rows_found, perc.both_rows,
            "" if perc.lane_offset is None else perc.lane_offset,
            "" if perc.lane_curvature is None else f"{perc.lane_curvature:.4f}",
            f"{error:.1f}", f"{steering:.4f}", f"{throttle:.4f}", f"{autonomous.holding:.2f}",
            autonomous.profile_label, autonomous.stop_reason or "",
            "" if last_sonar is None else f"{last_sonar.state}:{last_sonar.nearest_cm}",
            "" if last_sonar is None else " ".join("-" if d is None else str(d) for d in last_sonar.distances),
            obstacle_guard.label if obstacle_guard.nearest_cm is None else f"{obstacle_guard.label}:{obstacle_guard.nearest_cm}",
            mission.stage.value, mission.target or "", mission.next_exit, mission_command.state.value, int(fork_passed),
            # The sliders change between laps; a lap is only comparable with what it was driven with.
            f"{params.kp:.4f}", f"{params.kd:.4f}", f"{params.ema_alpha:.2f}",
            f"{params.roi_x0:.2f} {params.roi_x1:.2f} {params.roi_y0:.2f} {params.roi_y1:.2f}",
            f"{params.base_speed:.2f}", f"{params.min_speed:.2f}",
            "" if perc.far_offset is None else perc.far_offset,
            "" if perc.far_curvature is None else f"{perc.far_curvature:.4f}",
            "" if cargo is None else f"{cargo.grams} {int(cargo.stable)} {int(cargo.lid_closed)} "
                                     f"{int(cargo.unlocked)} {int(cargo.fault)}",
            challenge.value,
        ))

    gui.update_textures(perc)
    lane_text = perc.lane_state if perc.width_ratio is None else f"{perc.lane_state} w{perc.width_ratio:.2f}"
    if detour_return.stage:
        lane_text = f"{lane_text} | {detour_return.stage}"
    manual_note = None
    if manual is not None and manual.lease_lost:
        manual_note = "dashboard link lost - stopped"
    elif manual is not None and manual.waiting_neutral:
        manual_note = "release the throttle to drive"
    if supervisor.autonomous:
        stop = stop_cause(autonomous.stop_reason, autonomous.holding, hold, constraint, mission_command)
    else:
        # Manual: the controller and the policies are not applied.
        stop = None if manual_note is None else ("manual", manual_note)
    gui.update_telemetry(fps, int(round(error)), steering, throttle, lane_text, None if stop is None else stop[1])
    gui.update_path_status(path, perc.profile_ready, perc.lane_state)
    gui.update_sonar(last_sonar, obstacle_guard)
    applied = supervisor.autonomous and not speed_test
    why = "speed test" if speed_test else "manual"
    gui.update_challenge(challenge)
    gui.update_cargo(cargo)
    gui.update_signs(snapshot, constraint, applied=applied, why=why)
    gui.update_mission(mission, mission_command, applied=applied, why=why)
    publish(perc, steering, throttle, stop, constraint, mission_command, lights_output, applied, manual_note, cargo)
    gui.render_frame()

_close_recording()
cap.release()
if dashboard is not None:
    dashboard.stop()
sign_worker.close()
voice.close()
ps5.shutdown()
arduino.close()
gui.shutdown()
