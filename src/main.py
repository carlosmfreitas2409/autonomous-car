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
parser.add_argument("--camera", type=int, default=2, help="camera index when not replaying a video (default 2)")
parser.add_argument("--log", help="write per-frame telemetry to this CSV")
parser.add_argument("--no-voice", action="store_true", help="no spoken prompts at the docks")
parser.add_argument("--voice", help='dock prompt voice: a `say` voice name, or "system" for the Mac\'s system voice (e.g. a Siri voice); default Luciana')
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
sign_worker.start()

gui.set_pid_reset_callback(autonomous.reset)
gui.set_master_port_callback(arduino.set_master_port)
gui.set_lights_controller(lights)

challenge = Challenge.DELIVERY
tuning = {Challenge.DELIVERY: config, Challenge.SPEED: SPEED_TEST}

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

while gui.is_running():
    now = time.monotonic()
    dt = now - prev_time
    prev_time = now

    if dt > 0:
        fps = 0.9 * fps + 0.1 / dt

    for index in ps5.poll_button_presses():
        logger.info(f"PS5 button {index} ({BUTTON_NAMES.get(index, '?')})")

    if ps5.poll_estop_toggle():
        gui.toggle_estop()

    if ps5.poll_mode_toggle():
        gui.toggle_mode()

    requested_challenge = gui.poll_challenge_request()
    if requested_challenge is not None and requested_challenge is not challenge:
        if gui.is_stopped():
            tuning[challenge] = gui.read_params()
            challenge = requested_challenge
            gui.load_tuning(tuning[challenge])
            path = HOME_PATH
            detour_return = DetourReturn()
            fork_watch = ForkWatch()
            fork_passed = False
            sign_worker.set_paused(challenge is Challenge.SPEED)
            logger.info(f"Stage: {challenge.value}")
        else:
            logger.warning("The stage can only be changed while e-stopped")
    speed_test = challenge is Challenge.SPEED

    # Path buttons are read before the stopped branch so the Detour can be
    # armed while the car waits at the sign.
    for requested_path in (ps5.poll_path_buttons(), gui.poll_path_request()):
        if requested_path is not None and not speed_test:
            path = requested_path

    letters = gui.poll_mission_request()
    if letters is not None:
        # Parked after a delivery, the next order is given where it stands;
        # clearing both letters forgets that dock, so that needs an e-stop.
        if gui.is_stopped() or (mission.parked and letters != (None, None)):
            mission.configure(*letters)
        else:
            logger.warning("Pickup/delivery can only be changed while e-stopped or parked at a delivery")

    # The person at the dock: loading or unloading is done ("release"), or
    # not yet after all ("wait"). Triangle's e-stop is the PS5's wait.
    for request in (gui.poll_dock_request(), "release" if ps5.poll_release() else None):
        if request is None:
            continue
        refused = "e-stopped, resume first" if gui.is_stopped() else "no docks in the speed test" if speed_test else (
            mission.release(now) if request == "release" else mission.hold_departure())
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
        if gui.is_stopped():
            manual_open.request(now)
        else:
            logger.warning("Open compartment refused: e-stop first")
    open_by_hand = manual_open.update(now, cargo, stopped=gui.is_stopped())

    if gui.is_stopped():
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
        lights.update(0.0, estopped=True, autonomous=gui.is_autonomous(), speed_test=speed_test,
                      scene=None if speed_test else mission.scene)
        lights_output = lights.compute()
        arduino.send_lights(lights_output)
        gui.update_challenge(challenge)
        gui.update_signs(sign_worker.get_snapshot(), SignConstraint(reason="e-stop"), applied=False)
        gui.update_mission(mission, MissionCommand(reason="e-stop"), applied=False)
        gui.update_path_status(path, tracker.profile_ready, "e-stop")
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

    params = gui.read_params()

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
    if gui.is_autonomous():
        voice.announce(mission_command.announce)

    # The speed test runs without the sonar guard: it still observes, for
    # the GUI, but never slows or stops the car.
    guard_stopped = obstacle_guard.stopped and not speed_test
    guard_scale = 1.0 if speed_test else obstacle_guard.scale

    standing = gui.is_autonomous() and (guard_stopped or prev_hold)
    perc = tracker.process(frame, params, anchor_for(path), learn=not standing)

    # Stopped for an obstacle the car is not moving: the detour's gap and
    # steady-lane timers must not count a car standing in the gap.
    obstacle_stopped = gui.is_autonomous() and guard_stopped
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

    if gui.is_autonomous():
        steering, throttle = autonomous.step(
            perc.lane_offset, perc.lane_curvature, params, dt, guard_scale,
            profile=detour_return.profile_for(path), provisional=perc.provisional, rows_found=perc.rows_found,
            far_offset=perc.far_offset, far_curvature=perc.far_curvature,
        )
        if not obstacle_stopped:
            detour_return.observe_hold(autonomous.holding, dt)
    else:
        autonomous.reset()
        steering, throttle = ps5.poll()
        throttle *= params.base_speed

    if autonomous.stop_reason != prev_stop_reason:
        logger.info(f"Stop reason: {autonomous.stop_reason or 'none'} (lane {perc.lane_state})")
        prev_stop_reason = autonomous.stop_reason

    # Stop signs, lights and the mission's stops: the most restrictive wins.
    constraint = SignConstraint(reason="speed test") if speed_test else sign_policy.update(snapshot, now)
    scale = min(constraint.scale, mission_command.scale)
    hold = constraint.hold or mission_command.hold
    prev_hold = hold
    if gui.is_autonomous():
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
    if gui.is_autonomous() and mission_command.turn_signal is not None:
        lights.signal_override = mission_command.turn_signal
    else:
        lights.signal_override = TurnSignal.RIGHT if path is Path.DETOUR else None

    # Only an obstacle forces hazards; a sign/light stop just turns the strip
    # red. The mission's own stops are left out: a pickup signals left while
    # still stopped before it pulls away.
    sign_stopped = gui.is_autonomous() and constraint.hold
    lights.update(throttle, estopped=False, autonomous=gui.is_autonomous(),
                  obstacle_stopped=obstacle_stopped, sign_stopped=sign_stopped, speed_test=speed_test,
                  scene=mission_command.scene if gui.is_autonomous() else None)
    lights_output = lights.compute()
    arduino.send_lights(lights_output)
    arduino.send_motor(throttle, steering)
    if cargo is not None:
        # Only the mission holds the car at a dock: driven by hand it stays
        # locked. The master also refuses it while the wheels are driven.
        arduino.send_cargo(mission.unlock and gui.is_autonomous() and not speed_test)

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
    stop_text = autonomous.stop_reason
    if stop_text is None and hold:
        # The sign policy and the mission stop the car without the controller
        # knowing why, so the one red label still answers "why is it not moving".
        if constraint.hold:
            stop_text = constraint.reason or constraint.state.value
        else:
            stop_text = mission_command.reason or mission_command.state.value
    if stop_text is None and autonomous.holding > 0:
        stop_text = f"holding {autonomous.holding:.1f}s"
    gui.update_telemetry(fps, int(round(error)), steering, throttle, lane_text, stop_text)
    gui.update_path_status(path, perc.profile_ready, perc.lane_state)
    gui.update_sonar(last_sonar, obstacle_guard)
    applied = gui.is_autonomous() and not speed_test
    why = "speed test" if speed_test else "manual"
    gui.update_challenge(challenge)
    gui.update_cargo(cargo)
    gui.update_signs(snapshot, constraint, applied=applied, why=why)
    gui.update_mission(mission, mission_command, applied=applied, why=why)
    gui.render_frame()

_close_recording()
cap.release()
sign_worker.close()
voice.close()
ps5.shutdown()
arduino.close()
gui.shutdown()
