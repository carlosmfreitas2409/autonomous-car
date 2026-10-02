import argparse
import csv
import cv2 as cv
import logging
import time

from arduino import Arduino

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("main")

from config import Config
from lights import LightsController, TurnSignal
from perception.line_tracking import LineTracker
from input.autonomous import Autonomous
from input.ps5 import BUTTON_NAMES, PS5Controller
from input.sign_policy import SignConstraint, SignPolicy
from input.mission import Mission, MissionCommand
from path import HOME_PATH, DetourReturn, Path, anchor_for, profile_for
from perception.sign_worker import SignWorker
from gui import TunerGUI


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


_LOG_FIELDS = ("t", "frame", "path", "lane_state", "rows", "both_rows", "offset", "curvature", "error", "steering", "throttle", "holding", "profile", "stop_reason", "sonar", "sonar_lr", "mission_stage", "target", "next_exit", "mission_state")


parser = argparse.ArgumentParser(description="Autonomous car tuner")
parser.add_argument("--video", help="replay a recorded video instead of the camera")
parser.add_argument("--record", help="write every camera frame to this file, for replaying later")
parser.add_argument("--flip", action="store_true", help="mirror frames left-right (for a clip recorded flipped)")
parser.add_argument("--camera", type=int, default=1, help="camera index when not replaying a video (default 0)")
parser.add_argument("--log", help="write per-frame telemetry to this CSV")
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
    log_file = open(args.log, "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow(_LOG_FIELDS)

config = Config()
ratio = config.downscale_ratio
frame_w = round(first_frame.shape[1] / ratio)
frame_h = round(first_frame.shape[0] / ratio)
cap.set(cv.CAP_PROP_POS_FRAMES, 0)

gui = TunerGUI(config, frame_w, frame_h)
arduino = Arduino()
tracker = LineTracker()
autonomous = Autonomous()
detour_return = DetourReturn()
ps5 = PS5Controller()
lights = LightsController()
sign_cam_indices = tuple(i for i in SIGN_CAM_INDICES if args.video or i != args.camera)
sign_worker = SignWorker(sign_cam_indices, SIGN_MODEL_PATH, SIGN_CONFIDENCE)
sign_policy = SignPolicy()
mission = Mission()
sign_worker.start()

gui.set_pid_reset_callback(autonomous.reset)
gui.set_master_port_callback(arduino.set_master_port)
gui.set_lights_controller(lights)

fps = 0.0
start_time = time.monotonic()
prev_time = start_time
prev_stopped = False
last_sonar = None
held_steering = None
prev_stop_reason = None
path = HOME_PATH
prev_path = path
prev_lane_state = None
frame_index = -1
warned_calibrating = False

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

    # Path buttons are read before the stopped branch so the Detour can be
    # armed while the car waits at the sign.
    for requested_path in (ps5.poll_path_buttons(), gui.poll_path_request()):
        if requested_path is not None:
            path = requested_path

    letters = gui.poll_mission_request()
    if letters is not None:
        if gui.is_stopped():
            mission.configure(*letters)
        else:
            logger.warning("Pickup/delivery can only be changed while e-stopped")

    sonar = arduino.read_sonar()
    if sonar is not None:
        last_sonar = sonar

    if gui.is_stopped():
        if not prev_stopped:
            arduino.send_estop()
        prev_stopped = True
        gui.update_sonar(last_sonar)
        autonomous.reset()
        sign_policy.reset()
        mission.reset()
        held_steering = None
        lights.signal_override = TurnSignal.RIGHT if path is Path.DETOUR else None
        lights.update(0.0, estopped=True, autonomous=gui.is_autonomous())
        lights_output = lights.compute()
        arduino.send_lights(lights_output)
        gui.update_signs(sign_worker.get_snapshot(), SignConstraint(reason="e-stop"), applied=False)
        gui.update_mission(mission, MissionCommand(reason="e-stop"), applied=False)
        gui.update_path_status(path, tracker.profile_ready, "e-stop")
        gui.render_frame()
        continue
    prev_stopped = False

    ret, frame = cap.read()
    if not ret:
        logger.warning("Frame read failed (camera dropout or end of video), rewinding")
        cap.set(cv.CAP_PROP_POS_FRAMES, 0)
        tracker.reset()
        autonomous.reset()
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
    mission_command = mission.update(snapshot, path, now)
    if mission_command.path is not None:
        path = mission_command.path

    perc = tracker.process(frame, params, anchor_for(path))

    returned = detour_return.update(path, perc, dt)
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
            perc.lane_offset, perc.lane_curvature, params, dt, last_sonar,
            profile=detour_return.profile_for(path), provisional=perc.provisional,
        )
        detour_return.observe_hold(autonomous.holding, dt)
    else:
        autonomous.reset()
        steering, throttle = ps5.poll()
        throttle *= params.base_speed

    if autonomous.stop_reason != prev_stop_reason:
        logger.info(f"Stop reason: {autonomous.stop_reason or 'none'} (lane {perc.lane_state})")
        prev_stop_reason = autonomous.stop_reason

    # Stop signs, lights and the mission's stops: the most restrictive wins.
    constraint = sign_policy.update(snapshot, now)
    scale = min(constraint.scale, mission_command.scale)
    hold = constraint.hold or mission_command.hold
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

    obstacle_blocked = gui.is_autonomous() and ((last_sonar is not None and last_sonar.blocked) or hold)
    lights.update(throttle, estopped=False, autonomous=gui.is_autonomous(), obstacle_blocked=obstacle_blocked)
    lights_output = lights.compute()
    arduino.send_lights(lights_output)
    arduino.send_motor(throttle, steering)

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
            mission.stage.value, mission.target or "", mission.next_exit, mission_command.state.value,
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
    gui.update_sonar(last_sonar)
    gui.update_signs(snapshot, constraint, applied=gui.is_autonomous())
    gui.update_mission(mission, mission_command, applied=gui.is_autonomous())
    gui.render_frame()

cap.release()
sign_worker.close()
if recorder is not None:
    recorder.release()
if log_file is not None:
    log_file.close()
ps5.shutdown()
arduino.close()
gui.shutdown()
