import cv2 as cv
import logging
import time

from arduino import Arduino

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

from config import Config
from lights import LightsController
from perception.line_tracking import LineTracker
from perception.sign_worker import SignWorker
from input.autonomous import Autonomous
from input.ps5 import PS5Controller
from input.traffic_policy import PolicyState, TrafficPolicy
from gui import TunerGUI


# Two cameras: the drive camera is angled down at the lane, the sign camera
# looks forward. macOS reorders indices between reboots - tools/list_cameras.py
# shows which is which.
DRIVE_CAM_INDEX = 0
SIGN_CAM_INDEX = 1

SIGN_MODEL_PATH = "models/signs.onnx"


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


def draw_policy_banner(frame, constraint):
    """Show what sign perception is asking for, on the drive view."""
    if constraint.state is PolicyState.CLEAR:
        return

    colour = (0, 200, 255) if constraint.state is PolicyState.SLOWING else (0, 0, 255)
    label = constraint.state.value.upper().replace("_", " ")
    if constraint.reason:
        label += f" - {constraint.reason}"

    (text_w, text_h), _ = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    x = max(10, (frame.shape[1] - text_w) // 2)
    cv.rectangle(frame, (x - 8, 8), (x + text_w + 8, 16 + text_h + 8), (0, 0, 0), -1)
    cv.putText(frame, label, (x, 16 + text_h), cv.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2, cv.LINE_AA)


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


cap = cv.VideoCapture(DRIVE_CAM_INDEX)
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

ret, first_frame = cap.read()
if not ret:
    print("Error: Could not read first frame.")
    exit()

config = Config()
ratio = config.downscale_ratio
frame_w = round(first_frame.shape[1] / ratio)
frame_h = round(first_frame.shape[0] / ratio)
cap.set(cv.CAP_PROP_POS_FRAMES, 0)

gui = TunerGUI(config, frame_w, frame_h)
arduino = Arduino()
tracker = LineTracker()
autonomous = Autonomous()
ps5 = PS5Controller()
lights = LightsController()
traffic_policy = TrafficPolicy()

# Sign perception runs entirely on its own thread and its own camera. It opens
# that camera from inside the thread, so a slow or absent sign camera delays
# startup by nothing.
sign_worker = SignWorker(
    camera_index=SIGN_CAM_INDEX,
    model_path=SIGN_MODEL_PATH,
    confidence=config.sign_confidence,
    enabled=config.enable_sign_detection,
)
sign_worker.start()

gui.set_pid_reset_callback(autonomous.reset)
gui.set_master_port_callback(arduino.set_master_port)
gui.set_lights_controller(lights)

fps = 0.0
prev_time = time.monotonic()
prev_stopped = False

while gui.is_running():
    now = time.monotonic()
    dt = now - prev_time
    prev_time = now

    if dt > 0:
        fps = 0.9 * fps + 0.1 / dt

    if ps5.poll_estop_toggle():
        gui.toggle_estop()

    if ps5.poll_mode_toggle():
        gui.toggle_mode()

    if gui.is_stopped():
        if not prev_stopped:
            arduino.send_estop()
        prev_stopped = True
        autonomous.reset()
        traffic_policy.reset()
        lights.update(0.0, estopped=True)
        lights_output = lights.compute()
        arduino.send_lights(lights_output)
        gui.render_frame()
        continue
    prev_stopped = False

    ret, frame = cap.read()
    if not ret:
        cap.set(cv.CAP_PROP_POS_FRAMES, 0)
        tracker.reset()
        autonomous.reset()
        continue

    params = gui.read_params()

    perc = tracker.process(frame, params)

    # Two scalar assignments, no lock, no frame handling: this is the whole
    # cost of sign detection inside the control loop.
    sign_worker.enabled = params.enable_sign_detection
    sign_worker.confidence = params.sign_confidence
    signs = sign_worker.get_snapshot()

    if gui.is_autonomous():
        steering, throttle = autonomous.step(perc.lane_offset, perc.lane_curvature, params, dt)
    else:
        autonomous.reset()
        steering, throttle = ps5.poll()
        throttle *= params.base_speed

    # The policy runs in both modes but only applies in autonomous. Driving the
    # track manually while watching it approach, hold, and release is how you
    # validate the behaviour without it fighting you for the throttle.
    constraint = traffic_policy.update(signs, throttle, dt, now)
    if gui.is_autonomous():
        throttle = min(throttle, constraint.speed_cap)

    draw_steering_bar(perc.display_frame, steering)
    draw_throttle_bar(perc.display_frame, throttle)
    draw_policy_banner(perc.display_frame, constraint)

    lights.update(throttle, estopped=False)
    lights_output = lights.compute()
    arduino.send_lights(lights_output)
    arduino.send_motor(throttle, steering)

    gui.update_textures(perc)
    gui.update_telemetry(fps, perc.lane_offset or 0, steering, throttle)
    gui.update_sign_view(signs, constraint)
    gui.render_frame()

cap.release()
sign_worker.close()
ps5.shutdown()
arduino.close()
gui.shutdown()
