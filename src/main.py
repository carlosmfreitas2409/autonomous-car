import cv2 as cv
import time

from arduino import Arduino
from config import Config
from lights import LightsController
from perception.line_tracking import LineTracker
from input.autonomous import Autonomous
from input.ps5 import PS5Controller
from gui import TunerGUI


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
    bar_x = w - 30
    t = max(0.0, min(1.0, throttle))
    cv.rectangle(frame, (bar_x - 6, bar_y0), (bar_x + 6, bar_y1), (100, 100, 100), 1)
    fill_h = int(bar_h * t)
    fill_y0 = bar_y1 - fill_h
    color = (0, int(255 * (1.0 - t)), int(255 * t))
    if fill_h > 0:
        cv.rectangle(frame, (bar_x - 5, fill_y0), (bar_x + 5, bar_y1), color, -1)


cap = cv.VideoCapture('assets/road.mov')
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

ret, first_frame = cap.read()
if not ret:
    print("Error: Could not read first frame.")
    exit()

ratio = Config().downscale_ratio
frame_w = round(first_frame.shape[1] / ratio)
frame_h = round(first_frame.shape[0] / ratio)
cap.set(cv.CAP_PROP_POS_FRAMES, 0)

gui = TunerGUI(frame_w, frame_h)
arduino = Arduino()
tracker = LineTracker()
autonomous = Autonomous()
ps5 = PS5Controller()
lights = LightsController()

gui.set_pid_reset_callback(autonomous.reset)
gui.set_list_ports_callback(arduino.list_ports)
gui.set_master_port_callback(arduino.set_master_port)
gui.set_lights_port_callback(arduino.set_lights_port)
gui.set_lights_controller(lights)

fps = 0.0
prev_time = time.time()

while gui.is_running():
    now = time.time()
    dt = now - prev_time
    prev_time = now

    if dt > 0:
        fps = 0.9 * fps + 0.1 / dt

    if gui.is_stopped():
        autonomous.reset()
        lights.stopped = True
        lights_output = lights.compute()
        arduino.send_lights(lights_output)
        # gui.update_lights(lights_output)
        gui.render_frame()
        continue

    ret, frame = cap.read()
    if not ret:
        cap.set(cv.CAP_PROP_POS_FRAMES, 0)
        tracker.reset()
        autonomous.reset()
        continue

    params = gui.read_params()

    perc = tracker.process(frame, params)

    if gui.is_autonomous():
        steering, throttle = autonomous.step(perc.lane_offset, params, dt)
    else:
        autonomous.reset()
        steering, throttle = ps5.poll()

    draw_steering_bar(perc.display_frame, steering)
    draw_throttle_bar(perc.display_frame, throttle)

    lights.stopped = throttle < 0.05
    lights_output = lights.compute()
    arduino.send_lights(lights_output)

    gui.update_textures(perc)
    gui.update_telemetry(fps, perc.lane_offset or 0, steering, throttle)
    # gui.update_lights(lights_output)
    gui.render_frame()

cap.release()
ps5.shutdown()
gui.shutdown()
