import cv2 as cv
import time
import dearpygui.dearpygui as dpg

from pipeline import PipelineParams, PipelineState, process_frame
from gui import create_gui, read_params, update_textures, update_telemetry, set_pid_reset_callback

cap = cv.VideoCapture('road.mov')

if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# Read first frame to get display dimensions
ret, first_frame = cap.read()
if not ret:
    print("Error: Could not read first frame.")
    exit()

ratio = 1.5
frame_h = round(first_frame.shape[0] / ratio)
frame_w = round(first_frame.shape[1] / ratio)

# Reset video to beginning
cap.set(cv.CAP_PROP_POS_FRAMES, 0)

# Setup GUI
tags = create_gui(frame_w, frame_h)
state = PipelineState(PipelineParams())
last_frame = None


def on_pid_reset():
    state.pid.reset()


set_pid_reset_callback(on_pid_reset)

prev_time = time.time()

while dpg.is_dearpygui_running():
    now = time.time()
    dt = now - prev_time
    prev_time = now

    params = read_params()

    if not params.stopped:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv.CAP_PROP_POS_FRAMES, 0)
            state.reset()
            continue
        last_frame = frame
    else:
        state.pid.reset()
        frame = last_frame

    if frame is None:
        dpg.render_dearpygui_frame()
        continue

    result = process_frame(frame, params, state, dt)
    update_textures(result)
    update_telemetry(result)
    dpg.render_dearpygui_frame()

cap.release()
dpg.destroy_context()
