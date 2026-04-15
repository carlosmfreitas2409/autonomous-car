import cv2 as cv
import numpy as np
import time

from pid import PIDController

ratio = 1.5

# PID tuning parameters
# Output range: -1.0 (full left) to +1.0 (full right)
Kp = 0.01
Ki = 0.0
Kd = 0.005

pid = PIDController(kp=Kp, ki=Ki, kd=Kd, output_limits=(-1.0, 1.0))

cap = cv.VideoCapture('road.mov')

if not cap.isOpened():
  print("Error: Could not open video.")
  exit()

prev_track_center = None
smoothed_center = None
alpha = 0.4

prev_time = time.time()
fps = 0.0

while True:
  ret, frame = cap.read()

  if not ret:
    cap.set(cv.CAP_PROP_POS_FRAMES, 0)
    prev_track_center = None
    smoothed_center = None
    pid.reset()
    continue

  # FPS
  now = time.time()
  dt = now - prev_time
  prev_time = now
  if dt > 0:
    fps = 0.9 * fps + 0.1 * (1.0 / dt)

  height = round(frame.shape[0] / ratio)
  width = round(frame.shape[1] / ratio)

  center_x = width // 2

  frame = cv.resize(frame, (width, height))

  gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
  blur = cv.medianBlur(gray, 3)
  binary = cv.adaptiveThreshold(blur, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 51, -25)
  # _, binary = cv.threshold(blur, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

  roi_x0, roi_x1 = round(width * 0.1), round(width * 0.9)
  roi_y0, roi_y1 = round(height * 0.8), round(height * 0.92)

  roi_center_x = (roi_x0 + roi_x1) // 2

  blur_display = cv.cvtColor(blur, cv.COLOR_GRAY2BGR)
  cv.rectangle(blur_display, (roi_x0, roi_y0), (roi_x1, roi_y1), (0, 255, 0), 1)

  # Road tracking with multiple scan lines
  scan_ys = np.linspace(roi_y0, roi_y1, 5, dtype=int)
  centers = []

  scan_origin = prev_track_center if prev_track_center is not None else center_x

  cv.line(blur_display, (roi_center_x, roi_y0), (roi_center_x, roi_y1), (255, 255, 255), 1)

  for sy in scan_ys:
    left_x = -1
    right_x = -1

    for i in range(scan_origin, roi_x0, -1):
      if binary[sy, i] == 255:
        left_x = i
        break

    for i in range(scan_origin, roi_x1, 1):
      if binary[sy, i] == 255:
        right_x = i
        break

    if left_x != -1 and right_x != -1:
      centers.append((left_x + right_x) // 2)
      cv.line(blur_display, (left_x, sy), (right_x, sy), (0, 255, 255), 1)

  if centers:
    track_center_x = int(np.median(centers))
    prev_track_center = track_center_x

    if smoothed_center is None:
      smoothed_center = track_center_x
    else:
      smoothed_center = alpha * track_center_x + (1 - alpha) * smoothed_center

    smoothed_x = int(smoothed_center)
    error = smoothed_x - roi_center_x

    steering = pid.update(error, dt)

    cv.line(blur_display, (smoothed_x, roi_y0), (smoothed_x, roi_y1), (0, 0, 255), 2)
    cv.putText(frame, f"Error: {error:+d}px", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv.putText(frame, f"Steer: {steering:+.2f}", (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Steering bar visualization
    bar_y = height - 30
    bar_w = width - 40
    bar_cx = center_x
    cv.line(frame, (20, bar_y), (20 + bar_w, bar_y), (100, 100, 100), 2)
    cv.line(frame, (bar_cx, bar_y - 8), (bar_cx, bar_y + 8), (255, 255, 255), 1)
    indicator_x = int(bar_cx + steering * (bar_w // 2))
    indicator_x = max(20, min(20 + bar_w, indicator_x))
    color = (0, 0, 255) if abs(steering) > 0.5 else (0, 255, 255) if abs(steering) > 0.2 else (0, 255, 0)
    cv.circle(frame, (indicator_x, bar_y), 6, color, -1)

  cv.putText(frame, f"FPS: {fps:.1f}", (10, 90), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

  img_concat = cv.hconcat([
    frame,
    blur_display,
    cv.cvtColor(binary, cv.COLOR_GRAY2BGR),
  ])

  cv.imshow('Road', img_concat)

  if cv.waitKey(1) & 0xFF == 27:
    break

cap.release()
cv.destroyAllWindows()
