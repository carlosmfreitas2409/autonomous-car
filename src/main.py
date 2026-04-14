import cv2 as cv
import numpy as np
ratio = 1.5

cap = cv.VideoCapture('road.mov')

if not cap.isOpened():
  print("Error: Could not open video.")
  exit()

while True:
  ret, frame = cap.read()

  if not ret:
    break

  height = round(frame.shape[0] / ratio)
  width = round(frame.shape[1] / ratio)

  center_x = width // 2
  
  frame = cv.resize(frame, (width, height))
  
  gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
  blur = cv.medianBlur(gray, 3)
  _, binary = cv.threshold(blur, 95, 255, cv.THRESH_BINARY)

  # ROI
  # roi_width = round(width / 9)
  # width - roi_width
  # roi_height = round(height / 1.3)
  # height - round(roi_height * 0.1)

  roi_x0, roi_x1 = round(width * 0.1), round(width * 0.9)
  roi_y0, roi_y1 = round(height * 0.8), round(height * 0.92)

  roi_center_x = (roi_x0 + roi_x1) // 2
  roi_center_y = (roi_y0 + roi_y1) // 2

  cv.rectangle(
    blur,
    (roi_x0, roi_y0),
    (roi_x1, roi_y1),
    255,
  )

  # mask = np.zeros_like(binary)
  # x0, x1 = int(width * 0.15), int(width * 0.85)
  # y0, y1 = int(height * 0.7), int(height * 0.92)
  # polygon = np.array([[(x0, y0), (x1, y0), (x1, y1), (x0, y1)]])
  # cv.fillPoly(mask, polygon, 255)
  # roi = cv.bitwise_and(binary, mask)

  # Road tracking with multiple scan lines
  scan_ys = np.linspace(roi_y0, roi_y1, 5, dtype=int)
  centers = []

  cv.line(blur, (roi_center_x, roi_y0), (roi_center_x, roi_y1), 255)

  for sy in scan_ys:
    left_x = -1
    right_x = -1

    for i in range(center_x, roi_x0, -1):
      if binary[sy, i] == 255:
        left_x = i
        break

    for i in range(center_x, roi_x1, 1):
      if binary[sy, i] == 255:
        right_x = i
        break

    if left_x != -1 and right_x != -1:
      centers.append((left_x + right_x) // 2)
      cv.line(blur, (left_x, sy), (right_x, sy), 255)

  if centers:
    track_center_x = int(np.median(centers))
    cv.line(blur, (track_center_x, roi_y0), (track_center_x, roi_y1), 255, 2)

  img_concat = cv.hconcat([
    frame,
    cv.cvtColor(blur, cv.COLOR_GRAY2BGR),
    cv.cvtColor(binary, cv.COLOR_GRAY2BGR),
  ])

  cv.imshow('Road', img_concat)

  if cv.waitKey(1) & 0xFF == 27:
    break

cap.release()
cv.destroyAllWindows()