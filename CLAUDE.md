# Autonomous RC Car — Project Context

## What this is

A Python + Arduino software stack for an RC-scale autonomous vehicle. The car is driven with a PS5 DualSense controller over serial. Autonomous driving (camera + computer vision) will be added in the future.

The physical car is a 4-wheeled vehicle with independent motors on each wheel (4x BTS7960 H-bridges). Steering mechanism is TBD (trapezoidal spindle or Ackermann differential).

## Stack

- **Python 3.14** + **pygame 2.6** on macOS
- **Arduino Mega** (4x BTS7960 H-bridges, one per wheel, PWM pins 2–3, 5–10)
- **pyserial** for serial communication

## Project structure

```
src/                         — real car code
  main.py                    — main loop (PS5 → serial → Arduino)
  config.py                  — tunable constants
  input.py                   — PS5Controller, deadzone
  control.py                 — Smoother, EStopManager, CommandWatchdog, Controller
  vehicle/
    arduino.py               — ArduinoVehicle (serial, PWM encoding)
    protocol.py              — SerialProtocol (message format/parse)
  autonomous/                — routing logic (CV integration TODO)
    route_table.py           — pre-computed route table for all 12 pickup/delivery combos
    mission.py               — MissionPlanner (tracks pickup/delivery progress)
    state_machine.py         — behavior states (waiting, navigating, stopped, complete)

tools/                       — dev/testing tools (not deployed on the car)
  simulator.py               — pygame sim for PS5 controller testing
  physics.py                 — bicycle model physics (used by simulator + tests)
  route_demo.py              — animated route visualization
  run_sim3d.py               — 3D simulator entry point (PS5 controller)
  sim3d/                     — Panda3D 3D simulator package
    vehicle.py               — SimulatedVehicle3D (drop-in ArduinoVehicle replacement)
    track.py                 — procedural track geometry from SVG
    camera.py                — DashCam (offscreen → numpy BGR) + debug cameras
    props.py                 — signs, traffic light
    config.py                — simulator constants

arduino/
  motor_controller/motor_controller.ino

tests/
  test_control.py, test_physics.py, test_protocol.py
  test_route_table.py, test_state_machine.py, test_mission.py
```

## Running

```bash
# Drive the car (PS5 controller over serial)
python3 -m src

# PS5 controller testing simulator (no hardware needed)
python3 tools/simulator.py

# 3D simulator for CV testing (Panda3D, requires: pip install panda3d)
python3 tools/run_sim3d.py

# Route visualization demo
python3 tools/route_demo.py

# Tests
python3 -m pytest tests/ -v
```

## Key design decisions

- **`src/` is real car code only** — simulator, physics, and renderer live in `tools/`. No coordinate-based sign detection in `src/`; the real car will use camera + CV.
- **Throttle/steering are floats throughout** (`0..1` and `-1..1`). PWM encoding (`* 255`) only happens at the Arduino serial boundary in `arduino.py`.
- **Deadzone applied once**, in `input.py`, with rescaling: `sign(v) * (|v| - threshold) / (1 - threshold)`.
- **E-stop is a toggle** — Circle button on PS5 controller. Activating resets the smoothers so throttle ramps from zero on resume (no lurch).
- **Watchdog** — if no controller input arrives for 0.5s, e-stop activates. Arduino also has its own 500ms serial watchdog.
- **Pre-computed route table** — all 12 pickup/delivery combos are pre-computed. The track is an oval with two inner curve shortcuts; the route table picks the shortest path.

## Autonomous mode (future)

The car will do pickup/delivery missions on an oval track with 4 numbered signs. Before each run, two signs are assigned (pickup + delivery). The car must:
1. Drive to the pickup sign, stop for ~4 seconds
2. Continue to the delivery sign, stop for ~4 seconds

Sign detection will use **camera + computer vision** (OpenCV), not coordinate-based detection. Lane following will also use camera-based lane detection.

The routing logic (`src/autonomous/`) is already implemented and tested. What remains is the CV perception layer and wiring it into the main loop.

## Serial protocol (Python → Arduino)

```
M <throttle_pwm> <steering_pwm>\n  — motor command (-255..255)
E\n                                — emergency stop
P\n                                — heartbeat
```

The Arduino Mega applies throttle to all 4 motors equally. Steering mixing is a TODO (see comments in the Arduino sketch for planned approaches).

Arduino → Python: `S <dist_front> <dist_left> <dist_right> <imu_heading>\n`

## PS5 controller mapping

| Input | Axis/Button | Action |
|---|---|---|
| R2 trigger | axis 5 (-1..1 → 0..1) | throttle (forward only) |
| Left stick X | axis 0 (inverted) | steering |
| Circle | button 1 | toggle e-stop |

## Known quirks

- `python` command not found on this machine — use `python3`.
