# Autonomous RC Car — Project Context

## What this is

A Python + Arduino software stack for an RC-scale autonomous vehicle. There are two modes:

- **`sim`** — runs a 2D pygame simulator with bicycle-model physics, a live HUD, and a trail renderer. Used for development and testing without hardware.
- **`arduino`** — sends throttle/steering commands over serial to an Arduino that drives an H-bridge motor controller. No renderer runs in this mode.

The physical car is a 4-wheeled RC vehicle with front steering (not differential drive). The driver is a PS5 DualSense controller connected via USB.

## Stack

- **Python 3.14** + **pygame 2.6** on macOS
- **Arduino** (H-bridge motor driver, pins 4–7)
- **pyserial** for serial communication

## Project structure

```
src/
  main.py          — main loop
  config.py        — all tunable constants (set MODE here)
  input.py         — PS5Controller, deadzone
  control.py       — Smoother, EStopManager, CommandWatchdog, Controller
  simulation/
    physics.py     — BicyclePhysics, VehicleState
    renderer.py    — pygame rendering (sim mode only)
  vehicle/
    simulated.py   — SimulatedVehicle (wraps physics)
    arduino.py     — ArduinoVehicle (serial, PWM encoding)
    protocol.py    — SerialProtocol (message format/parse)
arduino/
  motor_controller/motor_controller.ino
tests/
  test_control.py, test_physics.py, test_protocol.py
```

## Running

```bash
# Simulator
python3 -m src

# Arduino mode (set MODE = "arduino" in config.py first)
python3 -m src

# Tests
python3 -m pytest tests/ -v
```

## Key design decisions

- **Throttle/steering are floats throughout** (`0..1` and `-1..1`). PWM encoding (`* 255`) only happens at the Arduino serial boundary in `arduino.py`.
- **Deadzone applied once**, in `input.py`, with rescaling: `sign(v) * (|v| - threshold) / (1 - threshold)`.
- **Bicycle model physics** — kinematic, treats the 4-wheeled car as 2 virtual centerline wheels with front steering. Steering alone doesn't move the car (no spin-in-place).
- **E-stop is a toggle** — Circle button on PS5 controller. Activating resets the smoothers so throttle ramps from zero on resume (no lurch).
- **Renderer and SimulatedVehicle are only instantiated in sim mode** — arduino mode has no pygame window.
- **Watchdog** — if no controller input arrives for 0.5s, e-stop activates. Arduino also has its own 500ms serial watchdog.

## Serial protocol (Python → Arduino)

```
M <left_pwm> <right_pwm>\n   — motor command (-255..255)
E\n                           — emergency stop
P\n                           — heartbeat
```

Arduino → Python: `S <dist_front> <dist_left> <dist_right> <imu_heading>\n`

## PS5 controller mapping

| Input | Axis/Button | Action |
|---|---|---|
| R2 trigger | axis 5 (-1..1 → 0..1) | throttle (forward only) |
| Left stick X | axis 0 (inverted) | steering |
| Circle | button 1 | toggle e-stop |

## Known quirks

- `python` command not found on this machine — use `python3`.
