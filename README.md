# Autonomous RC Car

Python + Arduino software stack for an RC-scale autonomous vehicle with a PS5 DualSense controller.

## Modes

| Mode | What runs |
|---|---|
| `sim` | pygame simulator with bicycle-model physics, HUD, and trail renderer |
| `arduino` | serial commands to Arduino H-bridge motor controller — no window |

Set `MODE` in `src/config.py`.

## Requirements

- Python 3.11+
- `pygame`, `pyserial` (`pip install pygame pyserial`)
- PS5 DualSense controller (USB)
- Arduino with H-bridge wired to pins 4–7 (arduino mode only)

## Running

```bash
python3 -m src         # run (mode set in config.py)
python3 -m pytest tests/ -v   # tests
```

## Controls (PS5 DualSense)

| Input | Action |
|---|---|
| R2 | Throttle |
| Left stick | Steering |
| Circle | Toggle emergency stop |

## Project structure

```
src/
  main.py          main loop
  config.py        tunable constants
  input.py         PS5 controller + deadzone
  control.py       smoother, e-stop, watchdog, controller
  simulation/      physics model + pygame renderer (sim only)
  vehicle/         simulated vehicle, Arduino vehicle, serial protocol
arduino/
  motor_controller/motor_controller.ino
tests/
  test_control.py, test_physics.py, test_protocol.py
```

## Serial protocol

```
M <left> <right>\n    motor PWM (-255..255)
E\n                   emergency stop
P\n                   heartbeat
S <front> <left> <right> <heading>\n   sensor data (Arduino → Python)
```

Baud rate: 115200. Arduino has a 500ms watchdog — stops motors if no command received.
