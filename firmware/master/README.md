# Master Arduino — Drive (4WD) + Steering

Firmware for the master microcontroller on the Kigali autonomous car. Owns the four drive motors (one BTS7960 per wheel) and the Ackermann-linkage steering motor (a fifth BTS7960 driving an electric-window motor whose position is read by a 10k linear potentiometer on an analog pin). The Python host in `../../src/arduino.py` already speaks the wire protocol.

Also acts as the **CAN gateway** to the lights Nano: the host sends `L` over USB serial, the master packs it into a CAN frame and transmits it on the MCP2515 bus (`firmware/lights/` is the receiver). One USB cable to the computer; the lights board is reachable only via CAN. The cargo node (`firmware/cargo/`) reaches the bus through the sonar node: its status arrives as CAN `0x180` and goes to the host as a `C` line, and the host's `K` goes out as CAN `0x181`.

## Build & flash

PlatformIO:

```sh
pio run -d firmware/master                      # compile
pio run -d firmware/master -t upload            # flash via USB
pio device monitor -d firmware/master -b 115200 # serial monitor
```

After clone or a new worktree, regenerate the clangd compile database once:

```sh
pio run -d firmware/master -t compiledb
```

Telemetry output is gated by a compile flag — uncomment `-DTELEMETRY_ENABLED=1` in `platformio.ini` to enable.

## Hardware

- **Board**: Arduino Mega 2560.
- **Drive**: 4× BTS7960 H-bridges (one per wheel: FL, FR, RL, RR), forward-only in firmware today (LPWMs are wired to PWM-capable pins so reverse is a software-only addition).
- **Steering**: 1× BTS7960 driving an electric-window motor. Bidirectional. Closed-loop position control via a 10k linear potentiometer on the ADC (wiper → A0, outer legs → 5V / GND).
- **PWM**: Timer 1, 3, 4, 5 set to phase-correct 8-bit PWM, prescaler 1 → ~31 kHz (above audible). Timer 0 untouched (`millis()`). Timer 2 untouched (`tone()` available).
- **Power**: Use a separate 5V BEC for the Mega; do not run logic from USB once motors are connected. Bulk cap (≥10 mF) across H-bridge V_in. Common-star ground.

### Pinout

| Signal | RPWM / LPWM | Timer | R_EN / L_EN |
|---|---|---|---|
| FL drive | 5 / 6 | T3 / T4 | 24 / 25 |
| FR drive | 7 / 8 | T4 | 26 / 27 |
| RL drive | 3 / 4 | T3 / T0 | 22 / 23 |
| RR drive | 9 / 10 | T2 | 28 / 29 |
| Steering | 11 / 12 | T1 | 30 / 31 |

| Signal | Pin |
|---|---|
| Steering pot wiper | A11 |
| MCP2515 SPI (CS / MISO / MOSI / SCK) | 53 / 50 / 51 / 52 |
| Reverse buzzer | 46 (T5) |
| Battery divider | A10 |
| Battery LED bar (bottom → top) | 43, 41, 39, 37, 35 (2 segments), 33 (2 segments) |
| Status LED | 13 |

`R_EN` and `L_EN` are driven by the firmware, not strapped to 5V — `motors::disable()` and
`steering::disable()` drop them, so an estop removes the bridge enables as well as zeroing PWM. The
BTS7960's internal pull-downs on RPWM/LPWM keep the bridges quiet during boot, before the firmware
initializes its pins.

## Serial protocol

USB Serial @ 115200 baud, newline-terminated ASCII.

### Inbound (host → master)

| Verb | Args | Effect |
|---|---|---|
| `M` | `<throttle 0..255> <steering -255..+255>` | Set drive throttle and steering target. Resets watchdog. First non-zero `M` clears latched estop. |
| `E` | — | Emergency stop. Latched. All PWMs forced to 0. Cleared only by a subsequent non-zero `M`. |
| `H` | — | Heartbeat — resets watchdog without changing setpoints. |
| `L` | `<left> <right> <drl> <low> <brake> <reverse> <logo> <projector> [strip]` | Pack the 8 booleans into a bitmask and transmit as CAN ID `0x100` to the lights Nano. `strip` is the optional LED strip effect id (`0` off, `1` red, `2` green, `3` blue, `4` strobe, `5` rainbow, `6` cyan, `7` red pulse, `8` police); omitted or out of range means off. Does **not** affect the drive watchdog. |
| `K` | `<unlock 0\|1>` | Transmit CAN ID `0x181` (DLC 1, bit 0 unlock) to the cargo node, via the sonar node's bridge: unlock (a lease the host renews every frame; the node locks 300 ms after the last `K 1`) or lock. Always sent as *lock* while the drive throttle is not 0. Does **not** affect the drive watchdog. The host only sends it once `C` lines arrive. |

Malformed lines produce `ERR <verb>` on the same serial. A CAN transmit failure (no MCP2515 / bus off) produces `ERR CAN`.

### Outbound (master → host, optional)

- `RDY car-master v1` — boot banner.
- `ERR ...` — parse errors and fault notifications.
- `T <angle_raw> <target_raw> <throttle> <steer_pwm> <state> <wd_ms>` — telemetry at 20 Hz, behind `TELEMETRY_ENABLED` compile flag. `state` ∈ `R W E F`.
- `B <millivolts> <percent>` — the battery gauge's smoothed reading (the same one the LED bar shows), once a second, behind `TELEMETRY_ENABLED`. Reads low under motor load.
- `S <state> <nearest_cm> <d0..d2>` — obstacle state at 20 Hz (the sonar node's rate), behind `TELEMETRY_ENABLED`. `state` ∈ `C` clear, `S` slowing, `B` blocked, `X` stale. Distances are cm, with `255` = nothing in range. The host uses the three distances and `X` to slow/stop in autonomous mode.
- `C <grams> <stable> <lid> <lock> <fault>`: the cargo node's status, one line per CAN `0x180` frame (10 Hz while it is connected, none without it). `lid` 1 = closed, `lock` 1 = coil powered (unlocked), `fault` 1 = the scale is not answering.

## Behavior

- **Boot state**: ESTOP, all PWMs at 0. The car cannot move between power-on and the first deliberate non-zero `M` from the host.
- **Watchdog**: 500 ms. With no command, throttle ramps to 0 and steering target freezes at the current measured angle (wheels stop where they are).
- **Drive slew**: up at 4 PWM/tick (200 Hz → 0→255 in ~320 ms); down at 32 PWM/tick (~40 ms).
- **Steering**: 200 Hz P+D loop on the pot's 0–1023 ADC reading (derivative on measurement). Gains `Kp = 2.2`, `Kd = 0.12`, `±3` raw-unit deadband. Min-move PWM = 150 to break static friction, capped at 210. The reading is median-of-9 filtered to reject ADC spikes. Soft limits at the configured max angles (refuse to drive into a stop, allow recovery in the opposite direction).
- **Steering-pot fault** (the pot reads a rail — 0 or ≥1023 — at boot, meaning a disconnected wiper or outer leg) → permanent FAULT state (`ERR STEER`), status LED blinks fast, all PWMs forced to 0.

### Battery gauge

The 12 V lead-acid battery (Heliar HE60DD) feeds a resistor divider into A10. `battery.cpp` samples it every 200 ms, converts the raw ADC count to battery volts with `BATT_VOLTS_PER_COUNT`, maps volts to charge through a resting-voltage lead-acid curve (11.80 V = 0 %, 12.70 V = 100 %), and smooths with a ~10 s moving average. The 8-segment bar is driven by 6 pins (35 and 33 each drive two segments) and lights at 15 / 30 / 50 / 65 / 80 / 97 %, with 3 % hysteresis so it can't flicker on a boundary. Below 15 % the bar is dark.

Voltage sags under motor load, so the bar reads low while driving — the at-rest reading is the accurate one.

**Calibration**: the ADC reference is the Mega's 5 V rail, which isn't exactly 5.00 V (≈ 5.17 V on the car), so calibrate against the raw count, not the A10 voltage. With the car powered as it normally runs and the motors stopped, read the battery with a multimeter and the raw A10 count (`analogRead(A10)`) at the same moment, then set `BATT_VOLTS_PER_COUNT = V_battery / count` in `config.h`. Shipped value: 12.56 V / 562.

### Obstacle avoidance

Front distance data arrives over CAN from the sonar node (`firmware/sonar/`, ID `0x080` at 20 Hz).
`obstacle.cpp` classifies the nearest reading into a state (`CLEAR`/`SLOWING`/`BLOCKED`/`STALE`) and
reports it on the `S` telemetry line together with the three per-sensor distances. It does **not**
touch the motors.

Obstacle avoidance is enforced by the host (`src/obstacle.py`, applied by `src/input/autonomous.py`)
and runs in **autonomous mode only** — manual (PS5) driving is never governed by the sonar. The host
ignores the master's `B`/`S` states (they count every sensor, including echoes beside the lane) and
works from the three distances: the centre sensor slows the car from 150 cm, the side sensors (which
point outward and see things beside the track) from 100 cm; the centre sensor stops it at 75 cm and the side sensors at 60 cm (they point outward at the poles beside
the track), and it resumes only after every sensor reads 15 cm past its stop distance for 0.5 s. While stopped the steering is held and the hazards flash.
`TELEMETRY_ENABLED` must stay on: without `S` lines the host has no sonar data and does not gate.

- **Fail-safe**: no frame for 300 ms means *blocked*, never *clear*.

## Steering calibration

Calibration is **edit-and-flash**, not runtime. Three constants in `src/config.h`, all in the pot's 0–1023 ADC range:

```cpp
constexpr uint16_t STEER_CENTER_RAW    = 539;   // wheels straight
constexpr uint16_t STEER_MAX_LEFT_RAW  = 993;   // safe max-left (NOT the hard stop)
constexpr uint16_t STEER_MAX_RIGHT_RAW = 15;    // safe max-right
```

The shipped values are conservative placeholders — you **must** capture real ones for your linkage. The dedicated tool `tools/calibration_pot/calibration_pot.ino` makes this easy (it pulses the motor by serial command and prints a ready-to-paste snippet):

1. **Disconnect motor power** (the steering BTS7960's V_in) if you want to move the linkage purely by hand; the calibration tool can also nudge it with short `r`/`l` pulses.
2. Flash `tools/calibration_pot/calibration_pot.ino`, open `pio device monitor -b 115200`.
3. Move the steering and capture:
   - Wheels straight → `c` → `STEER_CENTER_RAW`.
   - A *safe* max-left (well short of the hard stop, since the linkage locks up before it) → `ml` → `STEER_MAX_LEFT_RAW`.
   - A safe max-right → `mr` → `STEER_MAX_RIGHT_RAW`.
4. Type `p` to print the `config.h` snippet, paste the three values into `config.h`.
5. Flash the master firmware. During the bench step-response test, confirm a left command moves the wheels left; if not, swap `STEER_MAX_LEFT_RAW`/`STEER_MAX_RIGHT_RAW` or flip `STEER_INVERT_PWM`.

The values stay correct until the pot or linkage are physically moved. Recalibration means repeating steps 1–5.

## Bench verification (in order)

1. Open the monitor → `RDY car-master v1`.
2. Sensor only: rotate the steering shaft by hand (motor power off), enable telemetry, watch `angle_raw` move smoothly across the calibrated range within 0..1023 (no spikes — the read is median-filtered).
3. Wheels off the ground: `M 60 0` → all four wheels spin equally; `E` → all wheels stop (PWM = 0 on every bridge); `M 60 0` again → still stopped (latched).
4. `M 255 0` from rest → wheels reach full PWM in ~300 ms.
5. Stop sending → wheels stop within 500 ms; `H` alone does not re-spin them.
6. PID step response: `M 0 -255` → `M 0 255` → `M 0 0`. Tune `Kp`/`Kd` if it rings or overshoots.
7. Manually push past the configured max angle while commanding hold-center → PWM cuts in the limit direction, holds against in the other.
8. Per-wheel current sanity (clamp ammeter on each motor lead during a 0→full ramp): all four wheels within ~20% of each other.

### Obstacle reporting (sonar node powered, `TELEMETRY_ENABLED`)

The firmware only reports; the host acts on it. These check the `S` line is correct.

9. Hand at ~80 cm from the array → `S` state goes `S` (slowing); at ~60 cm → `B` (blocked). Withdraw
   → back to `C`. `nearest_cm` tracks a tape measure to ±2 cm.
10. Unplug the sonar node → `S` reports `X` (stale) within 300 ms. Reconnect → recovers.
11. In the host GUI, switch to autonomous and confirm the car slows/stops as the reading closes in, the
    wheels stay still and the hazards flash while it waits; switch to manual (PS5) and confirm it does
    **not**.
12. An obstacle in front of a front corner, seen only by a side sensor: the car slows from 100 cm and
    stops at 60 cm (the centre sensor stops at 75 cm).
