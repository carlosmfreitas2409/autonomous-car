# Master Arduino — Drive (4WD) + Steering

Firmware for the master microcontroller on the Kigali autonomous car. Owns the four drive motors (one BTS7960 per wheel) and the Ackermann-linkage steering motor (a fifth BTS7960 driving an electric-window motor whose position is read by an AS5600 magnetic angle sensor over I2C). The Python host in `../../src/arduino.py` already speaks the wire protocol.

A future slave Arduino will own the lights and talk to the master over `Serial1` (pins 18/19 reserved).

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
- **Steering**: 1× BTS7960 driving an electric-window motor. Bidirectional. Closed-loop position control via AS5600 (I2C, 0x36).
- **PWM**: Timer 1, 3, 4, 5 set to phase-correct 8-bit PWM, prescaler 1 → ~31 kHz (above audible). Timer 0 untouched (`millis()`). Timer 2 untouched (`tone()` available).
- **Power**: Use a separate 5V BEC for the Mega; do not run logic from USB once motors are connected. Bulk cap (≥10 mF) across H-bridge V_in. Common-star ground.

### Pinout

| Signal | Pin | Timer |
|---|---|---|
| FL drive RPWM / LPWM | 5 / 2 | T3 |
| FR drive RPWM / LPWM | 6 / 7 | T4 |
| RL drive RPWM / LPWM | 44 / 45 | T5 |
| RR drive RPWM / LPWM | 46 / 3 | T5 / T3 |
| Steering RPWM / LPWM | 11 / 12 | T1 |
| AS5600 SDA / SCL | 20 / 21 | I2C @ 400 kHz |
| Status LED | 13 | — |
| Reserved Serial1 (future slave) | 18 / 19 | — |

All BTS7960 `R_EN` and `L_EN` pins are tied directly to the Arduino's 5V rail — bridges are always armed. "Estop" means PWM = 0 on every drive and steering output (software-only). The BTS7960's internal pull-downs on RPWM/LPWM keep the bridges quiet during boot, before the firmware initializes its pins.

## Serial protocol

USB Serial @ 115200 baud, newline-terminated ASCII.

### Inbound (host → master)

| Verb | Args | Effect |
|---|---|---|
| `M` | `<throttle 0..255> <steering -255..+255>` | Set drive throttle and steering target. Resets watchdog. First non-zero `M` clears latched estop. |
| `E` | — | Emergency stop. Latched. All PWMs forced to 0. Cleared only by a subsequent non-zero `M`. |
| `H` | — | Heartbeat — resets watchdog without changing setpoints. |

Malformed lines produce `ERR <verb>` on the same serial.

### Outbound (master → host, optional)

- `RDY kigali-master v1` — boot banner.
- `ERR ...` — parse errors and fault notifications.
- `T <angle_raw> <target_raw> <throttle> <steer_pwm> <state> <wd_ms>` — telemetry at 10 Hz, behind `TELEMETRY_ENABLED` compile flag. `state` ∈ `R W E F`.

## Behavior

- **Boot state**: ESTOP, all PWMs at 0. The car cannot move between power-on and the first deliberate non-zero `M` from the host.
- **Watchdog**: 500 ms. With no command, throttle ramps to 0 and steering target freezes at the current measured angle (wheels stop where they are).
- **Drive slew**: up at 4 PWM/tick (200 Hz → 0→255 in ~320 ms); down at 32 PWM/tick (~40 ms).
- **Steering**: 200 Hz P+D loop. `Kp = 0.6`, `Kd = 0.02` (derivative on measurement). `±6` raw-unit deadband. Min-move PWM = 60 to break static friction. Soft limits at the configured max angles (refuse to drive into a stop, allow recovery in the opposite direction).
- **AS5600 fault** (no I2C ack or no magnet detected at boot, or the angle jumps wildly mid-run) → permanent FAULT state, status LED blinks fast, all PWMs forced to 0.

## Steering calibration

Calibration is **edit-and-flash**, not runtime. Three constants in `src/config.h`:

```cpp
constexpr uint16_t STEER_CENTER_RAW    = 2048;   // wheels straight
constexpr uint16_t STEER_MAX_LEFT_RAW  = 1536;   // safe max-left (NOT the hard stop)
constexpr uint16_t STEER_MAX_RIGHT_RAW = 2560;   // safe max-right
```

Defaults are conservative (`center = 2048, ±512` ≈ ±45°). To get the real values for your linkage:

1. **Disconnect motor power** (the steering BTS7960's V_in). The Mega + AS5600 still run on USB.
2. Build with telemetry on: uncomment `-DTELEMETRY_ENABLED=1` in `platformio.ini`, flash, open `pio device monitor`.
3. **Rotate the steering by hand**:
   - To wheels-straight → note the `angle_raw` value → that's `STEER_CENTER_RAW`.
   - To a *safe* max-left (well short of the hard stop, since the linkage locks up before it) → that's `STEER_MAX_LEFT_RAW`.
   - To a safe max-right → that's `STEER_MAX_RIGHT_RAW`.
4. Edit the three constants in `config.h`.
5. Re-comment the telemetry flag, reflash. Done.

The values stay correct until the magnet, sensor, or linkage are physically moved. Recalibration means repeating steps 1–5.

## Bench verification (in order)

1. Open the monitor → `RDY kigali-master v1`.
2. Sensor only: rotate the steering shaft by hand (motor power off), enable telemetry, watch `angle_raw` wrap cleanly 0..4095.
3. Wheels off the ground: `M 60 0` → all four wheels spin equally; `E` → all wheels stop (PWM = 0 on every bridge); `M 60 0` again → still stopped (latched).
4. `M 255 0` from rest → wheels reach full PWM in ~300 ms.
5. Stop sending → wheels stop within 500 ms; `H` alone does not re-spin them.
6. PID step response: `M 0 -255` → `M 0 255` → `M 0 0`. Tune `Kp`/`Kd` if it rings or overshoots.
7. Manually push past the configured max angle while commanding hold-center → PWM cuts in the limit direction, holds against in the other.
8. Per-wheel current sanity (clamp ammeter on each motor lead during a 0→full ramp): all four wheels within ~20% of each other.
