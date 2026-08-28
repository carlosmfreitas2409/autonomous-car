# Master Arduino — Drive (4WD) + Steering

Firmware for the master microcontroller on the Kigali autonomous car. Owns the four drive motors (one BTS7960 per wheel) and the Ackermann-linkage steering motor (a fifth BTS7960 driving an electric-window motor whose position is read by a 10k linear potentiometer on an analog pin). The Python host in `../../src/arduino.py` already speaks the wire protocol.

Also acts as the **CAN gateway** to the lights Nano: the host sends `L` over USB serial, the master packs it into a CAN frame and transmits it on the MCP2515 bus (`firmware/lights/` is the receiver). One USB cable to the computer; the lights board is reachable only via CAN.

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

| Signal | Pin | Timer |
|---|---|---|
| FL drive RPWM / LPWM | 5 / 2 | T3 |
| FR drive RPWM / LPWM | 6 / 7 | T4 |
| RL drive RPWM / LPWM | 44 / 45 | T5 |
| RR drive RPWM / LPWM | 46 / 3 | T5 / T3 |
| Steering RPWM / LPWM | 11 / 12 | T1 |
| Steering pot wiper | A0 | ADC |
| MCP2515 SPI (CS / MISO / MOSI / SCK) | 53 / 50 / 51 / 52 | — |
| Status LED | 13 | — |

All BTS7960 `R_EN` and `L_EN` pins are tied directly to the Arduino's 5V rail — bridges are always armed. "Estop" means PWM = 0 on every drive and steering output (software-only). The BTS7960's internal pull-downs on RPWM/LPWM keep the bridges quiet during boot, before the firmware initializes its pins.

## Serial protocol

USB Serial @ 115200 baud, newline-terminated ASCII.

### Inbound (host → master)

| Verb | Args | Effect |
|---|---|---|
| `M` | `<throttle 0..255> <steering -255..+255>` | Set drive throttle and steering target. Resets watchdog. First non-zero `M` clears latched estop. |
| `E` | — | Emergency stop. Latched. All PWMs forced to 0. Cleared only by a subsequent non-zero `M`. |
| `H` | — | Heartbeat — resets watchdog without changing setpoints. |
| `L` | `<left> <right> <drl> <low> <brake> <reverse> <logo> <projector> [strip]` | Pack the 8 booleans into a bitmask and transmit as CAN ID `0x100` to the lights Nano. `strip` is the optional LED strip effect id (`0` off, `1` red, `2` green, `3` blue, `4` strobe, `5` rainbow); omitted or out of range means off. Does **not** affect the drive watchdog. |

Malformed lines produce `ERR <verb>` on the same serial. A CAN transmit failure (no MCP2515 / bus off) produces `ERR CAN`.

### Outbound (master → host, optional)

- `RDY car-master v1` — boot banner.
- `ERR ...` — parse errors and fault notifications.
- `T <angle_raw> <target_raw> <throttle> <steer_pwm> <state> <wd_ms>` — telemetry at 10 Hz, behind `TELEMETRY_ENABLED` compile flag. `state` ∈ `R W E F`.

## Behavior

- **Boot state**: ESTOP, all PWMs at 0. The car cannot move between power-on and the first deliberate non-zero `M` from the host.
- **Watchdog**: 500 ms. With no command, throttle ramps to 0 and steering target freezes at the current measured angle (wheels stop where they are).
- **Drive slew**: up at 4 PWM/tick (200 Hz → 0→255 in ~320 ms); down at 32 PWM/tick (~40 ms).
- **Steering**: 200 Hz P+D loop on the pot's 0–1023 ADC reading (derivative on measurement). Starting gains `Kp = 1.2`, `Kd = 0.12`, `±3` raw-unit deadband. Min-move PWM = 120 to break static friction. The reading is median-of-9 filtered to reject ADC spikes. Soft limits at the configured max angles (refuse to drive into a stop, allow recovery in the opposite direction).
- **Steering-pot fault** (the pot reads a rail — 0 or ≥1023 — at boot, meaning a disconnected wiper or outer leg) → permanent FAULT state (`ERR STEER`), status LED blinks fast, all PWMs forced to 0.

## Steering calibration

Calibration is **edit-and-flash**, not runtime. Three constants in `src/config.h`, all in the pot's 0–1023 ADC range:

```cpp
constexpr uint16_t STEER_CENTER_RAW    = 512;   // wheels straight
constexpr uint16_t STEER_MAX_LEFT_RAW  = 620;   // safe max-left (NOT the hard stop)
constexpr uint16_t STEER_MAX_RIGHT_RAW = 404;   // safe max-right
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
