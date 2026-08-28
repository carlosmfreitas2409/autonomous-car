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
| `L` | `<left> <right> <drl> <low> <brake> <reverse> <logo> <projector> [strip]` | Pack the 8 booleans into a bitmask and transmit as CAN ID `0x100` to the lights Nano. `strip` is the optional LED strip effect id (`0` off, `1` red, `2` green, `3` blue, `4` strobe, `5` rainbow, `6` cyan, `7` red pulse, `8` red strobe); omitted or out of range means off. Does **not** affect the drive watchdog. **Ignored entirely while in FAULT** — the master drives the lights itself then. |

Malformed lines produce `ERR <verb>` on the same serial. A CAN transmit failure (no MCP2515 / bus off) produces `ERR CAN`.

### Outbound (master → host, optional)

- `RDY car-master v1` — boot banner.
- `ERR ...` — parse errors and fault notifications.
- `T <angle_raw> <target_raw> <throttle> <steer_pwm> <state> <wd_ms> <steer_fault>` — telemetry at 10 Hz, behind `TELEMETRY_ENABLED` compile flag. `state` ∈ `R W E F`; `steer_fault` ∈ `- D R S` (see *Steering feedback failsafes*).
- `S <state> <nearest_cm> <d0..d2>` — obstacle state at 10 Hz, behind `TELEMETRY_ENABLED`. `state` ∈ `C` clear, `S` slowing, `B` blocked, `X` stale. Distances are cm, with `255` = nothing in range. The host uses `state` and `nearest_cm` to slow/stop in autonomous mode.

## Behavior

- **Boot state**: ESTOP, all PWMs at 0. The car cannot move between power-on and the first deliberate non-zero `M` from the host.
- **Watchdog**: 500 ms. With no command, throttle ramps to 0 and steering target freezes at the current measured angle (wheels stop where they are).
- **Drive slew**: up at 4 PWM/tick (200 Hz → 0→255 in ~320 ms); down at 32 PWM/tick (~40 ms).
- **Steering**: 200 Hz P+D loop on the pot's 0–1023 ADC reading (derivative on measurement). Gains `Kp = 2.2`, `Kd = 0.12`, `±3` raw-unit deadband. Min-move PWM = 150 to break static friction, capped at 210. The reading is median-of-9 filtered to reject ADC spikes. Soft limits at the configured max angles (refuse to drive into a stop, allow recovery in the opposite direction).
- **Steering feedback fault** → permanent FAULT (`ERR STEER <code>`), status LED blinks fast, buzzer holds a solid tone, hazards blink and the LED strip goes hard red strobe, all PWMs forced to 0 and every `R_EN`/`L_EN` driven LOW. See the section below.

## Steering feedback failsafes

The steering loop is only as safe as the pot behind it. With a dead sensor the
error never converges and `STEER_MIN_MOVE_PWM = 150` guarantees the motor pushes
at full force into whatever it meets — that is how a steering assembly gets
broken. Three independent guards bound that, each catching what the others
cannot. Any of them latches FAULT; recovery is a power cycle, which re-runs the
checks from scratch.

| Code | Guard | Catches | Detects within |
|---|---|---|---|
| `D` | **Connectivity probe** | Pot unplugged / wiper open | boot, and every disabled→enabled edge |
| `R` | **Plausibility band** | Railed or out-of-travel reading | ~100 ms, any time |
| `S` | **No motion under drive** | Stuck-but-believable pot, snapped linkage, jammed rack, dead bridge, inverted `STEER_INVERT_PWM` | ~250 ms of driving |

**`D` — connectivity probe.** A disconnected AVR analog pin does *not* reliably
read a rail; it floats and holds charge from the previous conversion, so it
usually returns a perfectly plausible mid-scale value. The probe toggles the
internal pull-up (20–50 kΩ) and compares: a connected wiper is a low-impedance
node (≤ ~2.5 kΩ on a 10 kΩ pot) and shifts only ~40–60 counts, a floating pin
snaps to ~1023. A shift ≥ 150 counts means disconnected. It blocks for ~6 ms, so
it runs only when the loop is open — never inside `update()`.

**`R` — plausibility band.** Derived from the calibration constants, not
configured: the calibrated travel widened by a third of the span at each end.
With `300/526/800` that is `[133, 966]`. The headroom is deliberately generous —
this guard's job is to catch a *railed* reading fast without waiting for a
steering command, and the calibrated limits stop short of the real mechanical
stops, so shoving the wheels there by hand must not look like a wiring fault.
The reading must sit outside it for
`STEER_SENSOR_BAD_MS` (100 ms = 20 ticks) before faulting, so one bad conversion
cannot trip it. Runs even while steering is disabled, so a wire that falls off
while the car is parked in E-STOP is caught before the next resume.

**`S` — no motion under drive.** If we have been pushing one direction at
≥ `STEER_MIN_MOVE_PWM` for `STEER_STALL_TIMEOUT_MS` (250 ms) and the measurement
has not moved `stall_min_delta` counts, the feedback path is not real. The
threshold is derived, not configured: `max(2 × deadband, span / 16)` → 31 counts
at the current calibration, so it rescales if the pot is recalibrated. Only
armed while the bridge is actually enabled. Note the soft limits already zero the
output at a configured stop, which re-baselines the detector — parking against a
soft limit never trips it.

`STEER_STALL_TIMEOUT_MS` is the only number here that may need adjusting, and
only if you get false alarms — see *Failsafe verification* below. Nothing needs
recalibrating.

### Optional hardware belt-and-braces

A ~100 kΩ resistor from the pot wiper (A11) to GND makes a wiper disconnect drag
the ADC to 0 immediately, which guard `R` catches without relying on the pull-up
probe. It shifts a midscale reading by ~12 counts, so **recalibrate afterwards**.
The firmware does not depend on it.

### Obstacle avoidance

Front distance data arrives over CAN from the sonar node (`firmware/sonar/`, ID `0x080` at 20 Hz).
`obstacle.cpp` classifies the nearest reading into a state (`CLEAR`/`SLOWING`/`BLOCKED`/`STALE`) and
reports it on the `S` telemetry line. It does **not** touch the motors.

Obstacle avoidance is enforced by the host (`src/input/autonomous.py`) and runs in **autonomous mode
only** — manual (PS5) driving is never governed by the sonar. The host zeroes throttle at or below
30 cm and tapers up to full by 80 cm.

- **Fail-safe**: no frame for 300 ms means *blocked*, never *clear*.

## Steering calibration

Calibration is **edit-and-flash**, not runtime. Three constants in `src/config.h`, all in the pot's 0–1023 ADC range:

```cpp
constexpr uint16_t STEER_CENTER_RAW    = 526;   // wheels straight
constexpr uint16_t STEER_MAX_LEFT_RAW  = 800;   // safe max-left (NOT the hard stop)
constexpr uint16_t STEER_MAX_RIGHT_RAW = 300;   // safe max-right
```

These are captured values for the current linkage; recapture them if the pot or linkage move. Note the failsafe thresholds above are derived from these three numbers, so a bad calibration also means bad guard thresholds. The dedicated tool `tools/calibration_pot/calibration_pot.ino` makes this easy (it pulses the motor by serial command and prints a ready-to-paste snippet):

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

9. Hand at ~60 cm from the array → `S` state goes `S` (slowing); at ~30 cm → `B` (blocked). Withdraw
   → back to `C`. `nearest_cm` tracks a tape measure to ±2 cm.
10. Unplug the sonar node → `S` reports `X` (stale) within 300 ms. Reconnect → recovers.
11. In the host GUI, switch to autonomous and confirm the car slows/stops as the reading closes in;
    switch to manual (PS5) and confirm it does **not**.

## Failsafe verification

The car has **two separate motor systems**, and these tests treat them
differently:

- the **four drive motors**, which make the car roll — **unplug these**, so a
  test can't send the car across the room;
- the **one steering motor**, which turns the wheels left and right — **leave
  this plugged in**, it is what we are testing.

Put the car on blocks so all four wheels are off the ground.

**You do not have to turn anything by hand.** Every test below is done by
sending commands over USB; the steering motor does the turning. Open the serial
monitor from `firmware/master/`:

```
pio device monitor -b 115200
```

Commands you'll type: `M 0 60` means "no throttle, steer a little"; `M 0 255` is
"steer fully one way"; `M 0 -255` is fully the other way; `E` is emergency stop.

**A fault is obvious from across the room, no computer needed:**

- the **LED strip blinks hard red** (sharper than the slow red breathing that
  means e-stop);
- the **hazard lights flash**;
- the **buzzer holds one continuous tone**;
- the status LED on the Arduino blinks fast.

The master drives those lights itself rather than asking the laptop to, so a car
whose steering has broken still signals even if the laptop crashed or the USB
cable came out.

On the monitor you also get a line like `ERR STEER D`, where the last letter says
which guard tripped (`D` pot unplugged, `R` nonsense reading, `S` motor pushing
but nothing moving). If the Python app is running, it prints that line as a
warning too.

Run these in order.

1. **Everything normal.** Boot with the pot plugged in. You should see
   `RDY car-master v1` and no error. Send `M 0 60` — the wheels turn a little,
   no fault. This is your baseline; if it doesn't work, stop here.
2. **Boot with the pot unplugged.** This is the exact situation that broke the
   linkage. Unplug the pot connector, power on. Expect `ERR STEER D`
   immediately, red strobing strip, blinking hazards, continuous buzzer, and the
   steering motor never twitches. Leave it sitting in fault for a full minute and
   confirm the lights keep going — that checks the master is repeating its CAN
   frame rather than letting the lights board's 1 s watchdog blank everything.
3. **Pot falls off mid-run.** Plug the pot back in, power cycle, send `M 0 60`
   so steering is live, then pull the pot connector out. The steering motor
   should stop within about a quarter of a second. The letter may be `D` or `R`
   — either is correct.
4. **Pot falls off while stopped.** Power cycle, send `E` to emergency-stop,
   unplug the pot, then send `M 0 60` to resume. It should fault before the
   steering motor gets any power at all.
5. **Steering motor can't move.** Power cycle. Now unplug the **steering
   motor's** two power leads, but leave the pot plugged in. Send `M 0 255`. The
   firmware is commanding a turn and the pot reports nothing happening, so it
   should fault with `ERR STEER S` within about a quarter second. This is the
   same guard that would catch a snapped linkage or a seized rack — testing it
   this way means nobody has to physically fight the motor.
6. **Check for false alarms.** Plug the steering motor back in, plug the drive
   motors back in, put the car on the ground and drive it normally for at least
   five minutes. Include full turns both ways and — importantly — a few hard
   steering inputs while the car is **stopped with its weight on the wheels**,
   which is the slowest, heaviest case the steering motor ever sees. You should
   get no faults at all. If you do, see *If you get false alarms* below.
7. **Back to normal.** Power cycle with everything connected — clean boot, no
   fault, strip back under host control.
8. **Lights aren't hijacked when healthy.** With no fault, set the strip from
   the GUI and confirm it still obeys you. Then trigger a fault (repeat step 5)
   and confirm the GUI can no longer change it — the master holds the red strobe
   until you power cycle.

### If you get false alarms

Only one number can cause them: `STEER_STALL_TIMEOUT_MS` in `src/config.h`,
currently `250`. It is how long the firmware will keep pushing the steering
motor while seeing no movement before deciding something is wrong — a quarter of
a second.

If step 6 produces a false `ERR STEER S`, your steering motor is simply slower
than that under load. Raise it to `400`, reflash, and repeat step 6. Keep going
in steps of ~150 until it's quiet. Don't go past about `800` — beyond that the
guard stops being much protection.

Nothing else needs tuning, and **no recalibration is needed** — the other
thresholds are calculated from the calibration values you already captured.
