The simplest way to think about it: **two sets of knobs, two sets of symptoms.** What's broken tells you which side to touch.

## Decision flowchart

| Symptom | Tune where |
|---|---|
| Wheels don't actually reach the angle Python asked for, or take forever to settle, or wobble around it | **Arduino** (steering PID) |
| Wheels reach the angle fine, but the *car* still wanders off the lane / oscillates / crosses the line | **Python** (lane-following PID) |
| Car drives too fast in turns, or too slow on straights | **Python** (speed) |
| Wheels jolt the chassis when you accelerate from a stop | **Arduino** (slew) |

If unsure: **disconnect the camera** and command angles directly via `M 0 +127`, `M 0 -127`. If the wheels look good (snap to angle, hold, no buzz), Arduino is fine — any remaining bad behavior is Python's.

---

## Arduino `config.h`

### Definitely tune (one-time, on the bench)

| Constant | When | How |
|---|---|---|
| `STEER_CENTER_RAW`, `STEER_MAX_LEFT_RAW`, `STEER_MAX_RIGHT_RAW` | Once, after you mount the steering pot. Again only if you re-mount the pot or change the linkage. | Use `tools/calibration_pot/calibration_pot.ino`: move the steering, capture `c`/`ml`/`mr`, type `p` to print the snippet. Values are in the pot's 0–1023 ADC range. |

### Probably tune (bench, by feel during step-response tests)

| Constant | What you observe | What to do |
|---|---|---|
| `STEER_KP` (2.2) | Wheels move sluggishly toward target, undershoot | Raise. Common range 0.8–3.0. |
| | Wheels overshoot and oscillate around target | Lower. |
| `STEER_KD` (0.12) | Overshoots and rings before settling | Raise. Common range 0.05–0.4. |
| | Movement feels jerky/hesitant | Lower. |
| `STEER_MIN_MOVE_PWM` (150) | Wheel hums near target but doesn't actually move | Raise (130, 145...). |
| | Wheel "kicks" when reaching target | Lower. |
| `STEER_DEADBAND_UNITS` (3) | Wheel twitches when holding still | Raise (5, 8). |
| | Final position is sloppy/imprecise | Lower. |

Tuning workflow: send `M 0 -200`, wait, send `M 0 +200`, wait, send `M 0 0`. Watch the pot angle (`angle_raw`, 0–1023) in telemetry. Adjust one constant at a time.

> **Interaction with the steering failsafes.** Lowering `STEER_MIN_MOVE_PWM` or
> `STEER_MAX_PWM` makes the actuator slower, which eats into the margin of the
> no-motion guard (`ERR STEER S`). If you drop either meaningfully, re-time how
> long a healthy sweep takes to cover 31 counts and raise
> `STEER_STALL_TIMEOUT_MS` to ~3× that. The plausibility band and stall delta are
> derived from `STEER_CENTER_RAW`/`MAX_LEFT`/`MAX_RIGHT`, so a bad calibration
> also means bad guard thresholds — see *Steering feedback failsafes* in
> `firmware/master/README.md`.

### Maybe tune

| Constant | When |
|---|---|
| `STEER_MAX_PWM` (210) | Lower it if the steering feels too violent. Raise (toward 255) only if it's noticeably weak under real cornering load. |
| `DRIVE_SLEW_UP_PER_TICK` (4) | Lower if the car launches too aggressively or browns out. Raise if you want snappier acceleration. |
| `DRIVE_SLEW_DOWN_PER_TICK` (32) | Almost never. Lower it only if the deceleration is too abrupt for your taste. |

### Won't touch

`TELEMETRY_PERIOD_MS` — only matters when telemetry is enabled, and 100 ms (10 Hz) is fine.

---

## Python `src/config.py`

### Definitely tune (on the track)

| Constant | What you observe | What to do |
|---|---|---|
| `kp` (0.01) | Car drifts off the lane, doesn't correct enough | Raise (0.015, 0.02). |
| | Car weaves left-right rapidly across the lane | Lower (0.007, 0.005). |
| `kd` (0.005) | Car oscillates / overshoots back across the lane | Raise. |
| | Car feels jerky | Lower. |
| `base_speed` (0.4) | Whole car too slow / too fast for the track | Adjust until it feels right. |
| `min_speed` (0.2) | Car loses too much speed in turns | Raise. Stalls in tight curves | Raise more. |

### Probably leave alone

| Constant | When to touch |
|---|---|
| `ki` (0.0) | Only if the car has a *consistent* bias — always slightly off-center on long straights. Then a tiny value (0.0001) for a long time. Most cars don't need it. |

---

## The overall mental model

1. **Bench-tune the Arduino first.** Make the steering hit any commanded angle cleanly, with no oscillation or hum. Don't touch Python until this is done.
2. **Drive on the track. Tune Python.** With the inner loop solid, you only have one PID's worth of behavior to chase.
3. **If Python tuning feels impossible**, suspect the Arduino. The outer loop can't compensate for an unreliable inner loop.

The order matters. Always tune **inside-out** — that's the whole reason cascade control exists.


---


Yes — `STEER_KP` and `STEER_KD` are the main two. There are three more that can produce the same symptoms:

| Symptom | Likely culprit |
|---|---|
| Wheels are slow to reach the target, undershoot | `STEER_KP` (too low) |
| Wheels overshoot and oscillate | `STEER_KP` (too high) or `STEER_KD` (too low) |
| Movement looks jerky/hesitant | `STEER_KD` (too high) |
| Motor hums but doesn't move on small corrections | `STEER_MIN_MOVE_PWM` (too low) — can't break static friction |
| Wheel twitches when "at rest" | `STEER_DEADBAND_UNITS` (too low) |
| Wheels feel weak under real load | `STEER_MAX_PWM` (too low) |

So when tuning, **always start with Kp and Kd**. Touch the others only if their specific symptom shows up.

## How to test without Python

You don't need Python at all. The Arduino accepts the `M` command directly over USB. Use the serial monitor as your "host."

### Setup (one-time)

1. **Enable telemetry** in `platformio.ini` — uncomment `-DTELEMETRY_ENABLED=1`. You need to see the angle trace.
2. **Bump the watchdog** temporarily so it doesn't kill your tests. In `config.h`, change `WATCHDOG_TIMEOUT_MS` from `500` to something big like `60000` (1 minute). Revert after you're done tuning.
3. **Build and flash**: `pio run -t upload`.
4. **Wheels off the ground** (jack stands).
5. **Open the monitor**: `pio device monitor -b 115200`.

### Step-response test

Type these commands one at a time, with Enter:

```
M 0 0          ← arms the steering (clears estop, target = current angle)
M 0 -200       ← step left
                  watch the T lines — angle should swing toward max-left
M 0 +200       ← step right
                  watch the angle swing the other way
M 0 0          ← back to center
```

Each step is one "experiment." You're watching how the angle gets from where it was to where you asked it to go.

### Reading the telemetry

Telemetry lines look like:

```
T <angle_raw> <target_raw> <throttle> <steer_pwm> <state> <wd_ms>
```

The two numbers that matter for tuning are `angle_raw` (where the wheel actually is) and `target_raw` (where you asked it to be). Watch the angle approach the target.

Decision tree based on what you see:

- **Angle reaches target smoothly, no overshoot, settles fast** → Done. Don't touch.
- **Angle never quite reaches target** → Raise `STEER_KP` slightly (e.g. 2.2 → 2.6).
- **Angle overshoots target, comes back, overshoots again, oscillates** → Lower `STEER_KP` (2.2 → 1.8) **or** raise `STEER_KD` (0.12 → 0.2).
- **Angle approaches but stops a bit short, then jumps the last few units in a "kick"** → Lower `STEER_MIN_MOVE_PWM` (150 → 130) or raise `STEER_DEADBAND_UNITS` (3 → 5).
- **Motor hums, angle doesn't move at all on small commands** → Raise `STEER_MIN_MOVE_PWM` (150 → 175).
- **Angle wiggles when wheel is supposed to be still** → Raise `STEER_DEADBAND_UNITS` (3 → 8).

### The iteration loop

This is the tedious part of the job:

```
1. Edit one constant in config.h.
2. pio run -t upload   (~5 seconds to rebuild)
3. M 0 -200, watch angle
4. M 0 +200, watch angle
5. Decide based on the table above
6. Go to 1
```

Change **one constant at a time** so you can tell which knob did what. Don't be precious — bigger steps converge faster than tiny tweaks. Halve or double, not 0.6 → 0.62.

A clean tune usually takes 10–20 iterations.

### When you're done

- Revert `WATCHDOG_TIMEOUT_MS` back to 500.
- Comment `-DTELEMETRY_ENABLED=1` back out (optional — leaving it on costs ~500 bytes flash and a tiny bit of CPU time).
- Reflash. The Python side never had to be running for any of this.

### A useful sanity check

Before you touch any gain, do **one** step (`M 0 -200`) and just confirm the wheel **moves the right way and roughly the right amount**. If it goes the wrong direction, either your `STEER_MAX_LEFT_RAW` and `STEER_MAX_RIGHT_RAW` are swapped or `STEER_INVERT_PWM` is wrong (both depend on how you mounted the pot and wired the motor). Fix that before tuning anything else.