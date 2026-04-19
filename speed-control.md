# Speed Control

The PID controller only handles **steering** (lateral correction). Speed/throttle is a separate concern that needs its own control strategy.

---

## Option B: Adaptive Speed (no sensor needed)

Map throttle directly as a function of steering output. The car slows down when it needs to turn hard and speeds up on straights.

### How it works

```
steering ≈ 0  →  going straight  →  high speed
steering ≈ 1  →  sharp curve     →  low speed
```

### Implementation

```python
base_speed = 0.6    # cruise speed on straights (0.0–1.0)
min_speed  = 0.3    # floor speed in tight curves

speed = base_speed - abs(steering) * (base_speed - min_speed)
```

`abs(steering)` ranges from 0 to 1, so `speed` linearly interpolates between `base_speed` and `min_speed`.

### Tuning

| Parameter    | Effect                                                        |
|-------------|---------------------------------------------------------------|
| `base_speed` | Higher = faster on straights, but PID must keep up            |
| `min_speed`  | Lower = slower in curves, safer but can lose momentum         |

Start conservative (`base_speed=0.4`, `min_speed=0.2`) and increase once the PID is well-tuned at that speed range.

### Pros

- Simple, no extra hardware
- Works well on flat tracks with consistent surfaces
- Easy to tune alongside the steering PID

### Cons

- Open-loop: you set a throttle value but don't *verify* the actual speed
- If the battery drains, the same throttle produces less speed — no compensation
- On inclines, the car slows down without the system knowing

### When to use

- Flat tracks, consistent surfaces
- Early prototyping and competitions where simplicity wins
- When you don't have an encoder yet

---

## Option C: Two PIDs (encoder required)

Use a second PID controller for speed, with real velocity feedback from a wheel encoder. This is closed-loop: the system measures actual speed and adjusts throttle to match a target.

### Hardware needed

A **rotary encoder** on a wheel or motor shaft. Common options:

- Optical encoder disc (cheap, DIY-friendly)
- Hall-effect sensor on the motor (many brushless ESCs expose this)
- IR slot sensor + slotted disc on the axle

The encoder produces pulses as the wheel turns. Count pulses over time to compute speed.

### Speed measurement

```python
TICKS_PER_REV = 20                  # depends on encoder resolution
WHEEL_CIRCUMFERENCE = 0.21          # meters, for a ~67mm diameter wheel

ticks = read_encoder_ticks()        # ticks since last read
revolutions = ticks / TICKS_PER_REV
actual_speed = revolutions * WHEEL_CIRCUMFERENCE / dt   # m/s
```

### Implementation

```python
# Steering PID (already exists)
steering_error = lane_center - frame_center
steering = steering_pid.update(steering_error, dt)

# Adaptive target speed (combines with Option B logic)
target_speed = base_speed - abs(steering) * (base_speed - min_speed)

# Speed PID (new)
speed_error = target_speed - actual_speed
throttle = speed_pid.update(speed_error, dt)

# Send to hardware
set_servo(steering)
set_motor(throttle)
```

### Speed PID tuning

The speed PID is generally easier to tune than steering because the system is more linear (throttle → speed is roughly proportional).

1. Start with `Kp` only. Set a `target_speed` and increase `Kp` until the car reaches it without oscillating.
2. Add `Kd` if the car overshoots the target speed (accelerates past it then brakes, repeatedly).
3. Add `Ki` if the car consistently settles below the target (steady-state error from friction or incline).

Typical starting point: `Kp=1.0`, `Ki=0.1`, `Kd=0.05` — but this depends entirely on your motor, encoder resolution, and units.

### Pros

- Closed-loop: compensates for battery drain, inclines, surface changes
- Precise speed control — can stop at specific points, maintain exact velocity
- Robust on tracks with speed bumps, ramps, or mixed surfaces

### Cons

- Requires an encoder (extra hardware + wiring)
- More code complexity (two PID loops, encoder interrupt handling)
- More tuning work (two sets of gains)

### When to use

- Track has inclines or speed bumps
- Battery voltage drops noticeably during runs
- You need to stop at precise locations
- You want maximum consistency across runs

---

## Migration path

1. **Now:** Implement Option B. Tune `base_speed` and `min_speed` alongside the steering PID on the flat track.
2. **Later:** Add an encoder to a wheel. Implement the speed measurement. Replace the direct throttle mapping with the speed PID, keeping the adaptive target speed logic from Option B.

The steering PID stays the same in both options — only the throttle side changes.
