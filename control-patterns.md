# Control Patterns

Techniques to improve steering robustness and safety beyond the base PID + EMA setup.

---

## Lane-Loss Timeout

When the tracker loses the lane (`lane_offset is None`), the PID resets and outputs `steering = 0.0` — the car drives straight off the track.

A safer pattern: hold the last known steering for a short grace period, then stop.

### Implementation

```python
LANE_LOSS_TIMEOUT = 0.2  # seconds

if lane_offset is not None:
    last_steering = pid.update(lane_offset, dt)
    lost_since = None
else:
    if lost_since is None:
        lost_since = time.time()
    elif time.time() - lost_since > LANE_LOSS_TIMEOUT:
        last_steering = 0.0
        throttle = 0.0  # stop the car
```

During the grace period the car keeps turning the way it was. If the lane reappears within 200ms, it resumes normally. If not, it brakes.

### Tuning

| Parameter           | Effect                                               |
|---------------------|------------------------------------------------------|
| `LANE_LOSS_TIMEOUT` | Higher = more time to recover, but riskier if truly lost |

Start at `0.2s`. Increase if the tracker frequently drops frames on bumpy sections.

---

## Steering Rate Limiting

The PID can jump from `-1.0` to `+1.0` between two frames. A real servo can't do that instantly, and it causes aggressive jerky corrections.

Clamping the maximum change per frame produces smoother transitions. This is sometimes called "slew rate limiting."

### Implementation

```python
MAX_STEER_RATE = 0.1  # max change per frame

delta = new_steering - prev_steering
delta = max(-MAX_STEER_RATE, min(MAX_STEER_RATE, delta))
steering = prev_steering + delta
```

### Tuning

| Parameter        | Effect                                              |
|------------------|-----------------------------------------------------|
| `MAX_STEER_RATE` | Lower = smoother but slower to react to sharp turns |

Start at `0.1`. If the car can't keep up with tight curves, increase it. If the servo jitters on straights, decrease it.

---

## Normalized Error

The `lane_offset` is in raw pixels, so its magnitude depends on frame resolution. If `downscale_ratio` changes, the PID gains break because the same physical offset produces a different pixel count.

Normalizing the error to `[-1.0, 1.0]` makes the PID gains resolution-independent.

### Implementation

```python
roi_half_width = (roi_x1 - roi_x0) / 2
normalized_error = lane_offset / roi_half_width  # always in [-1.0, 1.0]
steering = pid.update(normalized_error, dt)
```

### Impact

- PID gains transfer across different resolutions without retuning
- Error values become intuitive: `0.5` means "halfway to the edge," regardless of resolution
- The PID output is already in `[-1.0, 1.0]`, so having the input in the same range simplifies gain reasoning

### Migration

After normalizing, existing PID gains (`Kp`, `Ki`, `Kd`) will need retuning since the error magnitude changes. Start with `Kp ≈ 1.0` (a full-width offset should produce full steering) and tune from there.
