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

During the grace period the car keeps turning the way it was. If the lane reappears in time, it resumes normally. If not, it brakes.

### Tuning

| Parameter           | Effect                                               |
|---------------------|------------------------------------------------------|
| `LANE_LOSS_TIMEOUT` | Higher = more time to recover, but riskier if truly lost |

`0.2s` is the Normal and Straight value and stays. The detour track needs far more, but only inside a detour: replaying the hand-driven clips through the tracker shows no line inside the ROI for **~1.2 s** at a detour entrance and **4.2-5.5 s** through the merge into the U-turn after a detour (`detour_entry.mov`, `detour_exit.mov`). The outer line is only visible at the very top of the frame there, and a far line does not say which way to steer; the driver's answer is to keep full lock until a line returns. So the **DETOUR control profile** (selected by `Path: Detour`, see below) holds the last steering and throttle for `_DETOUR_HOLD_S = 8.0` s. The price is that a car that has really left the track keeps circling for up to 8 s before it stops; the e-stop and the red `Stop:` label are the backstop. Normal never sees this constant.

Two things go with it, both DETOUR-only. A **provisional reading** (an anchored frame where only one scan row found an edge) does not end a running hold: coming out of the blind stretch the first thing the ROI meets is the outer line clipping its top-left corner, twelve one-row frames reading -7 to -73 px, i.e. full left, then two-row frames whose curvature swung between -0.54 and +0.9. One such frame used to end the hold, steer straight, lose the lane again and then hold *that*. And the first accepted reading after a hold **seeds the derivative** (`_prev_error = error`) so a seconds-old error does not feed Kd.

---

## Steering Rate Limiting

The PID can jump from `-1.0` to `+1.0` between two frames. A real servo can't do that instantly, and it causes aggressive jerky corrections.

Clamping the maximum change per frame produces smoother transitions. This is sometimes called "slew rate limiting."

Implemented in `autonomous.py` for the DETOUR profile only, as `_DETOUR_STEER_RATE = 4.0` per second (lock to lock in half a second, about what the actuator does anyway), applied to the PID output against the previous frame's command. Coming out of a blind stretch the raw commands read `+1.0, +0.1, +0.6, -1.0, +1.0` on consecutive frames; with the limit the wheels stay near full lock through that. Normal is untouched: the working path is byte-identical to `bb5853e`, proven by `tools/parity_check.py`.

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

### Tried and reverted (2026-09-10)

Implemented once (error in lane widths, `Kp` slider 0-6, hold tightening toward lock while blind) and reverted the same day: on the track it was much worse than the pixel error. The pixel gain on this camera (lane ~1200 px wide at the far ROI row, full lock at 43 px off with `Kp 0.0231`) is close to bang-bang, and the car's line through the detour exit depends on that: with proportional steering the hold entered the blind stretch at less than half lock. Keep the pixel error unless the whole loop is retuned from scratch, and only with a closed-loop test (`tools/synth_track.py`); open-loop replay of the hand-driven clips did not predict this.


---

## Path Choice at Detours

The scan takes the midpoint between the first white pixel on each side, so it always splits the difference. Where the track forks, one marking follows the branch and the other continues, and the midpoint is wrong four ways:

| Situation | Where the car is | What the plain scan does |
|---|---|---|
| Detour entrance, going straight | main road | follows the detour's outer line as it peels off, midpoint drifts right, snaps back at the island tip (oscillation) |
| Detour entrance, taking it | main road | never prefers the right line, so the car stays on the main road |
| Detour exit, going straight | main road | the right line ends at the island tip, the scan jumps to the detour's outer line |
| Merge out of the detour | inside the detour | the mirror image on the left, then a 4-5 s blind stretch into a ~180° right turn |

Anchoring pins the centre to one edge instead of to the middle of whatever the scan happens to hit.

### The topology does the work

The track is a clockwise loop and every detour leaves to the right. Two facts follow, and they are enough:

- the main road's **left** line is continuous everywhere;
- a detour's **right** line is continuous from its entrance until past its merge.

So: **follow the left edge on the main road, the right edge through a detour.** Nothing is detected automatically; the operator says which, and sign detection will later set the same selector.

### Operator model: `Path`

`src/path.py`: `Path.NORMAL | STRAIGHT | DETOUR`, a radio in the GUI header beside Mode/E-stop, PS5 **R1 = Detour**, **L1 = home**. The choice is latched: no automatic return, and neither the e-stop nor a lane-lost timeout changes it.

| Path | Tracker | Controller | Use |
|---|---|---|---|
| Normal (home) | AUTO, today's two-edge scan | today's rules, 0.2 s grace | the default; byte-identical to `bb5853e` |
| Straight | LEFT anchoring | today's rules | the fix for the fork oscillation; gets the first live A/B against Normal |
| Detour | RIGHT anchoring | DETOUR profile: 8 s hold, 4.0/s slew, provisional frames ignored while holding, derivative seeded on hold exit, release deferred 2 s | from the sign until the car is inside the detour (automatic), then the hold is carried through the merge |

`HOME_PATH` (what L1 and the "(home)" radio label mean) is Straight since 2026-09-22: the Straight lap ran clean where the Normal lap oscillated, and the bias check passed on the day-0 recording (`--judge-bias`, 43 clean frames, 1.9 px mean). Normal stays selectable.

**Operating rule:** press R1 at the sign, before the lines diverge. The car returns home by itself once it is inside the detour (below); L1 is the manual override and still works at any time. Arming Detour early on the plain road turns any hallucination-induced lane loss into an 8 s hold instead of a 0.2 s stop; arm at the sign.

### Automatic return (`path.DetourReturn`, 2026-09-29)

The merge cannot be detected reliably, but the entrance can, and Straight handles the exit and the U-turn as well as Detour does (live 2026-09-24, both modes). So Detour only has to last from the sign until the car is inside the detour:

1. armed (R1, later the sign detector);
2. the **entrance gap**: 0.3 s with no lane measured (no both-lines ROI row, fewer than two band rows, or no lane at all); the peeling right line produces exactly this;
3. **steady inside**: 2 s of two or more both-lines rows on non-provisional frames; then the path drops to `HOME_PATH` and the turn signal goes off.

The car is then still inside the detour with the blind merge ahead, and Straight's 0.2 s grace would stop it there (1.4 s blind on the day-1 recording). So the DETOUR control profile stays **carried** until one hold has run and the lane came back, or 30 s have passed. Re-arming by hand cancels the carry. `Lane:` shows `waiting for the entrance`, `inside 1.2s`, `hold carried 4s`.

Measured offline on the day-1 recording (armed at the sign): gap 1 s later, home 5 s after that, 8 s before the merge; the carried hold crosses the merge with no stop. On the entry clip: home at 14 s, inside. Armed on a plain lap it returns after the first spurious lane loss, which is harmless there. The one failure mode: a spurious gap *between* the sign and the entrance returns the car early and it passes the fork straight; re-arm with R1. `parity_check.py` asserts the timings on both recordings.

### Perception (`src/perception/line_tracking.py`)

```python
# LEFT / RIGHT: every row's centre comes from one edge plus half the width
# the lane has steadily had, at that row.
center = left_x + half_width_at(y)   # LEFT
center = right_x - half_width_at(y)  # RIGHT
```

- **Per-row lane width.** Perspective makes the lane wider in the rows near the car. Placing every row's centre with one number tilts the centreline, and the curvature fit (`60 x slope` goes into the PID error) reads the tilt as a bend worth ~40 px, nearly full lock, applied constantly. So the width is a line in the image row, `width = a*y + b`, fitted robustly (Theil-Sen; the far row regularly catches an edge that is not the lane).
- **Band calibration.** On this camera (wide angle, looking steeply down) the lane spans ~90 % of the frame at the far ROI row and the two near rows almost never see a line: the ROI has exactly one both-lines row per main-road frame and can never fit a line through it. The rows between `0.35 h` and the top of the ROI see both lines in every main-road frame, with the width linear in `y`, so the geometry is learned from those **band rows** (every 12th row, chained upward from the scan origin) merged with the ROI's both-lines rows. The intercept is re-anchored on the ROI rows when there is one, because the band's linear fit over-predicts the near rows by 2-5 % under barrel distortion. The band only measures; it never produces a centre, and Normal's output is untouched by it. Bootstrap counts 30 frames whose band-median width was steady for 15 frames, then freezes a copy of the profile for judging steadiness. The tracker reports `calibrating k/30` until then; Straight and Detour steer on the two-edge scan meanwhile and the GUI's Path label turns red.
- **No known width.** Nothing is assumed about how wide a lane is: the width is whatever the lane has *steadily* been. Whenever rows measure the lane and the width (normalised by the frozen calibration shape) has stayed within 3 % for 15 frames, the profile is relearned from the rows that agree with the median. A width that is still changing (a fork opening, a merge) is nobody's lane and teaches nothing, which is what keeps an entrance from ratcheting the width up as the right line peels away. The detour/main width ratio is *unknown* on this track: clip fits give 0.83-0.93x but with a pitch change mixed in, the SVG says 1.10x, the September "1.5x" premise was for another layout. At `Kp 0.0231` a 5 % half-width bias is more than full lock, so the sign matters and must come from a tape measure, not from the image.
- **Chained scan.** Rows go near-to-far, each row starting from the previous row's centre, so once the chain is inside a detour it keeps finding that detour's edge instead of the line that reappears across the island. An anchored edge that jumps *inward* by more than half a lane is dropped; a row where the anchored edge is missing but the other edge sits within a lane of the chain uses that edge instead (this is how the island line steers full right as the merge opens, and how the outer line alone steers the U-turn).
- **Anchored edge to the frame edge.** At a fork the peeling right line sits at x 1318-1349 of 1350 while the ROI slider ends at 1323: the last 13-22 px are where the line is. In anchored modes the anchored edge is searched to the frame edge; the other edge keeps the ROI bounds; AUTO is untouched.
- **Provisional frames.** An anchored frame with a single row yields an offset (Straight must not stop on this camera's normal two-row main road) but does not move the smoothing, the scan origin, the curvature or the steadiness history, and the DETOUR profile ignores it while a hold is running. `Lane: anchor-right 1/5 provisional`.
- `reset()` forgets the lane, not the camera: a frame-read failure or a `--video` rewind must not turn a Detour into an 8 s LOST hold while the profile recalibrates. Recalibrate by restarting the tuner on a straight.

Dropped from the September attempt, and why: gap counting, RIGHT-to-LEFT hand-back and the "weak" majority rule (six constants; never fired on the real exit, where there is no steady two-line stretch before the U-turn), the Config retune and camera index 3 (that session's camera), the stale-sonar change, a global hold and slew on the working path.

### The four situations, with the Path set right

| Situation | Path | What happens |
|---|---|---|
| Entrance, going straight | Straight | LEFT follows the continuous left line; the peeling right line only measures width, its ramp fails the steadiness gate, nothing is learned, nothing snaps back |
| Exit, going straight | Straight | the right line ending is the non-anchored edge; no effect |
| Entrance, taking it | Detour | `right_x - half(y)` per row while the line is in frame, then the hold carries the last right-turn command through the ~1.2 s blind entrance until the island line appears as a left edge (clip: offset +90..+155, full right) |
| Merge from inside | Detour | island line via the other-edge fallback (+46..+90, full right), island ends, hold full right at curve-floor throttle for up to 8 s (clip: 4.2-5.5 s at hand speed), one-row frames ignored, two-row frames accepted slew-bounded, then the U-turn on the outer line + `half(y)`, curvature ≈ +0.9 |

Whether held full lock over-rotates before the outer line re-enters the ROI (`v·t / R_lock`) is invisible to perception; it is decided in the closed-loop simulator with measured car parameters and on track day 2, not here.

### Clip provenance

`clips/detour_entry.mov` (mirrored; replay with `--flip`) and `clips/detour_exit.mov` are **iOS screen recordings** (`ReplayKitRecording`, 2700x1536, ~28 fps), not car-pipeline captures, hand-driven with the camera pitch changing between segments by 90-160 rows. Use them for perception timing only; every pixel number above must be re-measured from the car's own `--record` output (step 0 of the plan) before anything is tuned. `assets/road.mov` is the old track (no detour, no sharp bend).

### Proving it

Offline, without the car:

- `python tools/parity_check.py assets/road.mov clips/detour_exit.mov clips/detour_entry.mov --flip clips/detour_entry.mov` — Normal's PID error byte-identical to `bb5853e` on every frame under wide and tuned random sliders (default-argument and explicit-AUTO calls); the NORMAL controller identical on every step of random sequences with None runs, random dt, sonar states and resets; calibration ready within 60 frames (or 2 s) at `Config()` on every clip, `road.mov` included; the fork teaches nothing (the profile slope does not move from the moment the fork opens until the lane is measured again; before that the hand-held camera's pitch drift legitimately moves it); the operator replay (R1/L1 scripted, plus "forgot L1" and "early L1") through the real `Autonomous` meets its budgets; Straight against Normal on clean frames, i.e. frames where Normal resolved every row from both lines and Straight measured a whole lane on every row (Normal's single-edge rows carry the scalar-width tilt and its far row reaches across to the next line on `road.mov`, so it is not a reference elsewhere; pass/fail on a live recording with `--judge-bias`); LEFT and RIGHT smoke on all footage. Status 2026-09-21: everything green on the three clips; Straight against Normal on `road.mov` differs by 2.4 px mean on clean frames and by under 0.5 px row-wise on the four near rows.
- `python tools/clip_census.py CLIP [--flip CLIP] [--path detour]` — what the tracker sees on any clip or recording: per-row edge classes, both-rows histogram, offset/curvature/PID error, LOST and provisional runs, band visibility.
- `python tools/synth_track.py --scenario detour-hard --check --sweep` — the closed-loop simulator: camera model with distortion, kinematic bicycle from measured car parameters, the SVG track with fillet/merge/width as parameters, a scripted operator. It is gated by `--replay-census RECORDING` against a live Normal lap before any constant is read off it, and its results are go/no-go rankings, not predictions. Status 2026-09-21: it runs end to end, but every physical number in it is a placeholder (`--print-params` lists them) and with those placeholders even Normal crosses a line at the hard merge; nothing has been read off it yet.

Live, in this order: Normal lap; Straight lap A/B (`--record --log`); straight through both forks in Straight; the easy detour (R1 at the sign, L1 after re-acquisition); **then** the simulator with the measured parameters; **then** the hard detour at reduced `base_speed` with a hand on Triangle; then the `HOME_PATH` flip.

### Measurements the plan needs from the floor

| Measurement | For |
|---|---|
| Main and detour lane widths, line centre to centre (tape) | the width ratio's sign; `SCALE` in the sim |
| U-turn inner/outer radii, island tip to U-turn distance | the hard-exit scenario |
| Fork fillet radius, merge angle | the entrance/merge scenarios (the SVG's 67° unfilleted junction contradicts the described curve) |
| Wheelbase, full-lock turning radius both ways (chalk circle) | `R_lock`, over-rotation during the hold |
| m/s at throttle 0.375 / 0.50 / 0.63 (timed 3 m), steering lock-to-lock time | `v(throttle)`, actuator lag |
| Camera height, pitch, FOV, one checkerboard clip | the `car` camera preset, `k1` |
| Loop fps of `src/main.py` against `bb5853e` | the hold/release constants are in seconds; the chained scan and band cost CPU |

Replay: `python src/main.py --video clips/detour_exit.mov` (add `--flip` for the mirrored entry clip), `--record out.mp4 --log run.csv` to capture a run, `--camera N` to pick the camera.

---

## Obstacle Slow Zone in a Bend

The sonar node reports three HC-SR04 distances; the master forwards the nearest and all three. The controller slows from 150 cm and stops at 60 cm. On this track every tight right turn at full lock reads **97-143 cm** for several seconds (`recordings/day1_detour.csv` 47-53 s in the U-turn after the detour, `day0_normal.csv` 85-88 s): the sensors see the wall or cones on the outside of the bend, which the car's arc never reaches. The throttle fell from ~0.55 to 0.22-0.33 and the car crept; that was the "pause" at the detour exit, not the lane hold.

Fix (2026-09-22, all profiles): the slow zone only counts echoes the path can reach. With the wheels at `steering` the path is an arc of radius about `_R_LOCK_M / |steering|`, and after that distance it has turned 90° away, so an echo farther than that along the ray is off the path. The **stop zone is unchanged**: anything at or under 60 cm on any sensor still stops the car, and a `B` state from the master always stops it. With the wheels straight the rule is identical to the old one. `_R_LOCK_M = 0.6` is a placeholder until the full-lock radius is chalked; overestimating it only makes the rule more cautious. The CSV log now carries the three per-sensor distances (`sonar_lr`) so the sensor that trips can be identified.

Superseded (2026-09-30, `src/obstacle.py`): per-sensor slow distances, the stop is a latch, and the wheels are held while stopped.

- **Side sensors beside the track stopped the car.** The master's `B` state and the host's stop zone took the nearest of all three sensors, and the side sensors point outward, so a cone or a person beside the track stopped or slowed the car. The side sensors were first made stop-only (at or under 60 cm, right in front of a front corner); see below for their slow zone. The master's `B`/`S` states are ignored; only `X` (stale) still stops the car.
- **A geometric corridor was tried and dropped the same day.** Placing each echo at the end of its beam and testing it against the steering arc (then against the lane width near the car) needed the sensor offsets, the beam angles, the car and lane widths and the turning radius. With guessed values it missed in-lane obstacles in bends (the arc swung away from them), and with the lane width entered it slowed and stopped for a person standing off the track about 80 cm from a side sensor. The per-sensor rule needs none of those numbers. The bend-wall slow-down (`_R_LOCK_M` reach rule above) went with it: in a tight bend the centre sensor can again slow the car for the wall on the outside.
- **Side sensors stopped late and abruptly, and sometimes not at all** (third drive). Stop-only sides reached 60 cm at full speed, and an obstacle in front of a front corner sits at the edge of a side beam, where it echoes on some pings only; the median then read "nothing" between them and the car sped back up. The sides now slow the car too, from 100 cm (later than the centre's 150 cm, as they see the track side), a measured distance is held for 0.5 s through missing echoes, and the master reports the sonar at 20 Hz instead of 10 (it dropped every other CAN frame). Simulated at 60 % missed echoes: stops 71 % of approaches before, 96 % after; a person beside the track at 80 cm now halves the speed, the price of the side slow zone.
- **The car stalled in the slow zone** (fourth drive). The ramp scaled the throttle toward zero at 60 cm, and the heavy car stops moving well above zero, so it came to rest at 70-110 cm still "slowing": no stop, no hazards. The slowed throttle is now floored at the crawl the controller already uses on the sharpest bend (`min_speed * _SHARP_CURVE_FLOOR`, 0.375 at defaults), so the car creeps to 60 cm and does the real stop. With no obstacle the throttle is unchanged (`max(t, min(t, crawl)) == t`).
- **Stop-and-go in front of a stationary obstacle.** An object at the threshold reads a few cm either side of 60 from one ping to the next, and an HC-SR04 drops an echo (255) now and then. Each sensor is median-filtered over its last three readings (one reading, 50 ms at 20 Hz, of latency), and the stop latches: it resumes only after every sensor reads farther than 75 cm for 0.5 s.
- **Steering swung while stopped.** The controller kept running the PID on the lane reading of a car standing still, and that reading is noise. While the guard says stop, `Autonomous.step` returns the last steering with zero throttle and freezes the lane-loss hold, the profile release and the detour return timers; the first reading after it seeds the derivative.
- **Hazards** flash while stopped for an obstacle (and the strip goes red), over the operator's turn signal, which returns when the car moves off.

`tools/parity_check.py` checks the guard on scripted scenes (the old ramp dead ahead, side sensors that stop but never slow, the latch under a wobbling reading, a dropout and a ghost, stale data) and that NORMAL is otherwise identical to `bb5853e` with the throttle scaled by the guard.
