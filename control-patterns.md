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

`src/path.py`: `Path.NORMAL | STRAIGHT | DETOUR`, held by the main loop (`path` in `src/main.py`) and written by PS5 **R1 = Detour** / **L1 = home**, the GUI header radio, the pickup/delivery mission and the automatic return. The GUI only shows it. The choice is latched: no automatic return, and neither the e-stop nor a lane-lost timeout changes it.

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

The car is then still inside the detour with the blind merge ahead, and Straight's 0.2 s grace would stop it there (1.4 s blind on the day-1 recording; the day-4 run stopped exactly so after an L1 pressed early). So on **any** return home from Detour, automatic or L1, the DETOUR control profile stays **carried** until a hold of at least 0.3 s has run after the car was inside and the lane came back, or 20 s have passed. No carry if such a hold already ran while armed (the merge is behind the car). Re-arming by hand cancels the carry. `Lane:` shows `waiting for the entrance`, `inside 1.2s`, `hold carried 4s`.

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

### Blind stretches at speed (2026-09-29/30)

With `base_speed` raised to 0.75 and 0.83 the car passed both forks straight (`day2_detour`, `day3_detour`). The logs showed why: the peeling right line leaves the frame row by row, so the last two or three frames before the blind stretch read one or two rows, near zero or the wrong sign (the other-edge fallback re-centres on the main road), and the slew let the wheel decay from 0.8 to 0.30 (day 2) and to -0.64 (day 3) before the hold latched it. At 0.63 the same frames exist; the car simply had more frames at full lock and the last one happened to be positive. Two DETOUR-profile rules, no new knob:

- **Weak frames are not readings.** A one-row bearing, a minority of the scan rows, or a frame without curvature when the scan has rows to fit one, starts or continues a hold instead of steering (manila's majority rule, restored for DETOUR only; on this camera anchored rows are 4-5 of 5 on the main road and inside the detour).
- **The hold latches the strongest recent right turn.** When the lane is lost, the held steering is the largest right-turn command of the last 0.5 s of accepted readings, not the last frame's; throttle follows the curve rule for it. Every blind stretch of a right-hand detour is a right turn (entrance, merge, U-turn), so a left excursion in the window is a correction on the way in, not the turn. If nothing in the window turned right the last command is held as before (a hallucinated loss on a straight holds ~0).

Replayed offline, all four recordings now latch +1.00 at the first hold after arming and day 4 crosses the merge without a stop; day 1 (the run that worked) is unchanged in outcome. `parity_check.py` asserts these on the recordings, with the sliders they were driven with (ROI x 0.01-0.99). Whether the car actually enters at 0.83 is a track-day question: at some speed the fillet is tighter than the car's full-lock radius.

### The U-turn at speed (2026-09-30, runs 5-7)

With `min_speed` raised to 0.70 and 0.79 the car crossed onto the outer line in the U-turn and sat at full lock for 3-4 s before recovering (`day6`, `day7`). Two things in the logs, one of them fixable:

- **Garbage from the lens streak.** Deep in the U-turn the outer line is above the ROI entirely (visible only in the band rows). The only "edge" the ROI rows found was a bright streak 6-11 px wide along the last 20 px of the frame's right edge, taken by the other-edge fallback as the inner line: full right from garbage, then a hard left (-0.82) the moment the real line clipped the far row. That is the "turns right, then straight for a moment" the driver saw. Fixed in the anchored scan: a fallback hit that is both thinner than 2 % of the lane width and within 3 % of the frame width from the edge is dropped, so those frames become a hold at the latched right turn. Thinness alone would reject real lines (13-40 px here), position alone a line leaving the frame. The anchored edge is exempt so the peeling line's last remnant still drives the wheel to full lock at the fork. AUTO is untouched.
- **Speed.** The curve-speed rule can only slow to `min_speed`, so `min_speed` *is* the U-turn speed: 0.50-0.53 (run 1) took it inside the lane, 0.70 and 0.79 ran wide at full lock for seconds, which no steering rule can fix. `base_speed` can stay high: the curvature lookahead slows the car to `min_speed` before the bend.

Also: the carried DETOUR profile now lasts until the lane has been whole for 2 s after the merge hold, not until the hold ends, because the U-turn on the outer line follows the merge immediately and run 7 went back to the 0.2 s grace in the middle of it.

### The weave on straights (2026-10-01, run 10)

A full Straight lap at Kp 0.0231 and Kd 0.0008 weaves. It is not jitter: the error swings smoothly from -80 to +76 px with a 2.5-3 s period, the steering follows it with 0.95 frame-to-frame autocorrelation and sits at full lock 52 % of the time, and the error changes sign 1.6 times a second. That is the limit cycle of a bang-bang controller (full lock at 43 px, 5 % of the lane) with too much delay in the loop. Run 1 at the same Kp did not weave; it ran at 23.5 fps and 0.59 throttle, run 10 at 17.7 fps (59 ms per loop) and 0.68.

Two delay sources sit in the loop, and neither is a gain:

- **Stale frames.** The camera delivers 30 fps and the loop is slower, so `cap.read()` in the loop returns the oldest frame in the driver's buffer, several frames (100-170 ms) old. `main.py` now reads the camera on a thread (`FrameSource`) and hands the loop the newest frame; video replay stays sequential. This also takes the read's own cost out of the loop (the tracker costs 11.5 ms and the GUI 8 ms on the dev machine; the rest of the car's 59 ms is the read).
- **The EMA** (`alpha` 0.4) adds about 1.5 frames of lag; it is an existing slider, raise it before lowering gains.

Kp itself is the right lever against a limit cycle, and lowering it is now safe for the detour: the DETOUR hold is **full right lock** whenever the car was turning right into the blind stretch, whatever the PID had reached. Every recording that held 1.0 came out inside the lane (runs 1, 5, 6) and every one that held less came out wide (0.78 run 7, 0.73 run 8) or missed the fork (runs 2, 3), so the rule is both the driver's answer and the data's. The detour no longer depends on Kp.

### Measured laps and the blind exit (2026-10-13)

Taped: lane 180 cm between the tape's inner edges (±5-10 cm), tape 8 cm, car 90 cm across the tyres, so **45 cm to each side** when centred. The wide lens makes the tape leave the frame beside the tyres even then; that is not the margin. Calibration clips `cal_center/left/right` put the tape's inner edge at x≈366 (row 0.90) and x≈351 (row 0.95) of 1920 with the left wheels touching it, and x≈1597 / 1614 with the right wheels touching.

Main road (`lap_a1-a4` at Kp 0.014, `lap_b1-b2` at 0.010, ROI 0.60-0.85, EMA 0.7, base 0.80): no lane loss, full lock about 10 % of frames, |offset| p50 23 px / p95 45 px at 0.014 against 40 / 72 px at 0.010. Keep 0.014.

The detour laps (`lap_a5/a6`, the 90-degree exit) both left the lane at the exit and in the S-curve after it. The ROI saw no line for **6.5-6.7 s** after the merge while the DETOUR hold kept full right lock: the outer line sat in the top third of the frame the whole time, and the car turned far past 90 degrees. The driver's run of the same exit (`detour14`) eased from 1.0 to 0.8 after about 1.4 s and the line came back into the ROI 1.8 s after it left.

- **Far reading** (`LineTracker._read_far`, LEFT anchoring with a blind ROI only): the anchored scan runs on the rows above the ROI, and the nearest ROI-height of rows that resolve gives an offset and a slope in the ROI's units. A DETOUR hold steers on `Kp * (offset + 60 * slope)` from its second frame, P only and slew-limited; with no far reading it holds as before. Replayed on the driver's line it says full lock; on `lap_a5/a6` full lock for the first ~2 s, then 0.05-0.7 once the car had over-rotated. Not on RIGHT: at the entrance the junction's V of tape made it jump between full left and full right.
- `lap_c1` (Straight) unchanged; `lap_c2` (detour) stayed in the lane at the exit. Its exit hold ran 7.67 s, every frame after the first steered on the far line, 0.3 s short of the 8 s stop: the stop now counts only frames with no line anywhere (`_blind_time`), a far-steered frame resets it.
- **Entry width (tried and reverted, 2026-10-16).** The detour lane is narrower than the main road (taped 160 against 180 on 10-13; 150-170 on 10-16), and right-anchored the centre is the right line minus half the learned width. Scaling the working profile by 160/180 at the entrance gap looked right open-loop on `lap_c2`, where the readings had put the centre ~10 cm left and the car straightened mid-entry. On `lap_e1` (camera fixed, widths of the day) the same scaling turned a +0.25..+0.58 entry into full lock for 2 s and the car cut the inside tape. A fixed ratio cannot follow widths that change from day to day; the learned width stays the only width.
- **Weak frames follow the ROI** (`lap_d1`, 2026-10-15). After a GoPro remount the ROI (0.60-0.85, unchanged) saw the line on 2 of 5 rows on every straight. Straight drove it fine, but in DETOUR a frame with half the rows or fewer is weak, so from R1 on every frame was a hold at the last command (-0.03) and the car went straight past the fork. A DETOUR frame is now weak only below what the ROI typically resolves on the main road (median rows of the last 90 NORMAL frames with a lane), and never above half the scan lines: at 3-5 typical rows nothing changes. Replayed on d1 the car steers on the 2-row frames and the peel latches full lock. The ROI still belongs where the lines are: at 0.45-0.75 the same footage reads 4-5 rows.
- **Entry hold steering** (`_DETOUR_HOLD_STEER`, `lap_e4`, camera fixed). The entry kept cutting the right tape. A delay before the turn-in (0.15 and 0.25 s) moved the wheels 0.3 s later and changed nothing on the track: the cut happens in the blind holds right after (1.4 s and 1.0 s at full lock on e4), where full lock turns tighter than the detour's curve and the right tape ends up beside the right wheels. Neither line above the ROI can steer those holds (in their first part it is the main road's line, left or right). A blind right-turn hold now keeps less than full lock; at the exit that is only the first frame, the far reading steers the rest. Tuned on the track on 10-19 (camera fixed, base 0.80 / min 0.67): **0.70** enters cleanly on every test; from about 0.75 up it cuts the right tape again. Lower if the entry cuts, higher if it runs wide towards the island tip; re-tune if the camera angle or the speeds change.
- Two readings that look natural and are wrong: the median over all far rows (a bend ahead pulls it to "turn more"), and the lateral position on the nearest far rows alone around a held full lock (it reads about -70 px on the driver's own correct line too).
- **Carry release**: after the merge hold the carried profile now needs the lane *measured* (both lines in the ROI or the band) for 2 s, and only a hold of 0.3 s or more restarts the count. Both lines on two ROI rows happens in 16-27 % of frames at ROI 0.60-0.85, and every weak frame in the S-curve restarted it, so the carry ran to its 20 s cap. Replayed, it ends 2 s after the exit hold (frame 833/837 against 1079/1090).

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

## Pickup and Delivery

`src/input/mission.py`. The operator picks a pickup and a delivery letter (A, B or C) in the GUI's Mission block while e-stopped; picking both starts a run from before exit 1, "-" turns the mission off and the car drives exactly as without it. Parked after a delivery, the next pair can be picked without an e-stop and the run starts from that dock (`plan_route(..., start=letter)`); clearing both letters forgets the dock and needs an e-stop.

**Track model.** From the start: exit 1 (A inside its detour), exit 2 (B inside) a few metres further, C on the main road, round again. Exit 2's mouth sits just past exit 1's, so detour 1 rejoins the road beyond it: **exit 2 is only reached by going straight at exit 1**, and every other way (detour 1, or exit 2 either way) leads round past C to exit 1. Both detour signs look alike and nothing marks a lap, so the mission counts: `next_exit` starts at 1 and moves on at every detour sign by `_exit_after(exit, entered)` (2 only after straight at exit 1, else 1). The first version had "after exit 1 always comes exit 2" from the task's examples; live on 2026-10-07, pickup A then delivery B, the sign after A was exit 1's again, the car took it for exit 2, entered exit 1 for B, and A's fix (then "exit 2 next") kept it in that loop.

**At a detour sign** (confirmed, within 3 m, decided once): enter only if the current target - pickup until collected, then delivery - is the letter inside this exit; otherwise go straight. On a fixed cycle that is already the shortest route. The decision is a one-shot request on the loop's `path` (Detour, or Straight if not already), so R1/L1 still override; the return home is `DetourReturn`'s. A sign that turns up under a new track id counts as the one already decided if that one was seen within 2 s and the new one is no farther than it was (+0.5 m): the tracker lost it and found it again. A sign farther down the road is a new exit even while the last one is still in view; exits can sit a few metres apart (`record1_signs.mov`: exit 2's sign at 4.4 m while exit 1's was at 1.25 m, a detour sign in view without a break for 10 s, which the first version, "none in view for 3 s", never re-armed on). No sign is decided while the path is still Detour.

**Back-to-back exits.** With the exits that close, the second sign is decided (3 m) while the car is still level with the first exit's mouth. Arming Detour there would anchor on the first detour's right line and take the wrong exit, and `DetourReturn` would take that mouth for the entrance and send the path home before the second one. So the mission counts the exits it decided straight whose mouth the lane camera has not shown yet, and a Detour decided while one is still ahead **waits**. `path.ForkWatch` reports a mouth: outside Detour, the lane unmeasured for `_RETURN_GAP_S` (the right line peeling away) and then measured again for as long, the same test `DetourReturn` uses. The waiting Detour is armed on the frame the last mouth goes by. If its own sign is out of view for 1 s first, the mouth before it was missed, and arming then would come after this exit's own mouth, so it is dropped: straight on, next lap. Any letter read resets the count (every letter stands past the mouths behind it). The `--log` CSV has a `fork` column, 1 on the frames ForkWatch fired.

**Position fixes.** A letter read within 3 m is an absolute fix, once per track id: every letter sets `next_exit = 1` (inside either detour, or at C, the road leads round to exit 1). A detection failure costs a detour or a lap, never the mission:

| Failure | What happens |
|---|---|
| Needs B, counter one ahead, enters exit 1 and reads A | fixed to 1, no stop; next lap straight at exit 1, exit 2 for B |
| Needs A, exit 1's sign missed | exit 2 is taken for exit 1 and entered; B fixes the count; A one lap later |
| Target letter unreadable | nothing changes; the same exit is taken again next lap |
| Phantom or double-counted sign | the next letter read corrects it; C does every lap |

The price: a letter readable within 3 m from the neighbouring road would give a false fix. Check on the floor.

**Only confirmed, only close.** Tracks count after 3 of the last 5 frames, letters after 3 of the last 5 readings (`sign_tracker.py`), and nothing further than 3 m acts; a box clipped by the frame edge has no distance and never acts. A snapshot older than 0.4 s decides nothing.

**At the target letter: the dock handshake.** Same ramp as the stop sign (full speed at 3 m, stopped at 1.5 m), right signal on the approach. Then the car **docks** and waits for as long as loading or unloading takes. There is no timer: a truck carries a lot of cargo, and only the person at the dock knows when it is done. They **release** the truck: "Release truck" in the tuner (the dashboard's dock card later), or PS5 Cross. That is how real docks work: the attendant switches the dock light to green when loading is done and the dock is clear, and only then does the driver pull away. Aurora's driverless trucks likewise get a final self-check in a departure zone before they are cleared to leave.

| State | Car | Signals | Strip / lamps | Voice (pt-BR, macOS `say`) |
|---|---|---|---|---|
| Docked | held | hazards | BLUE_BREATHE; door projectors and logo on (the loading zone) | "Caminhão na doca A. Carregue e, quando terminar, toque em Liberar no painel."; a reminder every 30 s |
| Checking (released, pickup) | held until the voice has finished, then until the sonar reads the way ahead clear for 2 s; any echo restarts the 2 s | left | CYAN_SWEEP; projectors off | "Liberado. Afaste-se, o caminhão vai partir."; once if blocked: "Por favor, afaste-se da frente do caminhão." |
| Leaving | drives | left for 2 s | cyan | - |
| Delivered (released, delivery) | held, parked | none | GREEN for 3 s, then BLUE_BREATHE with projectors on | "Entrega concluída na doca C. Obrigado." |

"Clear" is the sonar guard: not stopped and no sensor in its slow zone (150 cm centre, 100 cm sides), so an attendant standing beside the truck also holds it. "Wait" (tuner) or an e-stop takes a released car back to docked, and it needs a new release; an e-stop keeps the dock, since the car has not moved. A release away from a dock is refused and logged. The same letter for both loads and unloads at one dock. An approach whose track is lost for 1 s is abandoned (the letter comes round again); one that never reaches 1.5 m within 8 s docks where it is. The loop takes the lower of the sign policy's and the mission's scale and either one's hold; signals go through `LightsController.signal_override`, which beats the operator's radio while set (Right while Detour is armed, the mission's while it drives). The dock scene ranks below the operator's strip choice, e-stop, obstacle and sign stops, and the projectors/logo are OR-ed with the operator's checkboxes. BLUE_BREATHE (9) and CYAN_SWEEP (10) are new lights-Nano effects; firmware older than them shows the strip dark at a dock. "Cyan" on the strip (the autonomous underglow and the sweep) is drawn as SAE J3134's ADS-marker turquoise, the colour of Mercedes' DRIVE PILOT marker lights (CIE 1931 x 0.012-0.20, y 0.32-0.495): green 255 with blue 85 sits mid-box for WS2812 datasheet primaries, where the old full cyan (blue 255) read y 0.21, a sky blue. Check it by eye on the real strip; LED bins vary. `--no-voice` silences the prompts.

**The cargo compartment.** This works the way Nuro and Starship delivery robots do: the cargo travels locked and opens only at the right stop. A cargo node (`firmware/cargo`, an ESP32 that the sonar node bridges onto CAN over a serial cable) has three parts:
- a load cell under the bed,
- a switch on the lid,
- a 12 V solenoid lock.

The master forwards its status as `C` lines (`src/cargo.py`, `arduino.read_cargo`). The host counts the node as present while those lines are less than 1 s old. **With no node there are no checks and no `K` lines,** so the handshake above runs exactly as written.

With the node:
- **Locked except at the dock.** It unlocks while the truck waits at its pickup or delivery dock, until the lid is lifted. It never unlocks while driving, slowing, checking, leaving, parked, at a dock it is only leaving from, or while driven by hand. The slanted latch catches again as the lid shuts.
- **Three locks in series.** The mission's `unlock`, the master refusing any unlock while the throttle is not 0, and a 300 ms lease on the node: the host renews `K 1` every frame, so a dead host, master, bus, sonar node or cable leaves the compartment locked.
- **Closing the lid is the release.** It counts only if the lid was lifted at this dock first; a lid that stays shut releases nothing. A Starship robot leaves when the customer closes the lid. The tuner and PS5 releases still work.
- **Weight is checked, never a trigger.** A truck carries a lot of cargo, so the weight cannot tell when loading is done.

Once released, the truck runs its own checks, in order:

| Check | Fails when | Then |
|---|---|---|
| Lid closed | open (e.g. released from the tuner mid-load) | holds in checking until it shuts: "Feche o compartimento de carga." once, instead of "Liberado" |
| Weight (once, at a pickup or delivery dock) | pickup: under 20 g; delivery: 20 g or more | back to docked and unlocked again: "Nenhuma carga detectada..." / "Ainda há carga no compartimento..."; **the next release at that dock skips the weight** (the attendant knows best); the tuner button reads "Release anyway" |
| Sonar clear 2 s | as above | as above |

More detail:
- **Settling.** The weight waits for a stable reading (the node's last second within 5 g). It is judged as it stands after 3 s.
- **Fault.** A scale fault skips the weight check, and so does a dock the truck is only leaving from.
- **Manifest.** The weight as the truck leaves its pickup is the manifest (`mission.manifest_g`), shown in the mission status.
- **Losing the node.** If the node drops out mid-check, the truck leaves as it would without one.
- **Prompts.** While the node is present, the dock prompts say to open the lid and close it again (`docked_pickup_lid`...).
- **Opening by hand.** "Open compartment" in the tuner unlocks it while the car is e-stopped, to load before a run or empty it after one. It stays unlocked until the lid is lifted, or for 10 s at most, and never once the car may move (`cargo.ManualOpen`).
- **Calibration.** It is a one-time firmware constant. The scale tares itself at boot, with the bed empty.
- **Logging.** The `--log` CSV has a `cargo` column (`grams stable lid lock fault`).

Not done: pulling over toward the right line at a stop. Any useful bias is a full-right swerve inside the metre between 3 m and 2 m at this near-bang-bang gain, and a swerve back on leaving; it needs `tools/synth_track.py` and a track day first.

Proof: `python -m pytest tests/test_mission.py` drives every ordered pair and each failure above on a scripted lap that follows the mission's own path requests; the dock handshake (waiting, release, blocked departure, wait/e-stop, parking and the next order from every dock) and the compartment (unlock only while waiting, lid release, weight refusal and override, settling, fault, a lost node) are in the same file, the `C`/`K` lines in `tests/test_arduino.py`; the `--log` CSV carries `mission_stage`, `target`, `next_exit`, `mission_state` (`docked`, `checking`).

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
