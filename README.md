## Calibrate PID

Start with only Kp (set Ki=0, Kd=0):

1. Put the car on the track at low speed
2. Start with a very small Kp (e.g., 0.005)
3. Increase gradually until the car follows the lane but starts oscillating (wiggling left-right)
4. Back off Kp by ~30% — this is your baseline

Add Kd to kill oscillation:

1. Start with Kd = Kp * 0.5 as a rough starting point
2. Increase until oscillation stops and steering feels smooth
3. Too much Kd → sluggish response, slow to react to curves

Add Ki only if needed:
1. Most lane-following doesn’t need it
2. Only add if the car consistently drifts to one side on straights (steady-state error)
3. Start very small (Ki = Kp * 0.01) — integral winds up fast and causes overshoot

### Practical tips
- Tune at competition speed — PID behavior changes with speed
- Watch the steering bar you now have on screen — it tells you if the output is saturating (red) too often. If it’s always maxed out, Kp is too high
- Log the values — print error, steering, and dt to a file so you can review after a run instead of guessing
- The 0.01 Kp I set is a guess. Your actual value depends on the pixel error range and how sensitive your servo is. If typical errors are ~100px and you want ~0.5 steering output, then Kp = 0.005 is a better start