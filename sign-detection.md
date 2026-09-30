# Sign and traffic-light detection

Phase 1. The car has to recognise five things: a **stop** sign, a **detour**
sign, three **delivery** signs (pentagon B, hexagon A, heptagon C — white,
thin black outline, big black letter), and a **traffic light**.

Phase 1 did not touch how the car drives. Phase 2 (section I, at the end)
wires the stack into `src/main.py` and `src/gui.py` as one throttle scale.
`src/config.py`, `src/input/autonomous.py`, `src/arduino.py` and the firmware
are still untouched, and no new `Config` field exists.

---

## The one thing to know first

The previous attempt reported **mAP50 ≈ 0.97**. On the 28 filename-labelled
venue photos it got **6 right out of 28**, and it called five of six *unlit*
housings RED.

It was not a bad model. It was an unmeasured one: its validation split came
from the same compositor, from the same cutouts, as its training set. So this
phase built the instruments first and the model last, and every number below
is printed by a tool in `tools/` that anyone can re-run.

---

## A. Model

**Fine-tune from COCO weights at `imgsz=640`, export to ONNX (opset 12), run
through `onnxruntime`.** Train both `n` and `s` on the training box and keep
whichever wins on measured real-photo recall at acceptable latency — on a 3060
that is one extra unattended run, so there is no reason to guess.

| Variant | Train on the 3060 | Inference, M2 Pro | Note |
|---|---|---|---|
| YOLOv8n / YOLO11n | ~15–25 min | ~25–40 ms | the safe default |
| YOLOv8s / YOLO11s | ~35–50 min | ~70–110 ms | 3× the parameters; small distant objects are where the extra capacity pays |

A sign camera does not need 30 fps. It needs to not miss the light. If `s`
clears the gates and `n` does not, 10 fps is a trade worth making, and
`tools/eval_signs.py` prints the latency that decides it.

`imgsz` is **640 and is not a parameter**. At 320 the traffic light was missed
outright.

Rejected: RT-DETR / RF-DETR nano (want far more real data than exists, slow on
M2 Pro CPU), MobileNet-SSD and EfficientDet-lite (no accuracy advantage, worse
tooling), anything larger than `s` (the Mac cannot run it usefully), and
classical template or shape matching (in a dark hall the housing has no edges
at all, only three discs — the first attempt proved this).

What travels to another room is barely about the architecture. Any of these
memorises one room if trained on one room. What survives a venue change is the
training *distribution* — see section D.

---

## B. Four classes

```
delivery_sign    <- pentagon B, hexagon A and heptagon C collapse into this
detour
stop
traffic_light    <- the housing only. No red/yellow/green classes.
```

The light's **state** and the delivery sign's **variant** are both read from
the crop, by ordinary image processing, in
`src/perception/traffic_light.py` and `src/perception/delivery_variant.py`.
One architecture, no special case, and both are unit-testable without a camera
and fixable without retraining.

### Why A/B/C are one class

The artwork is frozen at 5/6/7 sides, the hardest discrimination the geometry
allows. Measured on the previous six-class model's *own synthetic validation
split* — the easiest test it will ever face:

| true | predicted as |
|---|---|
| pentagon B | **6 % hexagon A** |
| heptagon C | **10 % pentagon B** |
| stop | ~0 % cross-class |
| traffic_light | ~0 % cross-class |

That is not a training deficiency; it is the information content of one extra
side at 25–40 px, and real footage is worse. A wrong B at range is a
*confident wrong routing decision*. A `delivery_sign` with the variant
deferred costs nothing — the car does not need to know which bay until it is
close enough to turn in. Collapsing also triples the per-class training signal.

---

## C. Reading the light's state

`read_light_state(crop) -> LightReading`. What the real photos actually show,
inspected directly:

- The lit core is **pure white** — saturation ~5/255. There is no hue in it.
- All the colour survives in the **halo**, which spills outside the housing.
- With yellow lit, the two unlit covers pick up the spill and read *brighter*
  than the housing. "Brightest region" is not "lit lamp".
- With everything off, the three covers are grey-white discs that pass any
  brightness test, and the hall's warm stage lighting gives them genuine
  saturation.
- Red hue measures 2–28, yellow 29–31. **Hue cannot separate red from yellow.**

The reader, in order:

1. **Structural verification.** Find three round, vertically co-linear,
   evenly-spaced lamp cells. If the crop does not verify as a three-cell stack
   it is not a light and is never reported as one. This is a second gate,
   independent of the detector. Aspect ratio provably cannot do this job: real
   housings measure h/w 2.2–2.45, box jitter swings that to 1.15–3.22, and
   standing people measure 2.7–4.8.
   Cells are fitted from *pairs* of candidate discs, proposed from three views
   of the crop — plain grey, CLAHE, and the per-channel minimum — because a lit
   lamp fuses with the covers either side of it and stops being a blob of its
   own. Measured over the 38 venue photos: grey alone finds the stack in 29,
   grey plus the minimum in 34, all three in 36.
2. **Which cell is lit**, from the *found cell centres*. The previous version
   sliced the box into equal thirds, and a 2.3:1 housing rounds to two bands,
   so there was no middle one and **every yellow read as red**.
3. **Emission, not brightness.** A source lights up the housing beside it
   (*glow*) and blazes far brighter than its neighbours (*blaze*). Warm stage
   lighting on a grey plastic cover does neither. Measured: the two cues sum to
   **0.33 at most on the eight unlit housings and 0.53 at least on the
   twenty-six lit ones**, so the floor sits at 0.43 with 24 % clear either way.
4. **Colour from the halo ring only, never the core** — including outside the
   housing box, where the spill actually lives. Hue is tallied as votes over
   coarse bands weighted by each pixel's own saturation, never averaged (red
   wraps 0/180; averaging a red halo gives cyan). Weighting by saturation
   removes the need for a saturation floor, which is the constant that cannot
   be set once yellow is in the room — yellow's ring peaks near S=98 where red
   reaches 177.
5. **Colour says "warm or green" only.** Position separates red from yellow -
   with one veto: YELLOW is never answered when the whole crop's glow is red
   rather than amber (round 5, below). Colour can prove "not yellow"; it can
   never prove "red", because a yellow in front of a red-lit stage glows red.
6. **Decision.** Position and colour are weighted by how much evidence each
   carries. If they disagree, or the margin is thin, or no cell verifies, or
   nothing is emitting, the answer is **`UNKNOWN`** — a first-class answer, not
   a failure.

### The one thing step 1 structurally cannot see

Yellow is the middle lamp, so its flood erases the covers on **both** sides and
no three cells survive. Red and green flood one neighbour and read correctly
12/12 and 9/9. Measured on the approach clip: the whole yellow phase — 19
frames where the lamp is plainly lit — returned `UNKNOWN`, with every candidate
disc being the flooded region itself and **0 of 88 hypotheses** clearing the
contrast test. The premise "a cover is lighter than the body it sits in" had
simply stopped being true.

So when no stack is found, one last cue is tried: where the blown core sits in
the box, as a fraction of its height. The three cells sit at 0.22, 0.53 and
0.84 straight out of the housing geometry, and measured over the clip the core
lands at 0.18–0.26 for red, 0.49–0.51 for yellow and 0.78–0.83 for green.
Two gates gate it: something must actually be clipping the sensor (lit lamps
reach 253–254 at the 97th percentile across all 30 lit photos, unlit housings
183 at most), and the bright region must be *one lamp* rather than the whole
box.

**This path may only ever answer YELLOW**, and that restriction is why it is
safe to have. Position alone is not trustworthy here — on the two closest venue
photos the housing is cut off by the frame, the fractions mean nothing, and the
core lands at 0.32, nearer red's 0.22 than yellow's 0.53. Allowed to answer
freely it calls both of them RED confidently, turning two safe refusals into
two wrong answers; I measured that and did not ship it. Restricted to yellow it
refuses them instead, and the worst it can ever do is say YELLOW when the truth
was red, green or nothing — all of which mean "do not proceed" or cost one
needless stop. It can never produce the answer that actually hurts, which is
GREEN.

It is **not** the old "slice the box into three bands" bug, which rounded a
2.3:1 housing into two and lost the middle lamp entirely. Nothing is rounded: a
continuous centroid is compared against three known positions, and an answer is
given only when it is clearly nearest to one of them.

Two other repairs were tried first and both measured worse, so neither shipped:
anchoring the fit to the caller's box (no change at all, 35/38 either way), and
calibrating the threshold ladder on the housing rather than the whole crop
(fixed one photo, broke two others, 35/38 → 33/38).

---

## D. Suppressing false positives, and why there is no slider

A model trained on a clean room never sees an empty scene. It concludes a sign
is always present somewhere and picks whatever is most sign-like — at an
exhibition, a stage lamp, a phone screen or a person. In the record: a bare red
disc on black called `stop` at **0.86**, a green disc called `traffic_light` at
**0.63**.

| Layer | What it kills |
|---|---|
| ~1/6 of training frames with no sign at all | "there is always a sign somewhere" |
| unlabelled distractors in ~half the frames — lamps, lamp *pairs*, LED bars, screens | bare emitters |
| real venue hard negatives: the stage rig, people | show lighting, people |
| structural three-cell verification on the crop | stage lamps, people, anything tall |
| a light must read a state or it is not a light | everything not emitting |
| confidence threshold **measured** by an F1 sweep on real photos | guessed thresholds |

Not on that list: a slider you turn up when it gets chatty. Every layer changes
the representation instead. A vertical stack of *three* lamps is deliberately
never drawn as a distractor — that is the light's own silhouette.

Per-class paste sizes matter as much as the distractors. `traffic_light` is
pasted at 22–640 px and everything else at 22–260 px, because a ~50 cm housing
is ~260 px at 2 m and ~520 px at 1 m, while a 15 cm delivery sign can never
exceed ~200 px in a 720p frame. The previous 280 px cap on the light measured
0.97 at 260 px, **0.18 at 400 px and 0.00 at 500 px** — it vanished exactly
where the car has to stop. `tools/probe_signs.py` fails the build on that.

Every box — synthetic or hand-drawn — covers the **sign face or the lamp
housing and nothing else, never the post**, and never the halo either. Phase-2
distance is `real_height / pixel_height`, which is only constant if the box
always bounds the same physical object.

---

## E. Running any of this

Everything that cannot be regenerated is versioned under `assets/`; everything
under `dataset/` is derived and the whole directory is safe to delete.

```
assets/real/images/{close,far,med,very}/*.jpg   the venue photographs
assets/real/labels/{close,far,med,very}/*.txt   the hand-drawn boxes
assets/sign_cutouts/<class>/*.png               cutouts to paste, background removed
dataset/backgrounds/*.jpg                       frames to paste onto      (derived)
dataset/synth/                                  the composited training set (derived)
```

From a fresh clone:

```bash
pip install -r requirements.txt
pytest
python3 tools/eval_light_state.py --confidence 1.1   # the reader, hand boxes
python3 tools/eval_delivery_variant.py --trials 60   # needs dataset/backgrounds too
```

`dataset/backgrounds/` is the 513-frame background pile and is not in the repo;
`tools/extract_frames.py` rebuilds it from clips.

## F. Retraining, about an hour

On the training box (Ryzen 7 5700X, RTX 3060):

```bash
python3 tools/synthesize_dataset.py --count 4000 --preview 24
#   look at dataset/synth/preview/ first. If the boxes are not tight on the
#   signs, nothing downstream matters.
python3 tools/train_signs.py --base yolov8n.pt --name signs_n
python3 tools/train_signs.py --base yolov8s.pt --name signs_s
```

Copy **one file**, `models/signs_*.onnx`, to the Mac. Then there:

```bash
python3 tools/probe_signs.py  --model models/signs_n.onnx      # pass/fail gate
python3 tools/eval_signs.py   --model models/signs_n.onnx      # P/R, F1 sweep, latency
python3 tools/eval_light_state.py --model models/signs_n.onnx  # the reader, end to end
```

Keep the winner as `models/signs.onnx`. The dataset, the `.pt` weights and the
run logs stay on the training box and are gitignored.

Everything else:

| Tool | What it is for |
|---|---|
| `tools/inspect_signs.py` | one window, boxes, crop, masks, and *visible rejections* |
| `tools/label_boxes.py` | `cv.selectROI`-style labeller, `--prefill` for accept-or-fix |
| `tools/eval_light_state.py` | the state reader on the venue photos, filename ground truth |
| `tools/eval_delivery_variant.py` | letters vs silhouettes, sets `MIN_CROP_HEIGHT` |
| `tools/eval_signs.py` | real-photo P/R, F1 sweep, latency on this machine |
| `tools/probe_signs.py` | the gate: close range, bare emitters, and real-photo recall |
| `tools/extract_frames.py` | clip → frames |
| `tools/list_cameras.py` | which camera index is which |

`pytest` runs 54 tests against drawn fixtures in `tests/synthetic.py`, with no
camera and no model.

---

## G. What is measured, and on what

Kept deliberately separate, because this is the section the previous write-up
did not have.

### Verified on real photographs

**The traffic-light state reader**, on the 38 venue photos with hand-checked
boxes (`tools/eval_light_state.py --confidence 1.1`):

```
36/38 correct
                 RED   YELLOW    GREEN  UNKNOWN
       RED        12        0        0        0
    YELLOW         0        7        0        2
     GREEN         0        0        9        0
   UNKNOWN         0        0        0        8

lit photos read correctly   28/30  (0.93)
unlit photos read UNKNOWN   8/8
red read as green, or back  0
```

Both failures are **refusals** — close range, yellow lit, glow so heavy that
no three-cell stack can be fitted. Never a wrong state. The baseline in the
record was 6/28.

On the plan's own gate set — the 28 filename-labelled photos, excluding the ten
close-range ones whose state was read off the photo by eye and recorded in
`assets/real/images/close/states.txt` — it is **28/28**. Both failures are
close-range frames.

**The detector**, as it stands today: there is no model in this branch yet. Run
against the previous branch's model for comparison, `tools/eval_signs.py`
reports `traffic_light` recall **0.24** at its best-F1 threshold, and recall by
apparent height of **0.38 (90–180 px), 0.28 (180–320 px), 0.08 (>320 px)** —
an independent reproduction of the close-range collapse described above.
`tools/probe_signs.py` fails it on 228 counts.

### Verified only against synthetic renders

**The delivery-variant reader**. `tools/eval_delivery_variant.py` renders each
artwork at a ladder of apparent heights through the *training set's own*
degradation functions, onto real backgrounds, with ±4 % per-edge box jitter.
This is structurally optimistic in one way — the glyph templates come from the
same artwork the renders do — and pessimistic in another: it is a single-frame
score under motion blur up to 15 px and JPEG quality down to 38.

```
  px     n   sides   letter    fused   refused   wrong
   16   180    0.54     0.33     0.51      0.43    0.06
   28   180    0.72     0.58     0.69      0.28    0.03
   40   180    0.79     0.59     0.77      0.21    0.02
   60   180    0.88     0.86     0.86      0.13    0.01
   80   180    0.87     0.88     0.91      0.09    0.00
  110   180    0.92     0.97     0.96      0.03    0.01
  160   180    0.94     0.99     0.99      0.01    0.00
  224   180    0.91     0.97     0.97      0.03    0.00
```

Two findings, both of which settle arguments that were previously unsettled:

- **The silhouette survives further out than the letter**, and they cross over
  at about 80 px. That is the answer to the question the brief and the older
  write-up disagreed about, and the disagreement between the two pilots run
  while planning was entirely down to how the face was segmented — which is
  the real difficulty. Measured: without upsampling first, segmentation fails
  on **100 %** of crops under 64 px; with a tolerance from the crop's *global*
  standard deviation it fails on **100 %** at any size.
- **Wrong answers are rare everywhere and absent above 48 px.** What the reader
  does when it cannot tell is refuse.

`MIN_CROP_HEIGHT = 60`, where fused first reaches 0.95 and stays there — for a
15 cm sign on a camera that puts a 50 cm housing at 260 px from 2 m, about
**2.6 m**. (It was 110 px / 1.4 m until `SIDES_ONLY_MARGIN` landed; refusing
weak side counts took `wrong` to zero above 48 px, and the floor is set by
where wrong answers stop.) Two runs on different seeds agree on both 110 and the 80 px
crossover; individual rows move by about ±0.03, which is two standard errors at
180 trials, so the floor is pooled over every height at or above it rather than
read off a single row.

**This fails Gate 3 as the plan wrote it**, which asked for 0.95 by 40 px.
What that means in practice is narrower than "the crop reader does not work":
the car can name the bay from ~1.4 m and closer, and further out it says
`delivery_sign` and nothing more. If that is not close enough, the plan's own
alternatives stand — three detector classes with the measured 6–10 %
cross-confusion, or colour-coding the *letter* (not the face), which costs one
reprint and roughly doubles the usable range.

### Round 1 of training: measured, and it failed

Both sizes trained on 4000 composited frames, 2026-09-18.

| | `signs_n` | `signs_s` |
|---|---|---|
| `traffic_light` recall, real photos | **0.32** | 0.26 |
| ... of those with a lamp lit | **0.19** | 0.06 |
| latency, M2 Pro | 10 ms | 10 ms |

**Every miss is a lit lamp; every reliable hit is an unlit one.** Raw pre-NMS
scores at the light's location are *exactly 0.000* on lit photos and 0.960 on
the unlit one - an absent detection, not a weak one. On
`clips/light_approach.mp4` it found the light in 2 of 251 frames and called a
grey hoodie `delivery_sign` at 0.93 in 38 % of them.

Two things did work, and are worth keeping: the **close-range collapse is
fixed** (0.90-0.95 from 140 to 560 px, where the previous model measured 0.00
at 500 px), and **bare emitters score 0.00** on bright backgrounds.

The diagnosis, in four steps:

1. The *same cutout* pasted on a random background scores **0.96**; the photo
   it was cut from scores **0.30**. Same pixels - so the object is fine.
2. Not an NMS artefact: raw scores across all four classes at that location are
   0.000.
3. Keep the real housing, replace everything outside it with an ordinary room:

   | real pixels kept | box only | +0.5 box | +1 box | +2 box | whole photo |
   |---|---|---|---|---|---|
   | `med/red_head` | 0.96 | 0.89 | **0.00** | **0.00** | 0.30 |
   | `med/off_head` | 0.96 | 0.93 | 0.96 | 0.96 | 0.96 |

   The killer is the scene **one to two box-widths around a lit housing**.
4. Not darkness on its own - the venue averages 103 grey against the
   backgrounds' 138, but the `off` photos are just as dark and detect at 0.96.
   It is the **spill**.

Root cause: `bloom()` lit the cutout and never lit the room, so the model had
never seen a housing drowning in its own glow.

### What changed after that, and what it is worth

- `scene_glow()` now lights the room: a radial wash out to 1.1-3 housing
  heights, plus veiling glare across the whole frame scaled by apparent size.
- Backgrounds are darkened toward venue levels *before* the lamp is pasted.
- **37 hand-verified frames of the real clip** are mixed in at 20 % of the
  training set (`assets/clip_labels/`), with the person who walks through them
  as an unlabelled negative.

Measured against the round-1 model, which misses 87 % of real lit photos:

| composited lit lamps | mean confidence | missed |
|---|---|---|
| round 1: no glow, bright rooms | 0.95 | 0 % |
| + darkening | 0.95 | 0 % |
| + scene glow | 0.83 | 12 % |
| + veiling glare (current) | 0.72 | 18 % |
| *real venue photos* | *0.10* | *87 %* |

So the synthesis is meaningfully harder and still much easier than reality. It
narrows the gap; it does not close it. **The real frames are the lever, and 37
of them is not many** - shoot B is what finishes this.

### Round 2: the fix worked, measured the same way

Retrained on the same pipeline with `scene_glow`, darkened backgrounds and the
37 real clip frames mixed in.

| on real photographs, never trained on | round 1 | round 2 |
|---|---|---|
| `traffic_light` recall (gate check c) | 0.34 | **0.82** |
| ... of those with a lamp **lit** | 0.19 | **0.77** |
| best F1 | 0.47 | **0.84** |
| recall at 180-320 px | 0.39 | **1.00** |
| latency, M2 Pro | 10 ms | 8 ms |

| on `clips/light_approach.mp4`, 251 frames | round 1 | round 2 |
|---|---|---|
| light found | 2 frames (0.8 %) | **174 frames (69 %)** |
| ... and read a state | - | 160 frames (64 %) |
| hoodie called `delivery_sign` | 96 frames (38 %) | 26 frames (10 %) |

End to end, 35/38 photos correct, lit 27/30, unlit 8/8, no red read as green.
The detector now supplies 31 of 38 boxes itself, against 12 in round 1.

On the clip the reader now reports the full sequence — **GREEN → YELLOW → RED →
GREEN** — where before the yellow phase came back `UNKNOWN`. See the blown-core
fallback in section C.

**Runtime confidence: 0.35.** The F1 optimum is 0.50, but recall is worth more
than precision here because the crop readers are a second gate - measured, the
end-to-end score is 35/38 at 0.25, 0.35 and 0.50 alike, and the lower threshold
simply moves boxes from the hand-label fallback to the detector.

What is still wrong:

- **Nine photos still missed**, and they split in two. Four are the *right box*
  at low confidence (IoU 0.63-0.91, confidence 0.19-0.43) - a threshold
  problem. Five are genuine blanks: three very close green/yellow frames and
  two distant ones.
- **Check (a) now fails at 40-140 px.** Expected, and it is the check marking
  its own homework again: the training distribution moved toward dark glowing
  scenes, so bright easy pastes are relatively rarer. (c) is the number to read.
- **Bare emitters reach 0.83 on dark backgrounds.** The false-positive
  suppression only ever held in bright rooms; darkening the probe exposed it.

### The hoodie, and the one genuinely unsafe residue

Of the 26 remaining `delivery_sign` false positives on the clip, the variant
reader refused 21 - but five came back as a confident "A". All five arrived by
the same route: **"sides only - letter unreadable"**, with the harmonic beating
its runner-up by just 1.1 to 1.4x.

That path was the one place in the module where a single cue was accepted
without checking that it had spoken clearly. Measured over rendered signs: when
the side count is right its harmonic wins by 3.25x at the median; when it is
wrong, by 1.23x with a 95th percentile of 1.68x. `SIDES_ONLY_MARGIN = 1.8` sits
just above that, and **all 26 false positives are now refused**.

### Round 2's blind spot: a real close-up

Found on the footage shot 2026-09-22, and it is the reason the model "did not
work" when tested by holding the light up to the camera.

**Every class dies above roughly 200 px of apparent height, on real images
only.** Walking toward the detour sign:

| box height | 116 | 142 | 166 | 184 | 201 | 220+ |
|---|---|---|---|---|---|---|
| confidence | 0.92 | 0.92 | 0.78 | 0.15 | 0.27 | **0.00** |

Same for the light: 0.74 recall with the light at mid distance on a red-lit
stage, **0.08** (1 frame in 13) with it close up in the dark hall, where the
boxes run 290-450 px.

It is size, not blow-out. Scaling those same close-up frames down so the light
becomes small takes it from 0.00 to 0.44-0.65 — identical pixels, different
apparent size.

And `probe_signs` check (a) passes at 220-640 px, because it pastes cutouts
through the compositor. The synthetic-versus-real gap that made round 1 blind
to lit lamps is still there; it has just moved to close range. The 37 real clip
frames that fixed round 1 span **89-359 px**, so the model has never seen a
real large one.

Lowering the runtime threshold does not help: the confidences are 0.00, not
0.2. The fix is real close-up frames, which is what
`assets/real/images/lightning/` and `.../signs/` now hold.

### Round 3: close range fixed, far range traded away

Trained on the 63 hand-labelled stills from the 2026-09-22 shoot, wired in via
`assets/real_train/`.

| recall by apparent height, on the untouched 38 venue photos | round 2 | round 3 |
|---|---|---|
| far, 134-136 px | 4/8 | **1/8** |
| med, 285-302 px | 18/18 | 18/18 |
| close and very close, 321-374 px | 7/12 | **12/12** |
| end to end, lit photos read correctly | 27/30 | **28/30** |

**The close-range blind spot is closed.** It was also a straight trade: the far
band went the other way, and not because it fell below the threshold - it is
1/8 even at 0.15 and the raw scores are 0.00.

The cause is the training size distribution, measured over 370 boxes:

| band | round 3 | after the fix below |
|---|---|---|
| 0-90 px | 20.0 % | 32.4 % |
| **90-180 px** | **10.3 %** | **20.0 %** |
| 180-320 px | 27.0 % | 20.4 % |
| 320+ px | 42.7 % | 27.3 % |

Every new still is a close-up, and `real_sample` can only make an object
*larger* - it crops into a frame and blows the crop up, so it cannot zoom out
past the frame's edge. The 90-180 px band, which is exactly where the eval's
distant photos sit, was left at a tenth of the training boxes.

`real_patch_paste` fixes it by lifting the object plus a couple of box-widths
of the scene around it - which is where its spill lives - and pasting that at a
smaller apparent size onto a different background. It is a better small-object
sample than an alpha cutout, not a worse one, because the spill is precisely
what a cutout throws away and what round 1 died without. The patch is matched
to the destination's exposure at its rim and faded radially rather than
rectangularly; a square seam is a shortcut a detector will learn in place of
the object.

Two other things found and fixed while wiring the stills in, both of which had
been silently corrupting round 2's real data as well:

- real frames were getting the *full* synthetic degradation on top of the real
  camera's, so a red-lit stage pushed another 1.45x in gain arrived as a white
  rectangle with a box around it. Photographs now get a gentle pass.
- `real_sample` resized an arbitrary window to 1280x720 without preserving
  aspect, stretching every object. Traffic lights came out wider than tall.
  After the fix the augmented aspect is median 2.12 with 97 % between 1.8 and
  3.0, against real housings at 2.2-2.45.

### Round 4: the trade resolved

`real_patch_paste` plus the spotlight label removed from `cam0_000210`.

| on the untouched 38 venue photos, at the runtime threshold 0.35 | r2 | r3 | **r4** |
|---|---|---|---|
| far, 134-136 px | 5/8 | 1/8 | **6/8** |
| med, 285-302 px | 18/18 | 18/18 | **18/18** |
| close and very, 321-374 px | 8/12 | 12/12 | **12/12** |
| total | 31/38 | 31/38 | **36/38** |

Recall **0.95**, best F1 **0.973**, and **zero false positives** anywhere in the
sweep from 0.10 to 0.45 - so the runtime threshold sits in the middle of a wide
plateau rather than on a cliff. Latency 8 ms.

`probe_signs` check (c), the only one that cannot mark its own homework,
**passes for the first time**: 36/38, recall 0.95, lit 0.94, against a 0.9
floor. Bare emitters 0.00.

End to end the detector now supplies **36 of 38 boxes itself**, against 12 in
round 2, and on the approach clip the hoodie false positives are gone entirely
- 26 `delivery_sign` boxes in round 3, none in round 4.

**The one hole: yellow at mid range.** On the clip's yellow phase (f55-73) round
2 scored 0.92-0.93 and rounds 3 and 4 score 0.00. It is not a size problem - the
light is 110 px at f54 and 151 px at f76 and both are found at 0.90 - it is the
yellow appearance, which is the thinnest state in training: 2 of the 37 clip
frames and one group of stills. The venue photos do not show it because their
yellows are read 7/9 by the state reader either way.

Note also that `probe_signs` still exits non-zero on check (a), entirely at
40-80 px. A 40 px housing is 17 px wide, which is below anything the state
reader can work with - it needs roughly 90 px to find three cells. That band is
testing something the system cannot use.

### The second car-camera shoot: round 4 measured on it, before retraining

Held out in `assets/real`: `signs2/` (40 frames, stop and detour) and
`lighting2/` (frames 330-520 of the light shoot, the second room, states in
`states.txt`). The rest of the light shoot is `assets/real_train/lighting2/`.
Split by clip segment, never by frame - frames 10 apart are near-duplicates.

| round 4, 96 photos at 0.30 | |
|---|---|
| detour | **17/17** |
| stop | **21/23** - both misses blurred close-ups cut by the frame edge |
| traffic light found | 52/56 |
| false positives | 3 `delivery_sign` on stage fixtures, 1 `traffic_light` |
| yellow in `lighting2/` found by the detector | **0/2** (read correctly on hand boxes) |

The first honest number for `stop` and `detour`, and both are strong.

**Stage fixtures are `delivery_sign`.** Switched off, the venue's round LED cans
are a pale disc with dark marks inside, which at 20-80 px is what a delivery
sign is. Pasted into dark frames the round-4 model calls them `delivery_sign`
at >= 0.35 in 30-52 % of pastes at 24-80 px, up to 0.95. Harmless today - the
policy never acts on `delivery_sign`, and every one seen on the car was under
the variant reader's 60 px floor and refused - but real. The fixtures are now
cut out in `assets/distractor_cutouts/` and pasted unlabelled; car-camera
footage of them is the better fix and is still to be shot.

**Red read as yellow, once.** `lighting2/cam0_000370`: the cell fitter put a
phantom cell in the glow above the red lamp with its radius inflated (21 vs
16), so the red lamp became the middle cell. Colour only separates warm from
green, so red versus yellow is position alone. The right stack was the runner-up,
0.188 against 0.205. With states written for the 92 training frames too, the
reader alone is 247/274 over every frame and box source, and this is the only
misread in the dangerous direction; the rest are yellow read as red (a needless
stop) or refusals.

A constraint that the fitted stack's implied housing must match the box
(|edge offset| <= T box heights) was simulated and **not shipped**: T=0.15
fixes it and four red refusals (252/274, no dangerous reads), but 370's offset
is 0.17, T=0.10 and T=0.20 are both worse, and many correct green reads rest
on wild fits the rule would reject without its fallback. That is a threshold
fitted to one frame. What makes it tolerable is the tracker: a state commits on
3 of the last 5 readings and UNKNOWN never votes, so an isolated misread between
two reds cannot flip a committed RED - and 370 is isolated.

### Round 5: the fixtures and the second shoot's training frames

Trained with `assets/distractor_cutouts/` and `assets/real_train/lighting2/`.
On the 96 held-out photos:

| | r4 | **r5** |
|---|---|---|
| best F1 | 0.947 @ 0.30 | **0.990** @ 0.20 (0.984 at the runtime 0.35) |
| traffic light found | 52/56 | **56/56** |
| stop / detour | 21/23, 17/17 | **22/23**, 17/17 |
| `delivery_sign` false positives (stage fixtures) | 3 | **0** |
| probe (c) real-photo recall at 0.35 | 0.93 | **0.98** |
| approach clip, yellow phase f55-73 found | **0/19** | **19/19**, 18 read YELLOW |
| latency | 8 ms | 10 ms |

The clip now reads GREEN -> YELLOW -> RED -> GREEN end to end. Probe (a) still
fails, almost entirely at 40 px; the one large-size miss is an unlit housing at
560 px, which the policy ignores either way.

**Red read as yellow came back, twice in a row.** With r5's boxes both
`lighting2/cam0_000360` and `370` read YELLOW - 360 flipped from RED on a 2 px
change of box. Consecutive samples 10 frames apart are about a third of a
second of yellow, enough to pass 3-of-5, so the tracker no longer covers it.

The fix is colour, used in the one direction it is reliable in. Red versus
yellow had been position alone; the whole crop's glow says whether it is red or
amber, and every red lamp in every shoot measures >= 0.89 red, yellows under
ordinary light <= 0.73. Yellows in front of the first shoot's red-lit stage
measure red too, so the glow cannot prove red - but it can prove *not yellow*.
`read_light_state` now only answers YELLOW when the glow is under 0.80 red;
otherwise RED. Only when the lamp's own halo voted warm: a colourless halo in
front of a red stage (a green lamp the fit got wrong, `lighting/cam0_000050`)
would otherwise have become RED at a green.

| state reader, every grade | red read as yellow | red <-> green |
|---|---|---|
| before | 2 (r5 boxes), 1 (r4 boxes, hand) | 0 |
| after | **0** | **0** |

Venue plus second shoot 54/56 with either model and with hand boxes; every
yellow is still read. The cost is yellow read as red under the red stage
(`lighting/cam0_000120`), a needless stop before a lamp that turns red next.

### Not measured at all yet

- **`delivery_sign` has no real photograph.** Every instance the code has
  seen is a paste of the artwork, so its recall on the car is unknown, and its
  only real-photo numbers are false positives.
- Nothing has been measured in a room other than the venue.

---

## H. What to shoot

The count of venue photos is not the problem; the variety is. All 38 are one
camera, one exposure, roughly one spot, one room, one evening. Three rules,
then the list:

1. **Shoot with the car's own camera at the car's own settings** (auto white
   balance off, same resolution). The blown-out white core is a property of
   *that sensor at that exposure*; a phone photo is processed differently and
   would teach the model and the reader the wrong thing.
2. **Shoot video, not stills.** Thirty seconds of walking is ~900 varied
   frames; `tools/extract_frames.py` turns a clip into a folder.
3. **Two different jobs, two different setups.** Do not try to get both from
   one clip.

| | What | Roughly |
|---|---|---|
| **Shoot 3** | **Negatives.** No sign and no light anywhere in frame: people at 1–4 m, phone and laptop screens, the stage rig alone, coloured lamps. The single most direct fix for a false positive at the show. | 10 min |
| **Shoot 2** | **Real validation.** The light in situ, 4 states × 4 lighting conditions (room lights on / off / stage and coloured lights on / a torch or coloured lamp in frame), ~20 s walking 5 m → 0.5 m each. Vary camera height and yaw *within* the walks. Ambient light is the axis that is completely missing today, and it is what the old state reader broke on. | 40 min |
| **Shoot 4** | **The printed signs.** Stop, detour and the three delivery signs, mounted, 0.5–5 m, straight on to ~45°, in two different rooms. Until this exists there is no honest number for `stop` or `delivery_sign`. | 20 min |
| **Shoot 1** | **Cutout harvesting.** The light against a plain, evenly-lit backdrop — then background removal is nearly automatic instead of the evening of manual cutting the existing 58 cutouts cost. Four states × (one slow orbit at ~1.5 m + a walk from 4 m to 0.3 m). | 20 min |

If time is short, that is the order of value. The existing cutouts are usable;
the existing negatives and lighting variety are not.

Backgrounds want **variety, not quantity** — 200 frames across six rooms
generalises where 1 000 frames of one room teaches that room. No background
frame may contain a real sign: an unlabelled sign actively teaches that the
sign is not a sign.

---

## I. Phase 2: running it live, as a speed constraint

Nothing in the readers or the class list changed. What was added is a thread,
a tracker, a policy and a GUI pane, arranged so the control loop never waits
on any of it.

```
sign camera (index 1/2/3, never 0)
   └─ SignWorker thread ─ SignDetector ─ SignTracker ─ render_overlay
            │  publishes one immutable SignSnapshot behind a lock
control loop: snapshot = worker.get_snapshot()          ← the only added cost
              constraint = SignPolicy.update(snapshot)   ← pure arithmetic
              throttle *= constraint.scale; steering frozen while .hold
```

**Tracker** (`src/perception/sign_tracker.py`). Detections get an id by IoU
(same label, ≥ 0.2) and keep it through a gap of up to 0.6 s. A track is
*confirmed* when seen in 3 of the last 5 updates; a light's state and a
delivery letter *commit* when 3 of the last 5 readings agree. `UNKNOWN` never
commits and never votes against a committed state, so one blown-out frame
cannot flip RED to GREEN. A light with no committed state is not actionable at
all. Distance is `real_height × focal / box_height` with the printed signs at
0.43 m, the housing at 0.50 m, and the focal length from the 63.3° HFOV and
the frame width the camera actually delivers. With HFOV 63.3° at 1280 px, a
260 px housing reads 2.0 m, matching the anchor in §D. Boxes touching the top
or bottom edge are clipped and do not update the distance.

**Policy** (`src/input/sign_policy.py`). Same shape as the sonar's
`obstacle_scale`: a stop sign or a RED light ramps the throttle from
1.0 at `_SLOW_DISTANCE_M` (3 m) to 0.0 at `_STOP_DISTANCE_M` (2 m as designed,
1.5 m after the first car test on 2026-09-23). A stop then holds for 0.5 s and dwells to 3.5 s
total, distance ignored once holding so jitter across the line cannot reopen
the ramp. YELLOW only slows: the same ramp, floored at the crawl speed by the loop,
and never a hold; a yellow that turns red while the car can still stop gates
like any red. A light gate forms anywhere inside 3 m; a light revealed by a curve already
inside the line holds at once (the car stops late rather than not at all),
and only one first seen under 0.8 m, where braking would put the car under
it, is waved through. Stop signs confirm on 2 of 3 frames instead of 3 of 5
for the same reason and waits for a committed GREEN; a stale snapshot,
losing the track for 2 s, or the 10 s ceiling also end it. Every hold ends.
A sign is consumed by track id once acted on, every stop id still in view at
release is consumed with it, and for 3 s after any release nothing within
2.5 m may start a new one. E-stop drops the active hold but keeps what was
consumed. Detour and delivery signs are reported, never acted on.

**Why the ramp has a floor.** On the car (2026-09-23) the ramp parked the
car above the line: as the scale fell the PWM dropped under what the motors
need to move, the car stalled at ≈2.4 m estimated, the policy never saw
≤ 2 m, never held, never released, and with throttle still ≈0.25 the brake
lights stayed off. Two fixes, no knobs: while SLOWING the loop never sends
less than `Autonomous.crawl_speed()` (the sharp-bend floor, `min_speed ×
0.75`), so the car keeps creeping to the line; and an approach that has not
reached the line after 8 s holds where it is, so every approach ends in a
stop and a release. The policy logs every commit, hold and release at INFO.

**Steering while stopped.** `obstacle_scale` zeroes throttle but the PID keeps
chasing threshold jitter on a static frame, and the firmware's minimum move
kicks the wheel. While `constraint.hold` is true the loop sends the same
steering value every iteration; the PID keeps running with its output
discarded, and there is no reset on release, so no derivative spike. This
relies on `ki ≈ 0` (the Config default).

**Constants, and why there are no sliders.** 3 m / 2 m are the stopping
distances the car can honour at base speed; 0.35 is the measured confidence
(§G); the heights and HFOV are physical. Nothing here is tunable from the GUI.

**Bench replay.** `SIGN_CAMERA` replaces the camera index list and accepts an
index or a video path:

```bash
SIGN_CAMERA=clips/light_approach.mp4 python3 src/main.py
```

With the car e-stopped, watch the third tile: GREEN → YELLOW → RED committing
with hysteresis, the distance falling, the policy going SLOWING → WAITING_GREEN
→ CLEAR, and every rejected housing drawn thin and grey with its reason.

**Without a model or a camera.** The model is not committed: copy the winner
to `models/signs.onnx` (gitignored). Missing model: header chip "Signs: no
model at models/signs.onnx" in orange, the pane shows live frames under a red
banner, the car drives exactly as before. Missing camera: "Signs: NO CAMERA
(tried 1,2,3)" in red, retried every 2 s, the car drives exactly as before. A
model copied in after start is picked up on the next start only.

Distances are nominal until the sign heights and the HFOV are confirmed on the
car's own camera; the GUI prefixes them with "≈".
