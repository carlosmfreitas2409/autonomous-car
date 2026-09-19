# Sign and traffic-light detection

Phase 1. The car has to recognise five things: a **stop** sign, a **detour**
sign, three **delivery** signs (pentagon B, hexagon A, heptagon C — white,
thin black outline, big black letter), and a **traffic light**.

Nothing here touches how the car drives. `src/main.py`, `src/gui.py`,
`src/config.py`, `src/input/autonomous.py`, `src/arduino.py` and the firmware
are untouched, and no new `Config` field exists.

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
5. **Colour says "warm or green" only.** Position separates red from yellow.
6. **Decision.** Position and colour are weighted by how much evidence each
   carries. If they disagree, or the margin is thin, or no cell verifies, or
   nothing is emitting, the answer is **`UNKNOWN`** — a first-class answer, not
   a failure.

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

`MIN_CROP_HEIGHT = 110`, where fused first reaches 0.95 and stays there — for a
15 cm sign on a camera that puts a 50 cm housing at 260 px from 2 m, about
**1.4 m**. Two runs on different seeds agree on both 110 and the 80 px
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

### Not measured at all yet

- **No real photograph of any printed sign exists anywhere in this project.**
  Every `stop`, `detour` and `delivery_sign` instance the code has ever seen is
  a flat paste of one artwork file. Until that shoot happens, `assets/real`
  says nothing whatsoever about those three classes, and this write-up will not
  round up.
- No model has been trained on this branch, so there is no detector number for
  the new class list, and no `n`-versus-`s` decision.
- Nothing has been run on the car's own camera.

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
