# Sign and traffic light detection

## How it is split

A neural net answers **"is there a sign, and where"** — a question about shape
and context. Deterministic, unit-tested code answers **"which lamp is lit"** and
**"how far away is it"** — questions about colour and geometry that we can get
exactly right, test without a camera, and fix without retraining.

| File | Does |
|---|---|
| `src/perception/sign_detection.py` | ONNX YOLO through onnxruntime. Synchronous, no camera, no thread. |
| `src/perception/traffic_light.py` | Lamp state from a crop. No model, no config. |
| `src/perception/tracking.py` | Track identity, N-of-M confirmation, pinhole distance. |
| `src/perception/sign_worker.py` | Owns the sign camera and the detection thread. |
| `src/input/traffic_policy.py` | Confirmed tracks into a throttle ceiling. |

Everything runs on **its own thread with its own camera**. The control loop's
entire cost is one lock acquisition. This is not optional: the Mega's watchdog
is 500 ms and the motor command that feeds it is emitted once per control-loop
iteration, so anything that can stall that loop stops the car.

If the model file is missing, onnxruntime is not installed, or the sign camera
never appears, detection reports itself unavailable and the car drives exactly
as it did before. The GUI says so loudly; the throttle is untouched.

## Is reading colour safe across venues?

Mostly yes, and the design is hedged for where it is not.

**A lamp is an emitter, not a reflector.** A red painted sign only looks red
because of the light falling on it, so its apparent colour swings with the
illuminant. An LED makes its own light: its hue is a property of the diode, not
of the room. That makes lamp colour far more stable across venues than sign
colour would be.

**Auto white balance is therefore disabled** on the sign camera
(`src/perception/sign_worker.py`). AWB assumes one illuminant and shifts the
whole image to make greys grey — in a warm room it pushes everything blue,
dragging red toward magenta and green toward cyan. It would be *introducing*
the venue dependence we are trying to avoid. Auto exposure stays on, because a
fixed exposure tuned for a bright room goes black in a dark one.

**Colour is read from the ring, never the centre.** On the real camera a lit
lamp is so much brighter than the scene that its core clips to pure white —
measured saturation there is 5 out of 255, meaning no colour at all. Every bit
of the colour survives in the ring just outside. Reading the lit region itself
found nothing for yellow and gave up.

**Yellow is the worst case and sets the saturation floor.** Red and green each
sit near a corner of the RGB cube; yellow already has two channels high, so it
washes out to white far sooner. Its ring peaks at S=98 where red reaches 177
and green 138 — a saturation threshold chosen for red excludes yellow entirely.

**Colour is also what proves something is lit at all.** An unlit cover is
reflective grey plastic: it can be the brightest thing in the crop and pass
every brightness test. Measured, unlit covers carry ~0.2% coloured pixels
against a lit lamp's 20–100%, so a bright region with no colour is reported as
`UNKNOWN` rather than by position.

**Colour never decides alone.** Its weight scales with how much saturated colour
the lamp actually shows, so a washed-out lamp is largely judged on position
instead. And if colour and position disagree, the answer is `UNKNOWN` and the
car does not act on a guess.

If you ever want to drop colour entirely, set `w_hue = 0.0` in
`read_light_state`. You would lose only the case where the bounding box is
cropped or loose enough that position is unreliable.

## Only two knobs

`enable_sign_detection` and `sign_confidence` in `src/config.py`. That is
deliberate — the previous attempt shipped seven sliders and was still unreliable.

Everything else is either a **physical constant** with a documented meaning
(`_STOP_DISTANCE_M`, `_DWELL_S`, `SIGN_CAM_HFOV_DEG`, `REAL_HEIGHT_M`) or is
**derived from the image at runtime**. The traffic light reader in particular
has nothing to tune: it scales its brightness threshold off the crop's own
maximum, derives the lamp count from the aspect ratio, and weights its two
signals by how much evidence each actually carries.

`sign_confidence` should not be guessed either — `tools/eval_signs.py` sweeps it
and reports the F1 at each value.

## Designing the signs

These cost nothing while the artwork is still changeable and are expensive to
retrofit.

1. **Distinct silhouette per class.** Shape survives low resolution and bad
   white balance far better than colour. Octagon for stop, clearly not an
   octagon for detour.
2. **At least 15 cm.** A sign needs roughly 20 px in the model's 640 px input:
   `size_m >= (20 / 640) * 2 * D * tan(HFOV / 2)`. At 65° and 3 m that is ~12 cm,
   so 15 cm leaves margin.
3. **Matte print. Never glossy or laminated.** Specular blowout destroys both
   detection and colour reading, and it happens exactly when the sun is behind
   you.
4. **Thick, high-contrast border**, white on a saturated field. Strong gradients
   are what make a sign findable at distance.
5. **Avoid the traffic light's red/amber/green** as a sign's dominant field
   colour where you can. The detector is shape-driven and copes either way, but
   it keeps the debug views legible.
6. **Mount flat, perpendicular to the road, near camera height.**
7. **Signs that differ only in fine detail need their side counts spread wide.**
   A 15 cm sign is 25 px in the model's input at 3 m and 38 px at 2 m. At 25 px
   a pentagon and a hexagon are near-identical blobs, while a triangle, a square
   and a hexagon are still obviously different. So choose 3/4/6, not 5/6/7/8 —
   and avoid the octagon already taken by the stop sign. Going to 25 cm signs
   turns 25 px at 3 m into 42 px, which is the cheapest accuracy available to
   the project.

   If the shapes cannot be spread out, split the job: detect a single
   `delivery_sign` class — easy and reliable at range — then identify which one
   from the crop once the track passes ~60 px, the same two-stage split used for
   the traffic light. `tools/eval_signs.py --real-height` reports recall per
   apparent size and will tell you which of the two you need.
8. **Hand over vector artwork (SVG/PDF).** `tools/synthesize_dataset.py` renders
   cutouts from it directly. This single deliverable is what turns dataset
   creation into an afternoon.

## Bringing it up

```bash
pip install -r requirements-dev.txt

python3 -m pytest                                  # logic, no hardware needed
python3 tools/list_cameras.py --preview            # which index is which
python3 tools/train_signs.py --bootstrap           # stock COCO, proves the chain
python3 src/main.py
```

Set `DRIVE_CAM_INDEX` and `SIGN_CAM_INDEX` in `src/main.py` from what
`list_cameras.py` reports.

The bootstrap model is COCO-pretrained: it sees stop signs and traffic lights
only, and poorly on scale models. It exists to exercise threading, tracking,
light reading, braking, and the GUI on real hardware before a custom sign
exists — not to drive the track.

### Calibrating distance

Braking is expressed in metres, so the camera's focal length matters.

```bash
python3 tools/calibrate_sign_camera.py --distance 2.0 --height 0.15
```

Paste the reported `SIGN_CAM_HFOV_DEG` into `src/perception/tracking.py`, and
update `REAL_HEIGHT_M` there once each sign is built and measurable.

Also measure `_SPEED_PER_THROTTLE` in `src/input/traffic_policy.py`: drive at a
known throttle for a few seconds and divide the distance by the time. It sets
both the deceleration ramp and the dead reckoning.

## Collecting the data

Three separate piles, and they are not the same thing.

### 1. Sign cutouts — tiny, and mostly not photographs

RGBA PNGs in `assets/sign_cutouts/<class>/`. These are what gets pasted.

- **Flat printed signs** (stop, detour, delivery): export straight from the
  artwork. One clean PNG per class is enough. Do not photograph them — the
  source file is already perfect, and `synthesize_dataset.py` applies all the
  angle, scale, blur and lighting variation itself.
- **The traffic light** is a 3D object with lit lamps, so it does need
  photographing: ~6–10 shots, background removed, including **red-lit,
  yellow-lit, green-lit and all-off**. All of them are labelled just
  `traffic_light` — the detector finds the housing, and state is read
  separately, so it must recognise the housing whatever is lit.

### Where the box goes — housing only, never the post

Every box, in every cutout and every hand-drawn label, covers **the sign face
or the lamp housing and nothing else**. Not the post it stands on.

Two things break otherwise. Distance comes from `real height / pixel height`,
which is only a constant if the box always covers the same physical object — a
post of varying length destroys it. And the light reader splits the box into
equal lamp bands, which only lines up when the box *is* the housing.

### 2. Backgrounds — a few hundred, deliberately varied

```bash
python3 tools/capture_dataset.py --auto            # from the sign camera
python3 tools/extract_frames.py laps.mov --output dataset/backgrounds
```

These teach the model **what is not a sign**. Trained on clean voids, a
detector fires on anything sign-shaped in a real room.

**Shoot them with the signs removed from the track.** Every unlabelled sign in
a background frame becomes a lesson that this sign is not a sign, which is
worse than not having the frame at all.

**Film whatever the model gets wrong.** If it calls a person a traffic light or
a phone screen a sign, shoot twenty seconds of exactly that and add the frames
here. This is the most direct fix there is for a false positive: a sixth of
these images are generated with no sign in them at all
(`--negative-fraction`), so the model gets told plainly that the thing it fired
on is nothing. Without any such images it never sees an empty scene, concludes
a sign is always present somewhere, and picks whatever is most sign-like.

**Variety matters far more than quantity or fidelity.** Since the test space
and the presentation venue differ, do not shoot 1000 frames of one room — shoot
200 across six: the test track, corridors, outdoors, different times of day,
lights on and off. A model that has seen many rooms generalises to an unseen
one; a model that has seen one room learns that room. If you can get even 50
frames from the real venue beforehand, they are worth a lot — and regenerating
plus retraining takes about an hour, so it can be done the day before.

### 3. A real validation set — ~100 photos, the only hand-labelling

Photograph the actual signs on the actual track: varied distance (0.5–5 m),
angle (straight on to ~45°), height, and lighting. Label with Roboflow, CVAT or
Label Studio, export **YOLO format**, and put it in `dataset/real/`.

This is the only number worth trusting — the synthetic val split flatters
itself. Recall matters more than precision here: a missed stop sign is a run red
light, a false one is a needless pause.

### iPhone to Mac

Shoot **video, not stills** — 30 seconds walking around a sign is 900 varied
frames. AirDrop the `.mov`, then `tools/extract_frames.py`.

For stills, iPhone writes **HEIC**, which OpenCV cannot read. Either set
Settings → Camera → Formats → **Most Compatible** before shooting, or convert:

```bash
sips -s format jpeg *.HEIC --out converted/
```

Bulk import without AirDrop: plug the phone in and use **Image Capture**
(built into macOS) to import straight to a folder.

### Then train

```bash
python3 tools/synthesize_dataset.py --count 1000 --preview 12
python3 tools/train_signs.py --data dataset/synth/data.yaml
python3 tools/eval_signs.py --dataset dataset/real
```

**A thousand images is enough** for a handful of rigid, flat classes fine-tuned
from COCO weights, and training time scales linearly with the count. Measured on
an M2 Pro at 640 px, roughly 7 s per epoch per 100 images, so ~1000 images is
about 70 s per epoch. Training stops early once the score plateaus, which on
synthetic data usually lands around 30–40 epochs — call it 45 minutes.

`train_signs.py` selects **MPS** on Apple silicon. Ultralytics' own
auto-selection returns `cpu` there even when MPS works, which is roughly **5×
slower** — if the banner does not say `Training on mps`, something is wrong.
Other levers, in the order worth reaching for: fewer images, then `--imgsz 480`
(about 1.8× faster, at some cost to spotting distant signs), then `--cache`.

Check the `--preview` images before training — if the boxes do not sit tightly
on the signs, nothing downstream will work.

This **fine-tunes** from YOLOv8's pretrained weights rather than training from
scratch. The network already knows edges, shapes and textures from millions of
images; we are only adding a few classes on top, which is why a few thousand
synthetic images suffice instead of a million real ones. The resulting model
replaces the bootstrap COCO one entirely and detects **only** our classes.

## Behaviour

```
stop sign     > 3 m  nothing
              3 m    ceiling ramps down with distance
              0.6 m  hold, settle, dwell 3 s, then release and never re-trigger

traffic light red or yellow within 3 m, same ramp, hold at 0.6 m
              held until a confirmed green
              unreadable while held: stay held
              unreadable on approach: creep and keep looking

detour        detected, tracked, reported. No steering behaviour yet - the
              routing layer will consume TrafficConstraint.detour.
```

**Nothing re-triggers.** A stop sign is remembered by track ID once dwelt at, so
it stays ignored for as long as the cart keeps seeing it — plus a 2 s re-arm
window covering the case where it blinked out of view during the halt and came
back with a fresh ID. On a later lap it arrives as a genuinely new track and
correctly stops the cart again, however long the lap took. A light needs no such
memory: it is released by a confirmed green, and while it stays green no new
gate forms.

**A light is only obeyed if the cart can still stop for it.** A gate forms only
while the cart is further out than the stop distance. Past that point with
nothing committed it has effectively been waved through, and a light changing
then would otherwise halt it parked on top of the crossing.

Three asymmetries are deliberate:

**A stop is latched.** With a forward camera and a roadside sign, the sign
leaves the frame *before* the cart reaches the line, so once a stop is committed
the approach continues on dead reckoning. Consumption is keyed on track ID, so
the same sign cannot trigger twice.

**A light is not.** A gate that only opens by *seeing* green must not stay shut
once frames stop arriving — it could never reopen. So a stale detector releases
a light gate, while a committed stop still completes: the stop is bounded and
self-terminating, the wait is not. Every hold also has a 20 s ceiling.

The policy returns a **speed ceiling**, applied as `min(throttle, cap)`. It
never touches steering, and it only applies in autonomous mode — but it is
computed and displayed in manual too, so you can drive the track with the pad
and watch it approach, hold and release without it fighting you for the throttle.
