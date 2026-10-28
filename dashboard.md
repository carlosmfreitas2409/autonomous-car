# Operator dashboard

A web page on a second laptop that shows what the cart is doing and lets the
operator take over. The cart's own laptop rides on the cart, so nobody can
read the tuner while it drives.

The page is in Brazilian Portuguese: the dock card faces strangers at the
event, and the truck's voice tells them to "tocar em Liberar no painel". Code
and this document stay English; the labels below are quoted as shown.

The dashboard never decides anything. Every number on it is something the
loop already computed: the lane tracker, the sign tracker and policy, the
mission, the path, the sonar guard, the lights and the firmware's `T` line.
Anything the car does not know is shown as absent, never estimated.

---

## Run it

On the cart laptop, once (needs internet):

```sh
pip install -r requirements.txt     # adds aiohttp
cd web && bun install && bun run build
```

Then as usual:

```sh
python src/main.py                   # dashboard on http://<cart-ip>/
```

Without aiohttp, without `web/dist`, or with the port taken, the car starts
and drives as before. The log says why the dashboard is missing.

The car boots stopped. Set the letters, then click Autônomo.

**Frontend development:** `cd web && bun run dev` proxies `/ws` to a cart or
replay on `127.0.0.1:8000`. `bun run test` runs the unit tests.

---

## Driving modes

One switch in the header: **Autônomo | Manual | Parar** (Retomar while stopped). Exactly one is lit,
and every change is one click, with no confirmations.

| Action | How | Rule |
|---|---|---|
| Stop | Parar, `Space`, PS5 Triangle, tuner | Any console may stop. Always wins. |
| Drive autonomously | Autônomo | From Stop it resumes; from manual it hands back. |
| Drive manually | Manual | The PS5 on the cart drives. **Latched**: stays manual until Autônomo or Parar. From Stop it resumes. |
| Path | Normal / Reto / Desvio | Same as PS5 R1/L1. |
| Letters | De / Para pickers | While stopped, or parked at the dock it last delivered to (clearing both still needs a stop). |
| Release the truck | Not from the dashboard: shutting the lid (cargo node), or the tuner / PS5 Cross | The dock card only shows what the truck waits for. After a failed weight check, shutting the lid again leaves without the weight check. |
| Hold the departure | "Esperar" on the dock card, tuner | While it checks itself before leaving; back to waiting at the dock. |
| Open the compartment | Ajustes dialog: "Abrir compartimento" | Only e-stopped, with a cargo node; until the lid is lifted, 10 s at most. |
| Speed | Ajustes dialog: top speed, curve speed, ±5 % a click | Any time, while driving too. Same numbers as the tuner's Base/Min Speed. Top speed also caps manual driving. |
| Steering PID | Ajustes dialog: Kp, Ki, Kd (± a step, or type a value) | Any time. Same numbers as the tuner's PID sliders, same ranges. |
| Lights | Ajustes dialog | Any time. Same as the tuner's Lights section. |

**The dashboard never drives.** Manual is always the PS5 on the cart, over
Bluetooth, so Wi-Fi never moves or stops the cart while you drive by hand.

**One e-stop, one mode.** They are two values in the loop (`estopped`,
`manual` in `src/main.py`). The tuner, the PS5 and every open dashboard set
them, and all of them show the same thing. Nobody "has control" and nothing
says which console made a change; the log line does.

Manual is fully manual: the sonar does not stop the cart in manual (as before).

---

## Screen

A light page that stays calm while things are normal. Color only means
something: red is stop and faults, amber a warning, green healthy. One blue
accent marks the current item: the target letter, the current step and road,
the path the autonomy steers. Autonomous and manual are told apart by name.

- **Header:** what the cart is doing, or why it is slow or stopped (worded
  from where the stop comes from, `drive.stop_source`, in red when it stops);
  the drive switch on the right.
- **World view** (3D, three.js): the cart from behind and above.
  - The cart is its own art, `web/src/world/cart.tsx`: the buggy's 3D
    model drawn as SVG from one chase camera, 40° down. The 3D view uses that
    same camera, scaled so the car is half a lane wide, and the art is laid
    over it. A test checks that the two line up. To change the car, replace
    that file with a new export; its `EGO_CAMERA` moves the view with it.
  - Its lights are the cart's: brake, blinkers and hazards, and the
    underglow in the light strip's colour (cyan autonomous, yellow manual,
    red stopped, off when the strip is off).
  - **The cart moves through the world** (`web/src/world/utils/motion.ts`):
    - What lies on the road moves with the cart's speed: the dashes flow
      past, and a sign or a fork it has passed slides by and fades behind it
      instead of vanishing when the sign camera loses it.
    - With no encoder, the speed is the cart's own estimate when it sends one
      (`EST_MPS_AT_FULL_PWM`), else the applied drive output through the
      speed table of `tools/synth_track.py`. That table holds placeholders
      until the cart is timed.
    - The cart turns with the lane it follows. The steering is a near
      bang-bang command, so on this cart it says little about the turn
      itself.
  - **Scale:** the drawing's unit is the lane, about half a metre wide
    (`LANE_REAL_M`). Sign ranges and sonar distances are drawn through it, so
    a sign 3 m away stands six lane widths ahead.
  - While stopped, the road the cart last saw stays, faded.
  - **The cart is never drawn sideways on its own road.** The cart's art
    cannot yaw, so whatever angle the road is laid at beside it is read as
    the cart sliding. The road therefore runs with the cart where the cart
    is standing on it, the bend eases in a lane width ahead
    (`roadCurve` in `web/src/world/utils/road.ts`), and behind the cart it carries
    straight back.
    - **The angle comes from the offset, not from the lane ahead**
      (`LaneHeading` in `web/src/world/utils/lane-reader.ts`): heading is the rate the
      offset crosses the cart, so it is taken from the offset and the
      distance driven. It cannot then disagree with the offset the way a
      separate fit could.
    - The fit's own slope is thrown away. The camera sees about 24 cm of road
      on a 50 cm lane, and over that baseline a quadratic's slope and bend
      terms are degenerate, so a real bend leaked out as a slope. Measured
      over the day-10 lap it ran to 24°, had r² 0.00 against the cart's
      actual drift across the lane, and integrated over 38 m claimed the cart
      had wandered 6 lane widths sideways when it had held to 0.29. Drawing
      from the offset instead: 3.4° at the 95th percentile, 7.0° at worst.
    - The bend survives that cut, because it is read over the whole baseline
      rather than at one end of it: on the same lap the known straight reads
      0.03 and the known U-turn 0.42. It is a slow signal and is averaged as
      one; its limit is a guard against nonsense, not a shaper (a tighter one
      clipped the U-turn itself on 19 % of frames).
    - The road is a fixed length, and eases out of its bend rather than
      stopping dead at a limit. It used to be cut back to `1.3/|curvature|`,
      so it pumped between 8 lane widths and 2.4 as the bend was re-read.
    - **The bend starts within the stretch the cart's own art hides** (about
      0.45 ahead of the nose), so the road is already turning where it comes
      out from behind the cart. Easing it in over 1.2 instead left the near
      half of every curve flat — on the day-10 U-turn only 11 % of the bend
      had arrived where the blue path starts, so the path left the cart
      straight and only bent at its far end, where it is fading out anyway.
    - The road carries on to 5 lane widths, not 8. Eight was a sixteen-fold
      extrapolation of the half lane the camera sees; once the bend was
      allowed to develop properly, that far end swung through most of a right
      angle and splayed across the picture as a widening wedge.
    - **Behind the cart the road is the track it actually drove** (`Trail` in
      `web/src/world/utils/motion.ts`), dead-reckoned from the speed and the lane's
      bend and kept only as far back as the road is drawn — a second or so,
      too little for the reckoning to drift anywhere. Drawn straight back
      instead, a bend the cart had just come round vanished the moment it
      passed under it. The cart's art cannot yaw, so the only turn there is
      to see is the world turning; the Tesla view works the same way (its car
      is pinned to one spot on the screen and never rotates — measured
      through a whole roundabout in `tesla-video.mp4`, it does not move a
      pixel).
    - This chase camera shows only about 0.76 lane widths of road behind the
      nose, so the trail is a quiet cue rather than a loud one. Raising the
      cart in the frame to where Tesla puts its car buys very little: at
      `CAR_AT` 0.74 the road behind grows to 1.04 lane widths while a sign 3 m
      ahead drops from 86 px below the top edge to 27, which cuts the heads
      off traffic lights. Measured with `.context/frame_eval.ts`.
    - `tools/../.context/yaw_eval.ts` writes the angle the road is actually
      drawn at, per frame of a recorded run; that is what the numbers above
      are measured from.
  - The lane as the camera sees it (`web/src/world/utils/lane-reader.ts`). It fades where
    the camera stops seeing.
    - Where the two lines agree, the lane is their midpoint. Where they
      disagree, it is read from the line the cart follows: the left one on
      the main road, the right one in a detour. The other line is peeling off
      at a fork or a merge, or has left the frame in a bend. Reading the
      midpoint there drew a right turn at every fork the cart drove straight
      past.
    - One line needs the lane's half width at each depth. The lens shrinks
      it towards the top of the frame, so it is learned on frames where both
      lines run straight together.
    - The floor mapping comes from the tracker's width profile, plus one
      assumption: the camera's height in lane widths.
    - **The width is schematic:** there is no known lane width.
  - **Both lane lines are drawn the same**, and lightly. The tape is real and
    has to be there, but at full strength two dark rails dominated a picture
    whose subject is the road between them. One solid and one dashed said which line the tracker was
    following, but it reads as the road-marking difference it is not: a
    dashed line is one you may cross and a solid one you may not. The anchor
    is already in the lane state text under Status.
    - What flows past instead are faint bars across the road, fixed to the
      ground, like boards in the floor. The dashes used to carry the sense of
      moving, and two solid lines on their own leave a straight stretch dead
      still.
  - **Detour forks** (`web/src/world/utils/fork-watch.ts`): both roads, the main road going
    straight on and the detour leaving to the right, with the blue path on
    the one the cart takes.
    - **The road being left stays in the picture for the whole turn**, not
      just while the camera can still see its line. It is the only thing that
      shows the cart turned at all: the cart's art cannot yaw and its own
      road is always drawn running with it, so a turn is legible only as the
      road it left swinging away. Where that road now lies is the fork's own
      geometry and the distance driven since the mouth — the same arc the
      detour was drawn as on the way in, so the picture holds together
      through the turn.
    - So once the cart is turning, the fork is past when it has driven 2 lane
      widths beyond the mouth, not when the camera stops seeing the opening:
      it stops seeing it immediately, being by then pointed down the detour.
    - The road being left keeps the normal marking colour even while the
      cart's own lane reads LOST. It is the cart's lane that was lost, not
      the road beside it.
    - The camera alone also sees a road leave at a merge, or the right line
      go in a long bend. So a fork is drawn only once something says one is
      there: a confirmed detour sign within 4 m, or a Detour waiting for its
      entrance.
    - It is placed at the sign while the sign is in view, then where the
      camera sees the right line peel away from a straight left one. It
      slides behind the cart as it passes, and fades once the lane is whole
      again.
    - The branch taken: an armed Detour takes it. Before the mission decides
      the sign, its plan; after, the path it set. `mission.decided` names the
      sign it last decided, because `next` has moved on to the following exit
      by then. The sign's caption ("take the detour", "go straight") follows
      the same rule.
  - The path: into the lane centre while the autonomy drives, a short grey
    arc where the wheels point in manual. While the autonomy holds its last
    command through a blind stretch, it shows where the wheels point, in
    blue. It starts a little ahead of the cart, where the cart's own art
    stops covering the road: drawn from the nose it came out from behind the
    roof and read as a beam standing on the cart.
    - **How far it reaches is how far the cart is committed**: as far as it
      gets in 2.2 s at the speed it is doing, capped at the full reach
      (`pathReach` in `web/src/world/utils/road.ts`). Slowing for anything draws the
      path in, and a cart that has stopped has no path at all. Taken from the
      speed it needs no reason code, so it covers every reason the cart ever
      slows — a stop sign, a red light, a pickup it is pulling up to, an
      obstacle — including the ones the dashboard is never told about.
    - **The cart's offset from the lane centre closes across the road**, as a
      lateral offset that runs out along it (`mergePath` in
      `web/src/world/utils/road.ts`), over about 2 lane widths — roughly what a
      line-follower takes to pull in a few centimetres.
    - It used to be done by scaling the lane's x towards the cart, which is
      not an offset of the road at all. Where the lane bent one way while the
      cart sat off to the other, the correction and the bend fought and
      folded the ribbon: a 9° break in a band a hand's width across, right in
      front of the cart, that read as two pieces of path joined at an angle.
      Squeezing the merge into 0.7 of a lane width made it a crease rather
      than a lean.
  - Signs at their measured range and bearing. One that would land on the
    road moves out to the kerb on the side it was first seen, so a wobbling
    bearing never carries it across. Unconfirmed ones are faint and untagged.
  - Sonar arcs only when something is in the slow or stop zone.
  - **The road lies in ground, not in the background:** a pool of light on
    the floor and a soft verge along each side. One taped lane on a view
    twice as wide as it is tall left four fifths of the picture empty,
    against about half for the Tesla view this is measured against; with the
    ground it fills about as much without cropping in, which would have cut
    the heads off traffic lights 2 to 4 m ahead. The verge stays close to the
    road: drawn broad, it swept around with the bend and read as a second
    road.
  - Overlays:
    - drive output and steering at the top left;
    - alerts at the top right, only while something is wrong;
    - the path control at the bottom left.
- **Entrega** (top right): the stage, letters A/B/C and where each one is,
  the next decision ("Entre no desvio na saída 1"), the route with the current
  step, and the track.
  - **Dock card**, while the truck waits at a letter: what it is doing there
    ("Carregando em A" / "Descarregando em C"), one instruction line ("Abra,
    carregue e feche a tampa") and the time waited. It has no release button:
    the lid releases the truck. Released, it shows what the truck is still
    checking (the lid, the weight, its voice, the way ahead) and "Esperar". A
    failed weight check says why ("Nenhuma carga detectada" / "Ainda há
    carga no compartimento"). Everything on it is `mission.dock` and the
    command's `check`; the rules stay in `mission.py`. On a phone the panel
    opens by itself when the truck docks.
  - **Compartimento:** weight (with ~ while it settles), lid, lock, scale
    fault, or "--" with no cargo node; the manifest ("Carga 420 g · coletada
    em A"). Opening it by hand is in Ajustes.
  - The track is the loop drawn from the mission's own table (`LETTER_EXIT`),
    lit where the mission's exit count puts the cart.
  - At the test venue B is on the main road; for the final day change that
    one entry and the map follows.
  - The letter pickers appear while stopped, and while parked after a
    delivery ("Entregue em C").
- **Cameras** (bottom): both at once, the drive camera with the tracker
  overlay and the sign camera with its boxes. The drive
  camera is not read while stopped, so it pauses.
- **Sistema** (bottom right): link, console, firmware, serial, sonar, cameras,
  sign model, letters, path, loop rate. Only what needs attention gets a
  mark. While stopped it also says "Pronto para retomar" (or not).
- **Ajustes** (the sliders button left of the battery): a dialog with the
  speed limits, the steering PID and the tuner's light controls. The speeds
  and gains belong to the loop, one set per stage (`tunings` in `src/main.py`); the tuner's
  sliders and the dashboard both ask for changes, and both show what is set.
  Each value can be stepped or typed (Enter sets it, Esc cancels). Reset
  returns to `Config`'s defaults. Two columns on a laptop, one on a phone. Turn signal, running lights, low beam, logo and door
  projectors, and the LED strip effect (Auto follows the drive mode). These are
  the operator's choices; the car still overrides the signal (detour, mission,
  obstacle hazards) and the dialog says so while it does. The tuner's widgets
  follow whatever either console set.

**Speed:** the car has no encoder. The large number shows the drive output the
firmware actually applied (from the `T` line; the commanded value if there is
no serial). To show an estimate, time a straight at a known PWM and set
`EST_MPS_AT_FULL_PWM` in `src/dashboard/contract.py`. It is always labelled
"est.".

---

## Network: our own router, no internet

The venue network is never used. Bring the team's home router and make a
closed LAN:

```
[cart laptop] ~~ 5 GHz Wi-Fi ~~ [home router, WAN port empty] == Ethernet == [operator laptop]
```

**Router (set up at home):**

- **Nothing in the WAN port.** Set DHCP reservations: cart `192.168.x.10`,
  operator `.20`.
- **One SSID, 5 GHz only, WPA2.** Turn the 2.4 GHz radio off.
- **A fixed non-DFS channel:** 36–48 or 149–165. A DFS channel can go silent
  for a minute after a radar detection. Use 20 or 40 MHz width, not 80.
- **On site,** run Wireless Diagnostics → Scan on a Mac and pick the
  emptiest of those channels.
- **Place it at the operator table,** raised, with line of sight to the track.
  Bring an extension cord.

**Operator laptop on Wi-Fi or Ethernet.** The dashboard never drives, so a
Wi-Fi hiccup only makes the page stutter. Ethernet (bring a USB-C adapter)
leaves one wireless hop instead of two sharing the same air, if the video
stutters too much.

**Both Macs:**

- Forget the venue Wi-Fi and turn off auto-join for every other network.
- On the cart laptop, run `caffeinate -dimsu` while driving.
- Turn off AirDrop/Handoff (`sudo ifconfig awdl0 down`). AWDL causes periodic
  Wi-Fi latency spikes on macOS.
- Keep Bluetooth on for the PS5.

**Protocol:** the cart serves the page itself. Two WebSockets carry
everything live:

- **`/ws/control`** (JSON) carries the state at 20 Hz, the events, the
  commands and their acks.
- **`/ws/video`** carries JPEG frames under credit flow control: the browser
  asks for the next frame after drawing the last one and gets the newest. On
  a bad link the frame rate drops; the delay does not grow.

**With no internet at all, everything works.** Nothing is fetched from the
internet: no fonts, CDN or map tiles.

| Failure | What happens |
|---|---|
| Venue internet down / captive portal | Nothing: the router has no WAN. |
| Wi-Fi lost, autonomous | The cart keeps driving autonomously. The page greys out within 1 s. Stop it with the PS5. |
| Wi-Fi lost, manual | Nothing: the PS5 drives over Bluetooth. |
| Router dies | Same as Wi-Fi lost. |
| Python dies | The firmware watchdog stops the motors after 0.5 s. |

**Rule for the event:** someone holds the PS5. The dashboard is never the
only way to stop the cart.

---

## Code

| Where | What |
|---|---|
| `src/status.py` | The "why is it not moving" cause and text, shared by the GUI and the dashboard. |
| `src/dashboard/contract.py` | `build_state` (pure, from the loop's objects), `parse_command`, track `layout`, segment / next and last decision / route progress. |
| `src/dashboard/hub.py` | Latest-value slots between the loop and the server. E-stop and mode are never dropped. |
| `src/dashboard/server.py` | aiohttp in a daemon thread: the page, `/ws/control`, `/ws/video`. |
| `web/` | Vite + React + TypeScript, Tailwind and three.js, all bundled: nothing is fetched at run time. `src/App.tsx` is the page; `components/` its parts (`ui/` primitives, `delivery-panel/`); `hooks/`, `services/` (control socket, cart commands) and `utils/` (health, battery, alerts, signs); `types.ts` mirrors the Python contract. `world/` is the 3D view: `world-view.tsx`, `cart.tsx` (the cart's art) and `utils/` (scene, layers, lane reader, fork watch). Tests live in `web/tests/`; the fork tests run on frames recorded at the venue (`tests/fixtures/forks.json`). `bun run lint`, `fmt`, `typecheck` and `test` check it all (oxlint, oxfmt). |
| `tests/test_dashboard.py` | Contract, hub, and a live server round trip. |
