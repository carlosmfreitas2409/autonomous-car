# Lights Arduino — Relay control + LED strip (CAN slave)

Firmware for the slave microcontroller that drives the car's lighting relays plus an addressable LED strip. Receives commands from the master Mega over a CAN bus (MCP2515). The Python host never talks to this board directly — it sends `L` over USB to the Mega, and the Mega forwards a CAN frame on `0x100`.

A lights crash still cannot affect motors or steering — the two firmwares run on physically separate MCUs.

## Build & flash

```sh
pio run -d firmware/lights                      # compile
pio run -d firmware/lights -t upload            # flash via USB
pio device monitor -d firmware/lights -b 115200 # serial monitor
```

After clone or a new worktree, regenerate the clangd compile database once:

```sh
pio run -d firmware/lights -t compiledb
```

## Hardware

- **Board**: Arduino Nano (ATmega328P, new bootloader). Same firmware pin map as Uno — no source changes required. If upload fails, switch `platformio.ini` to `board = nanoatmega328` and `upload_speed = 57600` (old bootloader).
- **Relays**: 8-channel active-LOW relay board (the common blue Arduino module).
- **LED strip**: 12V addressable strip. DIN on A2, positive wired straight to the 12V rail (not relayed), strip ground bonded to the Nano's ground.
- **Power**: relay coils on a separate 5V rail; common ground with the Nano. Do not back-feed the Nano from the relay supply.

### Pinout & relay map

| # | Pin | Function | Drives |
|---|-----|----------|--------|
| 1 | D8 | Right turn signal | Front+rear right amber lamps |
| 2 | D7 | Left turn signal | Front+rear left amber lamps |
| 3 | D14 (A0) | DRL / running | Front DRLs + rear position lamps |
| 4 | D9 | Low beam | Both front low-beam projectors |
| 5 | D3 | Brake | Brake wire of both rear 4-in-1 lamps |
| 6 | D4 | Reverse | Reverse wire of both rear 4-in-1 lamps |
| 7 | D6 | Logo | Logo projector |
| 8 | D5 | Projector | Front projector |
| — | A2 (D16) | LED strip DIN | Data line of the addressable strip (always-on 12V supply) |
| — | D13 | Status LED | Solid-on after boot (flickers during SPI traffic; cosmetic) |

MCP2515: CS=D10, MISO=D12, MOSI=D11, SCK=D13, INT=D2.

## CAN protocol

500 kbit/s, standard 11-bit IDs.

### Inbound (master → lights)

| ID | DLC | Payload | Effect |
|---|---|---|---|
| `0x100` | 2 | `data[0]` = 8-bit relay bitmask, `data[1]` = strip effect id | Set relay intents and strip effect. Sticky — held until the next frame. |

Bitmask layout: bit 0 left, 1 right, 2 drl, 3 low beam, 4 brake, 5 reverse, 6 logo, 7 projector.

Strip effect ids: `0` off, `1` red, `2` green, `3` blue, `4` strobe, `5` rainbow, `6` cyan, `7` red pulse, `8` police. Unknown ids are treated as off. A DLC-1 frame (from a master built before the strip existed) is still accepted, with the effect taken as off.

Turn-signal blink cadence is firmware-side: master sends a sticky boolean, the firmware toggles the relay every `BLINK_ON_MS` / `BLINK_OFF_MS` (`src/config.h`). Both sides share one state machine, so hazards stay in sync.

### Failsafe

If no `0x100` frame arrives for `CAN_WATCHDOG_MS` (default 1000 ms), all relays are forced off and the strip goes dark. Protects against a Mega reset or a bus fault leaving a light stuck on. Recovers automatically as soon as a fresh frame arrives.

### Outbound

USB Serial @ 115200 baud is kept for debug logging only:

- `RDY car-lights` — boot banner.
- `ERR CAN` — MCP2515 init failed.

## LED strip

`src/strip.cpp` drives A2. The strip's 12V is permanently connected, so the firmware only ever writes pixels — there is no power sequencing to do. `init()` blanks the strip at boot so it does not hold whatever its controller powered up with.

Police (id `8`) paints **car sides**, not wire order. The strip starts on the left, runs across the front (`STRIP_FRONT_COUNT` pixels, currently 9), down the right side through the second-to-last LED, then the last trio is the rear — of which only the final pixel is rear-left. Police red is that front run plus the last LED (10 pixels); blue is 10 through the second-to-last (12 pixels).

### Why two timing constants

They answer different questions and neither replaces the other.

`STRIP_FRAME_MS` is the **animation tick** — the fastest strobe or rainbow may advance. It exists because `pixels.show()` bit-bangs the whole strip with interrupts disabled (~30 µs/LED, so ~0.7 ms at 22 LEDs). `loop()` runs thousands of times a second; redrawing on every pass would spend most of the CPU with interrupts off, starving CAN polling and dragging `millis()`. At 10 ms the strip is redrawn 100×/s and the interrupts-off duty is ~7%.

`STRIP_REFRESH_MS` covers the opposite case: a **static** colour has nothing to animate, so it would otherwise be transmitted exactly once and never again. One frame corrupted by relay or motor switching noise would then sit on the strip until the effect next changed. Resending every 250 ms bounds that to a quarter second, for one `show()` every 25 animation ticks.

So: animations are limited by `STRIP_FRAME_MS`, static colours are floored by `STRIP_REFRESH_MS`.

Constants in `src/config.h`:

| Constant | Default | Notes |
|---|---|---|
| `STRIP_LED_COUNT` | 22 | **Set this to the strip's real length.** Too low leaves the tail dark. |
| `STRIP_BRIGHTNESS` | 150 | 0–255. Caps current draw and overall glare. |
| `STRIP_FRAME_MS` | 10 | Animation tick — how often strobe/rainbow may advance (100 Hz cap). |
| `STRIP_REFRESH_MS` | 250 | How often a *static* colour is resent, so a corrupted frame heals. |
| `STROBE_ON_MS` / `STROBE_OFF_MS` | 40 / 60 | Strobe cadence. |
| `RAINBOW_HUE_STEP` | 256 | Hue advance per frame, of 65536 → 256 frames ≈ 2.6 s per cycle. |
| `PULSE_STEP` | 437 | Red-pulse breath rate → ~1.5 s per cycle. |
| `STRIP_FRONT_COUNT` | 9 | Nose pixels (LEDs 1–9). Police red includes these plus the last LED. |
| `STRIP_REAR_COUNT` | 3 | Last trio is the rear; only the final pixel is rear-left. |
| `POLICE_FLASH_MS` / `GAP` / `PAUSE` | 50 / 50 / 180 | Double-flash wig-wag cadence. |

### Mode signature

By default the host drives the strip from the car's own state rather than a fixed choice, so a bystander can tell who is in control:

| State | Effect | Meaning |
|---|---|---|
| E-stopped | `7` red pulse | Halted. Wins over everything else. |
| Autonomous | `6` cyan | The computer is driving. |
| Manual | `2` green | A human has the pad. |

The mapping lives in `LightsController._resolve_strip()` (`src/lights.py`), not in firmware — this board only knows how to render an effect id. The GUI's **Auto** entry selects this behaviour; picking any other entry pins the strip to that effect for bench testing.

Red *pulse* rather than red strobe for e-stop: the car boots latched in e-stop and sits there for long stretches, and a strobe that whole time is fatiguing enough that you stop seeing it. Swap `RED_PULSE` for `STROBE` in `_resolve_strip()` if you want it louder.

### Colour order

This strip is **`NEO_BRG`** — it takes blue, red, green on the wire. Measured, not guessed: under `NEO_GRB` a green argument lit blue and a blue argument lit green while red was already right, which pins the wire order to blue-red-green.

To re-derive it for a different strip, drive one channel at a time and note the colour that appears. `NEO_xyz` names the order the *strip* wants, not the order you pass to `setPixelColor()` — that is always `(r, g, b)` regardless. `Color()` and `ColorHSV()` both return plain packed RGB, so fixing the constructor flag fixes every effect at once.

A wrong order gives **stable, repeatable wrong colours**. That is different from *garbage*: a wrong bitrate (`NEO_KHZ400` vs `NEO_KHZ800`) or a noisy data line desynchronises the shift register, so pixels flicker, change colour frame to frame, light in the wrong positions, or the first few work and the rest go random. Stable-but-swapped → change the order flag. Unstable → suspect timing, wiring or noise.

## Bench verification

All `L` commands below are sent over USB to the **master Mega**, not this board. The master forwards them as CAN frame `0x100`.

1. Open the master monitor → `RDY car-master v1`. Open the lights monitor → `RDY car-lights`. Neither should print `ERR CAN`.
2. Power the lights board with no CAN traffic → watchdog fires within 1 s → all relays off, strip dark.
3. Each relay individually: send (e.g.) `L 0 0 1 0 0 0 0 0` to the master → relay 3 clicks on; `L 0 0 0 0 0 0 0 0` → off. Repeat for each position.
4. Rear independence: `L 0 0 1 0 0 0 0 0` (DRL only — rear dim, fronts on); `L 0 0 1 0 1 0 0 0` (DRL + brake — rear bright, fronts unchanged); `L 0 0 0 0 1 0 0 0` (brake only — rear bright, fronts dark).
5. Turn signal blink: `L 1 0 0 0 0 0 0 0` → right relay clicks on/off at the configured cadence; rear lamp pulses, front sequential sweeps. Confirm sweep completes per cycle. Repeat for left and hazard (`L 1 1 0 0 0 0 0 0`).
6. LED strip: `L 0 0 0 0 0 0 0 0 1` → strip goes solid red; step through `2` green, `3` blue, `4` strobe, `5` rainbow, `8` police (left red / right blue, double-flash); `L 0 0 0 0 0 0 0 0 0` → strip clears. If nothing happens at all, the master is probably still running pre-strip firmware — it sends a DLC-1 frame, which the lights board reads as "off". Reflash it.
7. Backwards compatibility: an 8-token `L 0 0 1 0 0 0 0 0` still works and leaves the strip off.
8. Watchdog: with rainbow running and a light on, pull the CAN cable. All relays should drop and the strip go dark within 1 s. Reconnect → next frame restores the commanded state.
