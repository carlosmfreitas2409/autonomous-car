# Cargo Node — Secure Compartment (ESP32)

The truck's cargo compartment: a load cell under the bed, a limit switch on the lid and a 12 V
solenoid lock. It reports what it measures and unlocks only while the host keeps asking. The
mission (`src/input/mission.py`) decides when that is: waiting at the right dock, never on the move.

It is not on the CAN bus itself. A serial cable connects it to the sonar node (`firmware/sonar/`),
which bridges it onto the bus. From there it reaches the master (`firmware/master/`), and the
master passes it on to the host:

```
cargo ──UART "C ..."──▶ sonar ──CAN 0x180──▶ master ──USB "C ..."──▶ host
cargo ◀──UART "K 0|1"── sonar ◀──CAN 0x181── master ◀──USB "K 0|1"── host
```

This is not how the bus is meant to be used: ideally the cargo node would sit on CAN itself. It is
the quickest way with the parts at hand. If any link in the chain is missing, the car behaves exactly
as without the compartment. The missing link can be this node unplugged, or the sonar or master on
older firmware. The host sees no `C` lines and skips every compartment check.

## Build & flash

```sh
pio run -d firmware/cargo                      # compile
pio run -d firmware/cargo -t upload            # flash via USB
pio device monitor -d firmware/cargo -b 115200 # serial monitor
```

`tools/cargo_test/cargo_test.ino` is the bench tester: scale, calibration, lid, lock and the serial
link one at a time (see its header). Flash `firmware/cargo` again afterwards.

## Hardware

- **Board**: ESP32 devkit (`esp32dev`).
- **Link**: UART2 on GPIO 32 (TX) / 33 (RX) to the sonar ESP32's UART2 (RX2 16 / TX2 17), 115200
  baud, plus a common ground. Both boards are 3.3 V, so the wires go straight across.
- **Scale**: load cell + HX711.
- **Lock**: FE-91 mini 12 V solenoid lock, spring-return. Power pulls the pin in (unlocked), and
  without power the spring pushes it out (locked). The latch is slanted, so a closing lid pushes
  it in and it locks by itself.
- **Lid**: a limit switch the closed lid presses.

### Pinout

| Signal | GPIO |
|---|---|
| Link TX → sonar RX2 (GPIO 16) | 32 |
| Link RX ← sonar TX2 (GPIO 17) | 33 |
| HX711 DT / SCK | 26 / 25 |
| Lid switch | 13 |
| Lock driver input (LOW = unlocked); also the board LED | 2 |

### Wiring

- **Link:** cargo GPIO 32 → sonar GPIO 16, sonar GPIO 17 → cargo GPIO 33, GND ↔ GND. Keep the
  cable short and away from the motor leads,
  twisted with its ground if you can.
- **HX711:** VCC on **3V3** (so DT never sees 5 V), GND, DT, SCK; the load cell's four wires as
  the module marks them (E+ E− A+ A−).
- **Lid switch:** COM to 3V3, NO to GPIO 13, and 10 kΩ from GPIO 13 to GND. Lid closed = HIGH. A
  broken wire reads *open*, and the truck will not leave until it is fixed (or the node is unplugged).
- **Lock:** GPIO 2 → the lock driver's input, exactly as on the original bench sketch. **LOW powers
  the lock (unlocked), HIGH leaves it locked.** The firmware drives the pin HIGH first thing at boot.
  - **GPIO 2 is also the board's LED** and a boot strapping pin. If uploading ever fails with
    "Failed to connect", unplug the lock driver's wire from GPIO 2 while flashing.
  - The ESP32 holds GPIO 2 low for a moment while it resets, so the lock may click open briefly at
    power-up.
  - If the ESP32 loses power while the driver still has 12 V, the input falls LOW and the lock opens.

## Serial link

ASCII lines ending in `\n`, 115200 baud, both ways.

| Line | Direction | Meaning |
|---|---|---|
| `C <grams> <stable> <lid> <lock> <fault>` | cargo → sonar, 10 Hz | grams as an integer (0 while the scale faults); then 0/1: stable, lid closed, lock powered (unlocked), scale fault |
| `K 1` / `K 0` | sonar → cargo | unlock / lock |

- **Forwarding.** The sonar node turns each `C` line into a CAN `0x180` frame and drops a line unless
  it has exactly the integer and the four 0/1 flags. The master turns the frame back into the same
  `C` line for the host. The host counts the node as gone after 1 s without a `C` line.
- **The frames** (see `firmware/sonar/README.md`): `0x180` is grams (int16 LE), then the flags
  (bit 0 stable, 1 lid closed, 2 unlocked, 3 scale fault). `0x181` carries one byte, bit 0 = unlock.
- **Unlock is a lease.** The host sends `K 1` every frame while it wants the compartment open, and the
  node locks 300 ms after the last one, so a dead host, master, bus, sonar node or cable leaves it locked. `K 0` locks at
  once.
- **What the node accepts.** Only the exact lines `K 1` and `K 0`, so a garbled byte never reads as an
  unlock.
- **The master's interlock.** It never forwards an unlock while the wheels are driven.

## Behaviour

- **Scale:**
  - It **tares at boot**, so power the node up with the compartment empty.
  - The weight sent is the mean of the last 5 readings (10 per second).
  - It is *stable* when the last 10 (1 s) lie within 5 g.
  - It reports a fault if the HX711 sends nothing for 500 ms, or if it was missing at boot (`ERR SCALE`).
- **Lock:**
  - The coil is powered only while the lease is held **and** the lid is closed. Once the lid is
    lifted the pin springs out, and the slanted latch catches as the lid shuts.
  - The coil is not rated to stay powered for long, so it stays on for at most 60 s at a stretch, then
    rests 20 s, whatever the host asks.
- **USB serial:**
  - At boot: `RDY car-cargo v1`, plus `ERR SCALE` if the HX711 did not answer.
  - Then the `C` line it sends, at 2 Hz, for a look without the sonar node.

## Calibration (one time)

`SCALE_COUNTS_PER_GRAM` in `src/config.h` is the load cell's counts per gram (shipped: 423.4243, from
the bench). To redo it:

1. Flash `tools/cargo_test`, empty the scale, send `t`.
2. Put a known mass on it (weighed on a kitchen scale), send its grams, e.g. `500`.
3. Copy the printed `SCALE_COUNTS_PER_GRAM` into `src/config.h`, and flash `firmware/cargo`.

## Bench verification (in order)

1. **Wiring:** cargo 32 → sonar 16, sonar 17 → cargo 33, and the grounds joined (TX to RX both
   ways, never TX to TX).
2. **Scale and lid:** with `tools/cargo_test`, the weight follows a known mass to a few grams, and the
   lid reads `closed` only while the switch is pressed.
3. **Lock** (`tools/cargo_test`):
   - With the lid switch held closed, `u` pulls the pin in and `l` lets it spring back out.
   - `p 500` gives a 0.5 s click.
   - With the lid open, `u` does nothing.
   - After a minute of `u` the coil is warm, not too hot to touch.
4. **Through the sonar node and the master** (`firmware/sonar` with the cargo bridge flashed):
   - With `tools/cargo_test` running and `c` sent, the master's monitor shows its `C` lines, and
     `K 1` / `K 0` typed there switch the lock (the tester prints them as they arrive).
   - With `firmware/cargo` flashed and the master monitor open, the `C` lines arrive at 10 Hz.
   - A `K 1` typed by hand only unlocks for the 300 ms lease (a short pull on the pin); the host
     renews it every frame.
   - `K 1` while the wheels turn has no effect.
   - The sonar's `S` lines still come at 20 Hz with the cargo node attached.
5. **By hand:** in the tuner, e-stopped, "Open compartment" unlocks it until the lid is lifted (10 s
   at most). The button is greyed out unless the car is e-stopped.
6. **Unplugged:** unplug the node, and the host's tuner shows `Cargo: --` within 1 s; a mission runs
   as without it.
7. **A full mission:**
   - Dock at the pickup: it unlocks. Open, load, shut the lid: it departs after the sonar's 2 s.
   - Pickup empty: "Nenhuma carga detectada". Open and shut again, or Release: it departs anyway.
   - At the delivery: unload, shut, delivered.
