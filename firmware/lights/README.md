# Lights Arduino — Relay control (CAN slave)

Firmware for the slave microcontroller that drives the car's seven lighting relays. Receives commands from the master Mega over a CAN bus (MCP2515). The Python host never talks to this board directly — it sends `L` over USB to the Mega, and the Mega forwards a CAN frame on `0x100`.

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
- **Relays**: 7-channel active-LOW relay board (the common blue Arduino module).
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
| 7 | D6 | Logo | Both logo projectors |
| — | D13 | Status LED | Solid-on after boot (flickers during SPI traffic; cosmetic) |

MCP2515: CS=D10, MISO=D12, MOSI=D11, SCK=D13, INT=D2.

## CAN protocol

500 kbit/s, standard 11-bit IDs.

### Inbound (master → lights)

| ID | DLC | Payload | Effect |
|---|---|---|---|
| `0x100` | 1 | `data[0]` = 7-bit bitmask | Set relay intents. Sticky — held until the next frame. |

Bitmask layout: bit 0 left, 1 right, 2 drl, 3 low beam, 4 brake, 5 reverse, 6 logo. Bit 7 reserved (must be 0).

Turn-signal blink cadence is firmware-side: master sends a sticky boolean, the firmware toggles the relay every `BLINK_ON_MS` / `BLINK_OFF_MS` (`src/config.h`). Both sides share one state machine, so hazards stay in sync.

### Failsafe

If no `0x100` frame arrives for `CAN_WATCHDOG_MS` (default 1000 ms), all relays are forced off. Protects against a Mega reset or a bus fault leaving a light stuck on. Recovers automatically as soon as a fresh frame arrives.

### Outbound

USB Serial @ 115200 baud is kept for debug logging only:

- `RDY car-lights v2 (CAN)` — boot banner.
- `ERR CAN` — MCP2515 init failed.

## Tuning the front sequential module

The front turn-signal modules are sequential — they run their own internal sweep when powered. The firmware pulses the relay so the rear blinker (which is steady-on while powered) stays in sync with the front animation.

If the front sweep gets cut off mid-animation, raise `BLINK_ON_MS`. If sweeps overlap awkwardly, lower it. The OFF time is just the visible gap between blinks — pick what looks natural.

Defaults: 400 ms ON / 250 ms OFF. Tune by stopwatch against the real module.

## Bench verification

All `L` commands below are sent over USB to the **master Mega**, not this board. The master forwards them as CAN frame `0x100`.

1. Open the master monitor → `RDY car-control v1`. Open the lights monitor → `RDY car-lights v2 (CAN)`. Neither should print `ERR CAN`.
2. Power the lights board with no CAN traffic → watchdog fires within 1 s → all relays off.
3. Each relay individually: send (e.g.) `L 0 0 1 0 0 0 0` to the master → relay 3 clicks on; `L 0 0 0 0 0 0 0` → off. Repeat for each position.
4. Rear independence: `L 0 0 1 0 0 0 0` (DRL only — rear dim, fronts on); `L 0 0 1 0 1 0 0` (DRL + brake — rear bright, fronts unchanged); `L 0 0 0 0 1 0 0` (brake only — rear bright, fronts dark).
5. Turn signal blink: `L 1 0 0 0 0 0 0` → right relay clicks on/off at the configured cadence; rear lamp pulses, front sequential sweeps. Confirm sweep completes per cycle. Repeat for left and hazard (`L 1 1 0 0 0 0 0`).
6. Watchdog: while a light is on, pull the CAN cable. All relays should drop within 1 s. Reconnect → next frame restores the commanded state.
