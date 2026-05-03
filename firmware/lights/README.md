# Lights Arduino — Relay control

Firmware for the slave microcontroller that drives the car's seven lighting relays. Runs independently of the master Mega — the Python host opens a second USB serial port and sends `L` commands directly. A lights crash cannot affect motors or steering.

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

- **Board**: Arduino Uno.
- **Relays**: 7-channel active-LOW relay board (the common blue Arduino module).
- **Power**: relay coils on a separate 5V rail; common ground with the Uno. Do not back-feed the Uno from the relay supply.

### Pinout & relay map

| # | Pin | Function | Drives |
|---|-----|----------|--------|
| 1 | D2 | Right turn signal | Front+rear right amber lamps |
| 2 | D3 | Left turn signal | Front+rear left amber lamps |
| 3 | D4 | DRL / running | Front DRLs + rear position lamps (manual toggle) |
| 4 | D5 | Low beam | Both front low-beam projectors |
| 5 | D6 | Brake | Brake wire of both rear 4-in-1 lamps (independent of relay 3) |
| 6 | D7 | Reverse | Reverse wire of both rear 4-in-1 lamps |
| 7 | D8 | Logo | Both logo projectors |
| — | D13 | Status LED | Solid-on after boot |

D9–D12 free.

## Serial protocol

USB Serial @ 115200 baud, newline-terminated ASCII.

### Inbound (host → lights)

| Verb | Args | Effect |
|---|---|---|
| `L` | `<left> <right> <drl> <low> <brake> <reverse> <logo>` | Set relay intents. All seven values ∈ {0,1}. Sticky — held until next `L`. |

Turn-signal blink cadence is firmware-side: host sends a sticky boolean, the firmware toggles the relay every `BLINK_ON_MS` / `BLINK_OFF_MS` (`src/config.h`). Both sides share one state machine, so hazards are inherently in sync.

### Outbound

- `RDY car-lights v1` — boot banner.
- `ERR L` — malformed `L` line.
- `ERR <verb>` — unknown verb.

No estop, no watchdog. Last commanded state persists across silence; turn signals continue blinking until told otherwise.

## Tuning the front sequential module

The front turn-signal modules are sequential — they run their own internal sweep when powered. The firmware pulses the relay so the rear blinker (which is steady-on while powered) stays in sync with the front animation.

If the front sweep gets cut off mid-animation, raise `BLINK_ON_MS`. If sweeps overlap awkwardly, lower it. The OFF time is just the visible gap between blinks — pick what looks natural.

Defaults: 400 ms ON / 250 ms OFF. Tune by stopwatch against the real module.

## Bench verification

1. Open the monitor → `RDY car-lights v1`.
2. Each relay individually: send (e.g.) `L 0 0 1 0 0 0 0` → relay 3 clicks on; `L 0 0 0 0 0 0 0` → off. Repeat for each position.
3. Rear independence: `L 0 0 1 0 0 0 0` (DRL only — rear dim, fronts on); `L 0 0 1 0 1 0 0` (DRL + brake — rear bright, fronts unchanged); `L 0 0 0 0 1 0 0` (brake only — rear bright, fronts dark).
4. Turn signal blink: `L 1 0 0 0 0 0 0` → right relay clicks on/off at the configured cadence; rear lamp pulses, front sequential sweeps. Confirm sweep completes per cycle. Repeat for left and hazard (`L 1 1 0 0 0 0 0`).
5. Sticky behavior: send `L 1 0 0 0 0 0 0`, stop sending, blinker keeps going indefinitely. `L 0 0 0 0 0 0 0` stops it cleanly.
