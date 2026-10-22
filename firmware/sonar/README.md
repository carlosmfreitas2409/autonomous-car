# Sonar Node — Front Ultrasonic Array (ESP32)

Bumper-mounted CAN node that pings three HC-SR04 ultrasonic sensors and broadcasts their
distances. It makes no decisions — the master Arduino owns the obstacle reflex.

Third node on the CAN bus, alongside the master (`firmware/master/`) and the lights Nano
(`firmware/lights/`).

## Build & flash

```sh
pio run -d firmware/sonar                      # compile
pio run -d firmware/sonar -t upload            # flash via USB
pio device monitor -d firmware/sonar -b 115200 # serial monitor
```

## Hardware

- **Board**: ESP32 devkit.
- **CAN**: MCP2515 over SPI, the same transceiver setup as the master. 500 kbps, 8 MHz
  crystal, matching the master's `setBitrate(CAN_500KBPS, MCP_8MHZ)`.
- **Sensors**: 3× HC-SR04.

> **The ESP32 is a 3.3 V part and the HC-SR04's ECHO pin drives 5 V.** Fit a divider on
> every echo line, or use 3.3 V-native **HC-SR04P** / **JSN-SR04T** modules.

### Pinout

| Signal | GPIO |
|---|---|
| TRIG 0..2 | 27, 26, 25 |
| ECHO 0..2 | 35, 32, 33 |
| MCP2515 SPI (CS / MISO / MOSI / SCK) | 5 / 19 / 23 / 18 |
| Cargo link RX2 ← cargo GPIO 32 / TX2 → cargo GPIO 33 | 16 / 17 |
| Status LED | 2 |

## CAN frame — ID `0x080`, DLC 3, 20 Hz

`0x080` is numerically below the lights' `0x100`, so obstacle data wins bus arbitration.

| Byte | Meaning |
|---|---|
| 0–2 | Distance in cm for sensors 0..2, left to right (`0xFF` = nothing in range) |

The master treats a frame gap longer than 300 ms as *blocked*, never *clear*.

## Cargo bridge

The cargo node (`firmware/cargo/`) has no CAN interface, so it reaches the bus through this node's
UART2: 115200 baud, ASCII lines, with a common ground and no level shifting (both are 3.3 V).

| From | To | Becomes |
|---|---|---|
| cargo: `C <grams> <stable> <lid> <lock> <fault>` (10 Hz) | bus | CAN `0x180`, DLC 3: grams int16 LE, then bit 0 stable, 1 lid closed, 2 unlocked, 3 scale fault |
| bus: CAN `0x181`, DLC 1, bit 0 unlock | cargo | `K 1` / `K 0` |

- **Dropped lines.** A `C` line without exactly an integer and four 0/1 flags is dropped.
- **Acceptance filter.** The MCP2515's filters let only `0x181` in, so the lights frames can't fill
  its two receive buffers while a ping cycle blocks (up to ~45 ms).
- **Timing.** The bridge runs at the top of `loop()`, before the 20 Hz sonar timer, so the pings and
  the `0x080` frame are as before.
- **No cargo node.** Nothing arrives on UART2 and nothing goes out as `0x180`.
