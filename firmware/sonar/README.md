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
| Status LED | 2 |

## CAN frame — ID `0x080`, DLC 3, 20 Hz

`0x080` is numerically below the lights' `0x100`, so obstacle data wins bus arbitration.

| Byte | Meaning |
|---|---|
| 0–2 | Distance in cm for sensors 0..2, left to right (`0xFF` = nothing in range) |

The master treats a frame gap longer than 300 ms as *blocked*, never *clear*.
