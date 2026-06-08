# 01 — Hardware & firmware setup

## Board
Makerfabs **ESP32 UWB Pro with Display** — ESP32 + Decawave **DW1000** (BU01
module with power amplifier) + 1.3" SSD1306 OLED. You have 4 of these: 3+
anchors and 1 tag.

> ⚠️ Confirm the chip is **DW1000**. Makerfabs also sells a near-identically
> named **DW3000** board which is incompatible with this library. All boards in
> one network must be the same chip.

## Pin map (already encoded in `UwbConfig.h`)
| Function | Pin |
|---|---|
| SPI SCK | 18 |
| SPI MISO | 19 |
| SPI MOSI | 23 |
| DW1000 CS (SS) | 21 |
| DW1000 RST | 27 |
| DW1000 IRQ | 34 |
| OLED SDA | 4 |
| OLED SCL | 5 |

The DW1000 driver calls `SPI.begin()` on the ESP32 default VSPI pins (18/19/23),
which match this board — **do not** call `SPI.begin()` in your sketch.

## Toolchain (Arduino IDE)
1. Install the **ESP32 board package** (Espressif) via Boards Manager.
2. Install this library: copy the folder `libraries/UwbRtls` into your Arduino
   sketchbook `libraries/` directory
   (e.g. `Documents/Arduino/libraries/UwbRtls`). Restart the IDE.
   The examples appear under **File ▸ Examples ▸ UwbRtls**.
3. *(Optional OLED)* Install **Adafruit SSD1306** + **Adafruit GFX** and add
   `#define UWB_USE_OLED` at the top of a sketch. Without it, the display calls
   are harmless no-ops.
4. Board: select an **ESP32 Dev Module** (or the Makerfabs profile). Choose the
   correct **COM port**.

## Roles & per-board settings
- **Anchors** (`examples/Anchor`): set a unique `ANCHOR_ID` (`0x01`, `0x02`, …)
  and the board’s calibrated `ANTENNA_DELAY`.
- **Tag** (`examples/Tag`): set `ANTENNA_DELAY`, the `ANCHORS[]` list, and the
  transport (`UWB_HOSTLINK_SERIAL` or `UWB_HOSTLINK_UDP`; for UDP also the WiFi
  credentials and the MATLAB PC’s IP/port).

## First light (SPI sanity check)
Flash any sketch and open Serial Monitor at **115200**. You should see a
`DW1000 device id:` line containing `DECA`. If not, re-check wiring/board/port.

Next: **[02 — protocol](02_protocol.md)** and **[03 — calibration](03_calibration.md)**.
