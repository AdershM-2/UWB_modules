/*
 * UwbConfig.h - Shared radio / board configuration for the UwbRtls library.
 *
 * These settings MUST be identical on every board in the network (tag + all
 * anchors), with the sole exception of per-device values that each sketch sets
 * itself: the device short address and the calibrated antenna delay.
 *
 * Board: Makerfabs "ESP32 UWB Pro with Display" (Decawave DW1000 / BU01).
 */
#ifndef UWBRTLS_UWBCONFIG_H
#define UWBRTLS_UWBCONFIG_H

#include <Arduino.h>
#include "dw1000/DW1000.h"

// ---------------------------------------------------------------------------
// Pin map - Makerfabs ESP32 UWB Pro with Display
// ---------------------------------------------------------------------------
#define UWB_PIN_SCK   18
#define UWB_PIN_MISO  19
#define UWB_PIN_MOSI  23
#define UWB_PIN_SS    21   // SPI chip select for the DW1000
#define UWB_PIN_RST   27   // DW1000 reset
#define UWB_PIN_IRQ   34   // DW1000 interrupt (input-only pin, fine for IRQ)

#define OLED_PIN_SDA   4
#define OLED_PIN_SCL   5

// ---------------------------------------------------------------------------
// Radio profile - accuracy mode. Same on every board. (Phase 1.0)
//   MODE_LONGDATA_RANGE_ACCURACY = 110 kb/s, 64 MHz PRF, 2048 preamble.
//   64 MHz PRF sharpens the channel-impulse-response autocorrelation peak, so
//   the receiver locks the first-path arrival more reliably and rejects later
//   multipath peaks. This improves ranging ACCURACY, not range - the Pro board's
//   PA/LNA already covers range. The 16 MHz LOWPOWER variant is the more
//   noise-tolerant alternative if range ever becomes the limiting factor.
// ---------------------------------------------------------------------------
#define UWB_RADIO_MODE   DW1000.MODE_LONGDATA_RANGE_ACCURACY
#define UWB_CHANNEL      DW1000.CHANNEL_5

// Reply delay used for the delayed transmits in two-way ranging (microseconds).
// History: 7000 us (16 MHz LOWPOWER) → 6000 us (Phase 1.0, 64 MHz ACCURACY) →
//          5000 us (Phase 1.3, same mode, tighter margin but well-validated).
// Minimum safe value for this mode is ~3500 us (ESP32 SPI + DW1000 processing).
// If you switch to 6.8 Mb/s fast mode, this can drop to ~1500 us.
#define UWB_REPLY_DELAY_US   5000

// Default antenna delay (DW1000 ticks). Each board overrides this with its own
// CALIBRATED value (see examples/AntennaCalibration). 16384 is the chip reset
// default; tuned values typically land in 16450..16650.
#define UWB_DEFAULT_ANTENNA_DELAY  16384

// ---------------------------------------------------------------------------
// Addressing (1-byte short addresses, our own scheme - not 802.15.4).
//   0x00          : reserved / invalid
//   0x01 .. 0xEF  : anchors
//   0xF0 .. 0xFE  : tags
//   0xFF          : broadcast (reserved for future multi-tag announce/slotting)
// ---------------------------------------------------------------------------
#define UWB_ADDR_INVALID    0x00
#define UWB_ADDR_BROADCAST  0xFF
#define UWB_ADDR_TAG_BASE   0xF0   // first tag = 0xF0

// Network id (shared). Frame filtering is done in software, so this is mostly
// cosmetic, but we keep it consistent across the fleet.
#define UWB_NETWORK_ID  0xDECA

#endif // UWBRTLS_UWBCONFIG_H
