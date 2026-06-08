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
// Radio profile - long range / robust. Same on every board.
//   MODE_LONGDATA_RANGE_LOWPOWER = 110 kb/s, 16 MHz PRF, 2048 preamble.
//   This is the most range- and noise-tolerant mode, ideal for the Pro's PA.
// ---------------------------------------------------------------------------
#define UWB_RADIO_MODE   DW1000.MODE_LONGDATA_RANGE_LOWPOWER
#define UWB_CHANNEL      DW1000.CHANNEL_5

// Reply delay used for the delayed transmits in two-way ranging (microseconds).
// 7000 us is the proven value for the 110 kb/s / long-preamble profile above.
// If you switch to a faster mode you can reduce this (e.g. 3000 us at 6.8 Mb/s).
#define UWB_REPLY_DELAY_US   7000

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
