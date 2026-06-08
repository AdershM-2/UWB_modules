/*
 * Anchor.ino - UWB RTLS anchor (responder).
 *
 * Flash this to each anchor board. The ONLY things you change per board are
 * ANCHOR_ID and ANTENNA_DELAY (from calibration). Add as many anchors as you
 * like - just give each a unique ID and list it on the tag + in MATLAB.
 *
 * Board: Makerfabs ESP32 UWB Pro with Display (DW1000).
 * SPI is initialised by the driver on the default ESP32 VSPI pins (18/19/23),
 * which match this board - do not call SPI.begin() yourself.
 */
#define UWB_HOSTLINK_SERIAL     // anchors use serial only (for debug prints)
// #define UWB_USE_OLED          // uncomment + install Adafruit SSD1306/GFX

#include <UwbRtls.h>

// >>>>>>>>>>>>>>>>> SET PER BOARD <<<<<<<<<<<<<<<<<<
static const uint8_t  ANCHOR_ID     = 0x01;     // unique: 0x01, 0x02, 0x03, ...
static const uint16_t ANTENNA_DELAY = 16384;    // replace with calibrated value
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

TwrEngine  engine;
OledStatus oled;

void setup() {
  Serial.begin(115200);
  delay(200);
  engine.begin(TWR_ANCHOR, ANCHOR_ID, ANTENNA_DELAY);
  engine.printDeviceId();

  oled.begin();
  char l0[24];
  snprintf(l0, sizeof(l0), "ANCHOR 0x%02X", ANCHOR_ID);
  oled.show(l0, "waiting...");

  Serial.printf("Anchor 0x%02X ready (antenna delay %u)\n", ANCHOR_ID, ANTENNA_DELAY);
}

void loop() {
  engine.serviceResponder();

  // Periodic status to the OLED (no effect if UWB_USE_OLED is off).
  static uint32_t last = 0;
  if (millis() - last > 500) {
    last = millis();
    if (engine.lastPeer() != UWB_ADDR_INVALID) {
      char l0[24], l1[24];
      snprintf(l0, sizeof(l0), "ANCHOR 0x%02X", ANCHOR_ID);
      snprintf(l1, sizeof(l1), "tag 0x%02X", engine.lastPeer());
      char l2[24];
      snprintf(l2, sizeof(l2), "d=%.2f m", engine.lastDistance());
      oled.show(l0, l1, l2);
    }
  }
}
