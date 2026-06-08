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
#define UWB_USE_OLED            // on-board SSD1306 display
#define UWB_HOSTLINK_SERIAL     // serial debug prints

#include <UwbRtls.h>

// >>>>>>>>>>>>>>>>> SET PER BOARD <<<<<<<<<<<<<<<<<<
static const uint8_t  ANCHOR_ID     = 0x03;     // unique: 0x01, 0x02, 0x03, ...
static const uint16_t ANTENNA_DELAY = 16473;    // replace with calibrated value
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

TwrEngine  engine;
OledStatus oled;

static void splashIdent() {
  char title[12], l1[20];
  snprintf(title, sizeof(title), "ANCHOR %02X", ANCHOR_ID);
  snprintf(l1,    sizeof(l1),    "Delay: %u",   ANTENNA_DELAY);
  oled.showSplash(title, l1, "Waiting...");
}

void setup() {
  Serial.begin(115200);
  delay(200);
  oled.begin();
  splashIdent();

  engine.begin(TWR_ANCHOR, ANCHOR_ID, ANTENNA_DELAY);
  engine.printDeviceId();
  Serial.printf("Anchor 0x%02X ready (antenna delay %u)\n", ANCHOR_ID, ANTENNA_DELAY);
}

void loop() {
  engine.serviceResponder();

  static uint32_t last = 0;
  if (millis() - last > 500) {
    last = millis();
    char title[12], l1[20], l2[20], l3[20];
    snprintf(title, sizeof(title), "ANCHOR %02X", ANCHOR_ID);
    snprintf(l1,    sizeof(l1),    "Delay: %u",   ANTENNA_DELAY);
    if (engine.lastPeer() != UWB_ADDR_INVALID) {
      snprintf(l2, sizeof(l2), "Tag:  0x%02X", engine.lastPeer());
      snprintf(l3, sizeof(l3), "d=%.2fm", engine.lastDistance());
    } else {
      strncpy(l2, "Waiting...", sizeof(l2));
      l3[0] = '\0';
    }
    oled.showSplash(title, l1, l2, l3);
  }
}
