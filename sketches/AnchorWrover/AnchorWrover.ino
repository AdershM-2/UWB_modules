/*
 * AnchorWrover.ino  –  UWB anchor for a generic ESP32-WROVER + DW1000 module
 *                      (ESP32 UWB non-Pro) with external SSD1306 OLED.
 *
 * Peripheral map
 * ──────────────────────────────────────────────────────────────────────
 *  DW1000 (SPI) : SCK=18  MISO=19  MOSI=23  CS=4   RST=27  IRQ=34
 *  SSD1306 OLED : SDA=32  SCL=33   (Wire, I2C address 0x3C or 0x3D)
 *
 *  GPIO 4 = DW1000 CS on this board (non-Pro).
 *  Default OLED SDA=4 in UwbConfig.h would conflict — override to 32/33.
 *
 * SET PER BOARD: change ANCHOR_ID and ANTENNA_DELAY below before flashing.
 *   ANCHOR_ID     : unique address for each anchor (0x01, 0x02, 0x03, ...)
 *   ANTENNA_DELAY : per-device calibrated value from AntennaCalibration sketch
 */

#define UWB_USE_OLED
#define UWB_HOSTLINK_SERIAL

// ── Board pin overrides — MUST appear before #include <UwbRtls.h> ────────────
// GPIO 4 = DW1000 CS on non-Pro board; default OLED SDA=4 in UwbConfig.h
// would conflict.  Override both here.  Rewire OLED: SDA→32, SCL→33.
#define UWB_PIN_SS    4    // DW1000 chip select (hardware-wired on this board)
#define OLED_PIN_SDA 32    // OLED SDA — moved off GPIO 4 to avoid CS clash
#define OLED_PIN_SCL 33    // OLED SCL

#include <UwbRtls.h>

// >>>>>>>>>>>>>>>>> SET PER BOARD <<<<<<<<<<<<<<<<<<
static const uint8_t  ANCHOR_ID     = 0x01;            // unique: 0x01, 0x02, 0x03, 0x04
static const uint16_t ANTENNA_DELAY = UWB_DEFAULT_ANTENNA_DELAY;  // replace with calibrated value
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

TwrEngine  engine;
OledStatus oled;

static void splashIdent() {
  char title[12], l1[20];
  snprintf(title, sizeof(title), "ANCHOR %02X", ANCHOR_ID);
  snprintf(l1,    sizeof(l1),    "Delay: %u",   ANTENNA_DELAY);
  oled.showSplash(title, l1, "CS=4 Waiting...");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // DW1000 FIRST — must precede OLED init.
  // "DECA 01302001" in serial → CS pin correct; garbage → wrong CS pin.
  engine.begin(TWR_ANCHOR, ANCHOR_ID, ANTENNA_DELAY);
  engine.printDeviceId();
  Serial.printf("Anchor 0x%02X ready (CS=%d delay=%u)\n",
                ANCHOR_ID, UWB_PIN_SS, ANTENNA_DELAY);

  oled.begin();
  splashIdent();
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
      snprintf(l3, sizeof(l3), "d=%.2fm",      engine.lastDistance());
    } else {
      strncpy(l2, "Waiting...", sizeof(l2));
      l3[0] = '\0';
    }
    oled.showSplash(title, l1, l2, l3);
  }
}
