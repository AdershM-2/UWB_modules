/*
 * Tag.ino - UWB RTLS tag (initiator).
 *
 * The tag ranges to every anchor in ANCHORS[] each sweep and streams the raw
 * distances to the MATLAB host. It NEVER solves position itself, so adding
 * anchors only means editing ANCHORS[] here and the matching AnchorConfig in
 * MATLAB - no algorithm change, and no firmware change once anchors are added.
 *
 * Transport: pick ONE of the two #defines below.
 *   UWB_HOSTLINK_SERIAL : stream over USB serial (simplest; tethered)
 *   UWB_HOSTLINK_UDP    : stream over WiFi UDP to the MATLAB PC
 */

// ---- choose ONE transport (compile-time) ----
// #define UWB_HOSTLINK_SERIAL
#define UWB_HOSTLINK_UDP
#define UWB_USE_OLED

#include <UwbRtls.h>

// >>>>>>>>>>>>>>>>> CONFIGURE <<<<<<<<<<<<<<<<<<
static const uint8_t  TAG_ID        = UWB_ADDR_TAG_BASE;  // 0xF0
static const uint16_t ANTENNA_DELAY = 16466;              // calibrated tag delay

// Anchor short addresses. ADD ANCHORS HERE (mirror in MATLAB AnchorConfig).
static const uint8_t ANCHORS[]  = { 0x01, 0x02, 0x03 };
static const uint8_t N_ANCHORS  = sizeof(ANCHORS) / sizeof(ANCHORS[0]);

#if defined(UWB_HOSTLINK_UDP)
static const char*    WIFI_SSID  = "Biriyani";
static const char*    WIFI_PASS  = "Legpiece";
static IPAddress      HOST_IP(192, 168, 1, 101);   // MATLAB PC
static const uint16_t HOST_PORT  = 5005;
#endif
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// Number of ranging samples averaged per anchor pair during self-survey.
// 100 samples ≈ 4 s per pair; 6 pairs (4 anchors) ≈ 24 s total.
static const uint16_t SURVEY_SAMPLES = 100;

TwrEngine    engine;
UwbScheduler scheduler;
HostLink     host;
OledStatus   oled;
SensorImu    imu;     // stub today; populates the IMU,... packet tail later

static void splashIdent() {
  char title[10], l1[20], l2[20];
  snprintf(title, sizeof(title), "TAG  %02X", TAG_ID);
  snprintf(l1,    sizeof(l1),    "Delay: %u", ANTENNA_DELAY);
  snprintf(l2,    sizeof(l2),    "%u anchors", N_ANCHORS);
  oled.showSplash(title, l1, l2, "Starting...");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Init OLED before WiFi — WiFi startup corrupts I2C if OLED init runs after.
  oled.begin();
  oled.showSplash("TAG  F0", "Starting...", "WiFi...");

#if defined(UWB_HOSTLINK_UDP)
  host.begin(WIFI_SSID, WIFI_PASS, HOST_IP, HOST_PORT);
  // Re-init I2C after WiFi — WiFi RF can glitch the bus during association.
  Wire.end();
  delay(50);
  oled.begin();
#endif
  splashIdent();

  engine.begin(TWR_TAG, TAG_ID, ANTENNA_DELAY);
  scheduler.begin(&engine, ANCHORS, N_ANCHORS);
  imu.begin();
  engine.printDeviceId();
  Serial.printf("Tag 0x%02X ready, %u anchors\n", TAG_ID, N_ANCHORS);
}

// ---------------------------------------------------------------------------
// Anchor self-survey: ask every anchor pair (A,B) to range to each other.
// Outputs SURVEY_BEGIN / SURVEY,v1,... / SURVEY_DONE lines on HostLink.
// Triggered by sending "SURVEY\n" over Serial.
// Wire format: SURVEY,v1,<src_id>,<dst_id>,<avg_dist_mm>,<ok_samples>
// ---------------------------------------------------------------------------
static void runSurvey() {
  uint8_t pairs = (N_ANCHORS * (N_ANCHORS - 1)) / 2;
  char buf[64];

  snprintf(buf, sizeof(buf), "SURVEY_BEGIN,v1,%u\n", pairs);
  host.sendRaw(buf);
  Serial.printf("Survey: %u anchors, %u pairs, %u samples each\n",
                N_ANCHORS, pairs, SURVEY_SAMPLES);
  oled.showSplash("SURVEY", "Running...", buf);

  uint8_t pairIdx = 0;
  for (uint8_t i = 0; i < N_ANCHORS; i++) {
    for (uint8_t j = i + 1; j < N_ANCHORS; j++) {
      uint8_t a = ANCHORS[i], b = ANCHORS[j];
      float   sum = 0.0f;
      uint16_t ok = 0;
      pairIdx++;

      Serial.printf("  Pair %u/%u  0x%02X -> 0x%02X:\n", pairIdx, pairs, a, b);

      char oledL1[20], oledL2[20];
      snprintf(oledL1, sizeof(oledL1), "%u/%u  %02X->%02X", pairIdx, pairs, a, b);

      for (uint16_t s = 0; s < SURVEY_SAMPLES; s++) {
        float dist, rxp;
        if (engine.surveyRequest(a, b, dist, rxp)) {
          sum += dist;
          ok++;
        }
        if ((s & 0xF) == 0xF) {
          Serial.printf("    [%u/%u ok=%u]\n", s + 1, SURVEY_SAMPLES, ok);
          snprintf(oledL2, sizeof(oledL2), "%u/%u ok=%u", s+1, SURVEY_SAMPLES, ok);
          oled.showSplash("SURVEY", oledL1, oledL2);
        }
        delay(5);
      }

      if (ok >= SURVEY_SAMPLES / 4) {
        float mean = sum / ok;
        uint32_t mm = (uint32_t)lroundf(mean * 1000.0f);
        snprintf(buf, sizeof(buf), "SURVEY,v1,%u,%u,%lu,%u\n", a, b, mm, ok);
        host.sendRaw(buf);
        Serial.printf("  => %.3f m  (%u/%u ok)\n", mean, ok, SURVEY_SAMPLES);
      } else {
        Serial.printf("  FAILED (ok=%u/%u) — check link between 0x%02X and 0x%02X\n",
                      ok, SURVEY_SAMPLES, a, b);
      }
    }
  }

  host.sendRaw("SURVEY_DONE,v1\n");
  Serial.println("Survey complete.");
  oled.showSplash("SURVEY", "DONE", "See Serial/MATLAB");
}

void loop() {
  // Non-blocking check for serial commands (survey trigger).
  static char cmdBuf[16];
  static uint8_t cmdLen = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      cmdBuf[cmdLen] = '\0';
      if (strcmp(cmdBuf, "SURVEY") == 0) runSurvey();
      cmdLen = 0;
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }

  uint8_t good = scheduler.sweep();

  ImuSample s;
  bool hasImu = imu.read(s);
  host.sendSweep(millis(), TAG_ID, scheduler, hasImu ? &s : nullptr);

  char title[10], l1[20], l2[20];
  snprintf(title, sizeof(title), "TAG  %02X", TAG_ID);
  snprintf(l1,    sizeof(l1),    "Sweep #%lu", (unsigned long)scheduler.sweepSeq());
  snprintf(l2,    sizeof(l2),    "%u/%u anchors OK", good, N_ANCHORS);
  oled.showSplash(title, l1, l2);
}
