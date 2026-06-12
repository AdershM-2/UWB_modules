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
#define UWB_HOSTLINK_SERIAL
// #define UWB_HOSTLINK_UDP
// #define UWB_USE_OLED

#include <UwbRtls.h>

// v1.5: 64 MHz PRF, 5 ms reply, dead-anchor backoff, self-survey (Phase 1.0/1.2/1.3A/1.5)
#define FIRMWARE_VERSION "v1.5"

// >>>>>>>>>>>>>>>>> CONFIGURE <<<<<<<<<<<<<<<<<<
static const uint8_t  TAG_ID        = UWB_ADDR_TAG_BASE;  // 0xF0
static const uint16_t ANTENNA_DELAY = 16384;              // calibrated tag delay

// Anchor short addresses. ADD ANCHORS HERE (mirror in MATLAB AnchorConfig).
static const uint8_t ANCHORS[]  = { 0x01, 0x02, 0x03 };
static const uint8_t N_ANCHORS  = sizeof(ANCHORS) / sizeof(ANCHORS[0]);

#if defined(UWB_HOSTLINK_UDP)
static const char*   WIFI_SSID = "your-ssid";
static const char*   WIFI_PASS = "your-pass";
static IPAddress     HOST_IP(192, 168, 1, 100);   // the MATLAB PC's IP
static const uint16_t HOST_PORT = 5005;
#endif
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// Number of ranging samples averaged per anchor pair during self-survey.
static const uint16_t SURVEY_SAMPLES = 100;

TwrEngine    engine;
UwbScheduler scheduler;
HostLink     host;
OledStatus   oled;
SensorImu    imu;     // stub today; populates the IMU,... packet tail later

void setup() {
  engine.begin(TWR_TAG, TAG_ID, ANTENNA_DELAY);
  scheduler.begin(&engine, ANCHORS, N_ANCHORS);

#if defined(UWB_HOSTLINK_UDP)
  host.begin(WIFI_SSID, WIFI_PASS, HOST_IP, HOST_PORT);
#else
  host.begin(115200);
#endif

  imu.begin();    // false until the BNO085 driver is implemented
  oled.begin();
  engine.printDeviceId();
  Serial.printf("Tag 0x%02X  fw=%s  %u anchors\n", TAG_ID, FIRMWARE_VERSION, N_ANCHORS);
}

// Anchor self-survey — send "SURVEY\n" over serial to trigger.
// Wire format out: SURVEY_BEGIN,v1,<pairs> / SURVEY,v1,<src>,<dst>,<mm>,<ok> / SURVEY_DONE,v1
static void runSurvey() {
  uint8_t pairs = (N_ANCHORS * (N_ANCHORS - 1)) / 2;
  char buf[64];
  snprintf(buf, sizeof(buf), "SURVEY_BEGIN,v1,%u\n", pairs);
  host.sendRaw(buf);
  Serial.printf("Survey: %u anchors, %u pairs, %u samples each\n",
                N_ANCHORS, pairs, SURVEY_SAMPLES);

  uint8_t pairIdx = 0;
  for (uint8_t i = 0; i < N_ANCHORS; i++) {
    for (uint8_t j = i + 1; j < N_ANCHORS; j++) {
      uint8_t a = ANCHORS[i], b = ANCHORS[j];
      float   sum = 0.0f;
      uint16_t ok = 0;
      pairIdx++;
      Serial.printf("  Pair %u/%u  0x%02X -> 0x%02X:\n", pairIdx, pairs, a, b);

      for (uint16_t s = 0; s < SURVEY_SAMPLES; s++) {
        float dist, rxp;
        if (engine.surveyRequest(a, b, dist, rxp)) { sum += dist; ok++; }
        if ((s & 0xF) == 0xF)
          Serial.printf("    [%u/%u ok=%u avg=%.4fm]\n",
                        s + 1, SURVEY_SAMPLES, ok,
                        ok ? sum / ok : 0.0f);
        delay(5);
      }

      if (ok >= SURVEY_SAMPLES / 4) {
        uint32_t mm = (uint32_t)lroundf(sum / ok * 1000.0f);
        snprintf(buf, sizeof(buf), "SURVEY,v1,%u,%u,%lu,%u\n", a, b, mm, ok);
        host.sendRaw(buf);
        Serial.printf("  => %.3f m  (%u/%u ok)\n", sum / ok, ok, SURVEY_SAMPLES);
      } else {
        Serial.printf("  FAILED (ok=%u/%u)\n", ok, SURVEY_SAMPLES);
      }
    }
  }
  host.sendRaw("SURVEY_DONE,v1\n");
  Serial.println("Survey complete.");
}

void loop() {
  // Non-blocking serial command check.
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
  bool hasImu = imu.read(s);     // false in the stub
  host.sendSweep(millis(), TAG_ID, scheduler, hasImu ? &s : nullptr);

  char l0[24], l1[24];
  snprintf(l0, sizeof(l0), "TAG sweep %lu", (unsigned long)scheduler.sweepSeq());
  snprintf(l1, sizeof(l1), "%u/%u anchors", good, N_ANCHORS);
  oled.show(l0, l1);
}
