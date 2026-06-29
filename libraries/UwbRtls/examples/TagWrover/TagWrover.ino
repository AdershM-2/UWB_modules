/*
 * TagWrover.ino  –  UWB tag for a generic ESP32-WROVER + DW1000 module
 *                   with external SSD1306 OLED and BNO085 IMU.
 *
 * Reference copy (no OLED define, serial transport, generic credentials).
 * The active flash target with WiFi credentials lives in sketches/TagWrover/.
 *
 * Peripheral map
 * ──────────────────────────────────────────────────────────────────────
 *  DW1000 (SPI) : SCK=18  MISO=19  MOSI=23  CS=4   RST=27  IRQ=34
 *  SSD1306 OLED : SDA=4   SCL=5    (Wire,  I2C address 0x3C or 0x3D)
 *  BNO085 IMU   : SDA=16  SCL=17   (Wire1, I2C address 0x4A)
 *
 * ⚠  GPIO 16/17 are used for PSRAM on most ESP32-WROVER modules.
 *    If PSRAM is enabled in the Arduino IDE, change BNO_SDA/BNO_SCL to 25/26
 *    or 32/33 and rewire accordingly.
 *
 * Required libraries:
 *   Adafruit BNO08x, Adafruit SSD1306, Adafruit GFX Library
 */

// ── Transport ─────────────────────────────────────────────────────────────────
#define UWB_HOSTLINK_SERIAL
// #define UWB_HOSTLINK_UDP
// #define UWB_USE_OLED   // uncomment if OLED is connected

// ── Board pin override — ESP32 UWB (no display / non-Pro) uses CS=4 not 21 ──
#define UWB_PIN_SS 4

#include <UwbRtls.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ── Configuration ─────────────────────────────────────────────────────────────
static const uint8_t  TAG_ID        = UWB_ADDR_TAG_BASE;
static const uint16_t ANTENNA_DELAY = UWB_DEFAULT_ANTENNA_DELAY;

static const uint8_t ANCHORS[]  = { 0x01, 0x02, 0x03, 0x04 };
static const uint8_t N_ANCHORS  = sizeof(ANCHORS) / sizeof(ANCHORS[0]);

#if defined(UWB_HOSTLINK_UDP)
static const char*    WIFI_SSID = "your-ssid";
static const char*    WIFI_PASS = "";
static IPAddress      HOST_IP(192, 168, 1, 100);
static const uint16_t HOST_PORT  = 4100;
#endif

#define BNO_SDA   16
#define BNO_SCL   17
#define BNO_ADDR  0x4A

static const uint16_t SURVEY_SAMPLES = 100;

// ── Objects ───────────────────────────────────────────────────────────────────
TwrEngine    engine;
UwbScheduler scheduler;
HostLink     host;
OledStatus   oled;

Adafruit_BNO08x bno085(-1);
static bool     imuPresent = false;

static struct {
  float   qw=1, qx=0, qy=0, qz=0;
  float   ax=0, ay=0, az=0;
  float   gx=0, gy=0, gz=0;
  float   roll=0, pitch=0, yaw=0;
  uint8_t status = 0;
  bool    valid  = false;
} imuData;

// ── IMU helpers ───────────────────────────────────────────────────────────────

static void quatToRPY(float qw, float qx, float qy, float qz,
                      float& roll, float& pitch, float& yaw) {
  float sinr = 2.0f*(qw*qx + qy*qz);
  float cosr = 1.0f - 2.0f*(qx*qx + qy*qy);
  roll = atan2f(sinr, cosr) * 180.0f / (float)PI;

  float sinp = 2.0f*(qw*qy - qz*qx);
  pitch = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp)
                                 : asinf(sinp) * 180.0f / (float)PI;

  float siny = 2.0f*(qw*qz + qx*qy);
  float cosy = 1.0f - 2.0f*(qy*qy + qz*qz);
  yaw = atan2f(siny, cosy) * 180.0f / (float)PI;
}

static void setImuReports() {
  bno085.enableReport(SH2_ROTATION_VECTOR,      10000);
  bno085.enableReport(SH2_LINEAR_ACCELERATION,  10000);
  bno085.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
}

static void pollImu() {
  if (!imuPresent) return;
  if (bno085.wasReset()) {
    Serial.println("[IMU] reset — re-enabling reports");
    setImuReports();
  }
  sh2_SensorValue_t val;
  while (bno085.getSensorEvent(&val)) {
    switch (val.sensorId) {
      case SH2_ROTATION_VECTOR:
        imuData.qw     = val.un.rotationVector.real;
        imuData.qx     = val.un.rotationVector.i;
        imuData.qy     = val.un.rotationVector.j;
        imuData.qz     = val.un.rotationVector.k;
        imuData.status = val.status;
        quatToRPY(imuData.qw, imuData.qx, imuData.qy, imuData.qz,
                  imuData.roll, imuData.pitch, imuData.yaw);
        imuData.valid  = true;
        break;
      case SH2_LINEAR_ACCELERATION:
        imuData.ax = val.un.linearAcceleration.x;
        imuData.ay = val.un.linearAcceleration.y;
        imuData.az = val.un.linearAcceleration.z;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        imuData.gx = val.un.gyroscope.x;
        imuData.gy = val.un.gyroscope.y;
        imuData.gz = val.un.gyroscope.z;
        break;
    }
  }
}

static void printImuSerial() {
  static uint32_t lastMs = 0;
  if (millis() - lastMs < 200) return;
  lastMs = millis();
  if (!imuPresent) { Serial.println("[IMU] not present"); return; }
  if (!imuData.valid) { Serial.println("[IMU] waiting..."); return; }
  Serial.printf("[IMU] R:%+7.2f  P:%+7.2f  Y:%+7.2f deg    "
                "Ax:%+7.3f  Ay:%+7.3f  Az:%+7.3f m/s2    "
                "Gx:%+6.3f  Gy:%+6.3f  Gz:%+6.3f rad/s  conf=%u\n",
                imuData.roll, imuData.pitch, imuData.yaw,
                imuData.ax,   imuData.ay,   imuData.az,
                imuData.gx,   imuData.gy,   imuData.gz,
                (unsigned)imuData.status);
}

static void updateOled(uint8_t nGood) {
  char l0[22], l1[22], l2[22], l3[22];
  snprintf(l0, 22, "TAG:%02X  %u/%u OK", TAG_ID, nGood, N_ANCHORS);
  if (!imuPresent) {
    snprintf(l1, 22, "IMU not found");
    l2[0] = l3[0] = '\0';
  } else if (!imuData.valid) {
    snprintf(l1, 22, "IMU init..."); l2[0] = l3[0] = '\0';
  } else {
    snprintf(l1, 22, "R:%+6.1f P:%+6.1f",   imuData.roll,  imuData.pitch);
    snprintf(l2, 22, "Y:%+6.1f deg",          imuData.yaw);
    snprintf(l3, 22, "a:%+5.2f%+5.2f%+5.2f", imuData.ax, imuData.ay, imuData.az);
  }
  oled.show(l0, l1, l2, l3);
}

// ── Self-survey ───────────────────────────────────────────────────────────────
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
      float sum = 0.0f; uint16_t ok = 0;
      pairIdx++;
      Serial.printf("  Pair %u/%u  0x%02X -> 0x%02X:\n", pairIdx, pairs, a, b);
      for (uint16_t s = 0; s < SURVEY_SAMPLES; s++) {
        float dist, rxp;
        if (engine.surveyRequest(a, b, dist, rxp)) { sum += dist; ok++; }
        if ((s & 0xF) == 0xF)
          Serial.printf("    [%u/%u ok=%u]\n", s+1, SURVEY_SAMPLES, ok);
        delay(5);
      }
      if (ok >= SURVEY_SAMPLES / 4) {
        uint32_t mm = (uint32_t)lroundf(sum / ok * 1000.0f);
        snprintf(buf, sizeof(buf), "SURVEY,v1,%u,%u,%lu,%u\n", a, b, mm, ok);
        host.sendRaw(buf);
        Serial.printf("  => %.3f m  (%u/%u ok)\n", sum/ok, ok, SURVEY_SAMPLES);
      } else {
        Serial.printf("  FAILED (ok=%u/%u)\n", ok, SURVEY_SAMPLES);
      }
    }
  }
  host.sendRaw("SURVEY_DONE,v1\n");
  Serial.println("Survey complete.");
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  // DW1000 FIRST — must precede WiFi (mirrors Tag.ino ordering).
  // Serial shows "DECA 01302001" if CS pin is correct; garbage = wrong pin.
  engine.begin(TWR_TAG, TAG_ID, ANTENNA_DELAY);
  scheduler.begin(&engine, ANCHORS, N_ANCHORS);
  engine.printDeviceId();

  oled.begin();

  Wire1.begin(BNO_SDA, BNO_SCL);
  if (bno085.begin_I2C(BNO_ADDR, &Wire1)) {
    imuPresent = true;
    setImuReports();
  }

#if defined(UWB_HOSTLINK_UDP)
  host.begin(WIFI_SSID, WIFI_PASS, HOST_IP, HOST_PORT);
  Wire.end(); delay(50); oled.begin();
  if (imuPresent) { Wire1.end(); delay(20); Wire1.begin(BNO_SDA, BNO_SCL); }
#else
  host.begin(115200);
#endif

  Serial.printf("Tag 0x%02X ready — CS=%d  %u anchors  IMU:%s\n",
                TAG_ID, UWB_PIN_SS, N_ANCHORS, imuPresent ? "YES" : "NO");
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
  static char    cmdBuf[16];
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

  pollImu();
  uint8_t nGood = scheduler.sweep();
  pollImu();

  ImuSample imuSamp;
  if (imuPresent && imuData.valid) {
    imuSamp.valid  = true;
    imuSamp.status = imuData.status;
    imuSamp.qw = imuData.qw;  imuSamp.qx = imuData.qx;
    imuSamp.qy = imuData.qy;  imuSamp.qz = imuData.qz;
    imuSamp.ax = imuData.ax;  imuSamp.ay = imuData.ay;  imuSamp.az = imuData.az;
    imuSamp.gx = imuData.gx;  imuSamp.gy = imuData.gy;  imuSamp.gz = imuData.gz;
  }

  host.sendSweep(millis(), TAG_ID, scheduler,
                 (imuPresent && imuData.valid) ? &imuSamp : nullptr);

  printImuSerial();
  updateOled(nGood);
}
