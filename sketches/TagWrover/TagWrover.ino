/*
 * TagWrover.ino  –  UWB tag for a generic ESP32-WROVER + DW1000 module
 *                   with external SSD1306 OLED and BNO085 IMU.
 *
 * Peripheral map
 * ──────────────────────────────────────────────────────────────────────
 *  DW1000 (SPI) : SCK=18  MISO=19  MOSI=23  CS=4   RST=27  IRQ=34
 *  SSD1306 OLED : SDA=32  SCL=33   (Wire,  I2C address 0x3C or 0x3D)
 *  BNO085 IMU   : SDA=25  SCL=26   (Wire1, I2C address 0x4A  ← BNO_ADDR)
 *
 *  GPIO 4 = DW1000 CS on this board, so the default OLED SDA=4 conflicts.
 *  OLED moved to 16/17 (Wire). BNO085 on separate bus 25/26 (Wire1).
 *
 * ⚠  PSRAM note: GPIO 16/17 are used for PSRAM on some WROVER modules.
 *    If OLED fails to appear, check Arduino IDE → Tools → PSRAM → Disabled,
 *    or move OLED to 32/33 and update OLED_PIN_SDA/SCL above.
 *
 * Required libraries (install via Arduino Library Manager):
 *   – Adafruit BNO08x          (Adafruit)
 *   – Adafruit SSD1306          (Adafruit)
 *   – Adafruit GFX Library      (Adafruit)
 *
 * OLED display layout (128×64, text size 1):
 *   Row 0 (y= 0): "TAG:F0  4/4"        ← tag ID + anchors OK / total
 *   Row 1 (y=16): "R:+12.3  P: -5.6"   ← roll, pitch (degrees)
 *   Row 2 (y=32): "Y:+045.2 deg"        ← yaw (degrees)
 *   Row 3 (y=48): "a:+0.12-0.03+9.81"  ← linear acceleration (m/s²)
 *
 * Serial output (every 200 ms):
 *   [IMU] R:+12.30 P: -5.60 Y:+45.20 deg   Ax:+0.120 Ay:-0.030 Az:+9.810 m/s2
 *
 * The IMU tail (,IMU,status,qw,qx,qy,qz,ax,ay,az,gx,gy,gz) is appended to every
 * RTLS packet when BNO085 is present, making the data available to the Python host.
 */

// ── Transport: pick exactly ONE ───────────────────────────────────────────────
// #define UWB_HOSTLINK_SERIAL
#define UWB_HOSTLINK_UDP
#define UWB_USE_OLED

// ── Board pin overrides — MUST appear before #include <UwbRtls.h> ────────────
// ESP32 UWB (non-Pro) uses DW1000 CS=4, not 21.
// GPIO 4 is also the default OLED SDA in UwbConfig.h — conflict!
// Both SPI-CS and I2C-SDA cannot share the same pin.
// Fix: keep DW1000 CS=4 (hardware-wired) and move OLED to GPIO 32/33.
// Rewire your OLED: SDA → GPIO 32, SCL → GPIO 33.
#define UWB_PIN_SS    4    // DW1000 chip select (hardware-wired on this board)
#define OLED_PIN_SDA 32    // OLED I2C SDA — moved off GPIO 4 to avoid CS clash
#define OLED_PIN_SCL 33    // OLED I2C SCL

#include <UwbRtls.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ── Configuration — edit these for your setup ─────────────────────────────────
static const uint8_t  TAG_ID        = 0xF0;          // 0xF0
static const uint16_t ANTENNA_DELAY = 16473;  // calibrate!

static const uint8_t ANCHORS[]  = { 0x01, 0x02, 0x03, 0x04 };
static const uint8_t N_ANCHORS  = sizeof(ANCHORS) / sizeof(ANCHORS[0]);

#if defined(UWB_HOSTLINK_UDP)
static const char*    WIFI_SSID = "iitk";          // ← fill in
static const char*    WIFI_PASS = "";                    // ← "" for open network
static IPAddress      HOST_IP(255, 255, 255, 255);       // ← Python/MATLAB PC IP
static const uint16_t HOST_PORT  = 4100;
#endif

// BNO085 I2C bus  ── see PSRAM warning above ──────────────────────────────────
#define BNO_SDA   25   // Wire1 SDA — free from OLED and DW1000
#define BNO_SCL   26   // Wire1 SCL
#define BNO_ADDR  0x4A // SA0 LOW (default); use 0x4B if SA0 is tied HIGH

// Survey averaging samples (100 per anchor pair)
static const uint16_t SURVEY_SAMPLES = 100;

// ── Objects ───────────────────────────────────────────────────────────────────
TwrEngine    engine;
UwbScheduler scheduler;
HostLink     host;
OledStatus   oled;

Adafruit_BNO08x bno085(-1);    // -1 = MCU does not drive the reset pin
static bool     imuPresent = false;

// Latest IMU readings — updated by pollImu()
static struct {
  float   qw=1, qx=0, qy=0, qz=0;   // rotation vector (unit quaternion)
  float   ax=0, ay=0, az=0;          // linear acceleration, m/s² (gravity removed)
  float   gx=0, gy=0, gz=0;          // calibrated angular velocity, rad/s
  float   roll=0, pitch=0, yaw=0;    // Euler angles, degrees
  uint8_t status = 0;                 // BNO085 accuracy 0–3 (from val.status)
  bool    valid  = false;
} imuData;

// ── IMU helpers ───────────────────────────────────────────────────────────────

static void quatToRPY(float qw, float qx, float qy, float qz,
                      float& roll, float& pitch, float& yaw) {
  // Roll (rotation about X axis)
  float sinr = 2.0f * (qw*qx + qy*qz);
  float cosr = 1.0f - 2.0f * (qx*qx + qy*qy);
  roll = atan2f(sinr, cosr) * 180.0f / (float)PI;

  // Pitch (rotation about Y axis)
  float sinp = 2.0f * (qw*qy - qz*qx);
  pitch = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp)
                                 : asinf(sinp) * 180.0f / (float)PI;

  // Yaw (rotation about Z axis)
  float siny = 2.0f * (qw*qz + qx*qy);
  float cosy = 1.0f - 2.0f * (qy*qy + qz*qz);
  yaw = atan2f(siny, cosy) * 180.0f / (float)PI;
}

static void setImuReports() {
  // 10 000 µs interval = 100 Hz output rate for all reports.
  bno085.enableReport(SH2_ROTATION_VECTOR,       10000);
  bno085.enableReport(SH2_LINEAR_ACCELERATION,   10000);
  bno085.enableReport(SH2_GYROSCOPE_CALIBRATED,  10000);
}

// Drain the BNO085 FIFO.  Safe to call at any time; no-op if IMU absent.
static void pollImu() {
  if (!imuPresent) return;

  // If the sensor rebooted (e.g. brown-out), re-enable reports.
  if (bno085.wasReset()) {
    Serial.println("[IMU] BNO085 reset detected — re-enabling reports");
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
        imuData.status = val.status;   // 0–3 fusion accuracy from BNO085
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
      default:
        break;
    }
  }
}

// Throttled Serial print (every 200 ms)
static void printImuSerial() {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < 200) return;
  lastMs = now;

  if (!imuPresent) {
    Serial.println("[IMU] not present");
    return;
  }
  if (!imuData.valid) {
    Serial.println("[IMU] waiting for first sample...");
    return;
  }
  Serial.printf("[IMU] R:%+7.2f  P:%+7.2f  Y:%+7.2f deg    "
                "Ax:%+7.3f  Ay:%+7.3f  Az:%+7.3f m/s2    "
                "Gx:%+6.3f  Gy:%+6.3f  Gz:%+6.3f rad/s  conf=%u\n",
                imuData.roll, imuData.pitch, imuData.yaw,
                imuData.ax,   imuData.ay,   imuData.az,
                imuData.gx,   imuData.gy,   imuData.gz,
                (unsigned)imuData.status);
}

// Update OLED: UWB status on row 0, IMU data on rows 1-3.
static void updateOled(uint8_t nGood) {
  char l0[22], l1[22], l2[22], l3[22];
  snprintf(l0, 22, "TAG:%02X  %u/%u OK", TAG_ID, nGood, N_ANCHORS);
  if (!imuPresent) {
    snprintf(l1, 22, "IMU not found");
    l2[0] = l3[0] = '\0';
  } else if (!imuData.valid) {
    snprintf(l1, 22, "IMU init...");
    l2[0] = l3[0] = '\0';
  } else {
    snprintf(l1, 22, "R:%+6.1f P:%+6.1f",  imuData.roll,  imuData.pitch);
    snprintf(l2, 22, "Y:%+6.1f deg",        imuData.yaw);
    snprintf(l3, 22, "a:%+5.2f%+5.2f%+5.2f",
             imuData.ax, imuData.ay, imuData.az);
  }
  oled.show(l0, l1, l2, l3);
}

// ── Anchor self-survey (same as the standard Tag sketch) ─────────────────────
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

      char oledL1[20], oledL2[20];
      snprintf(oledL1, sizeof(oledL1), "%u/%u  %02X->%02X", pairIdx, pairs, a, b);
      Serial.printf("  Pair %u/%u  0x%02X -> 0x%02X:\n", pairIdx, pairs, a, b);

      for (uint16_t s = 0; s < SURVEY_SAMPLES; s++) {
        float dist, rxp;
        if (engine.surveyRequest(a, b, dist, rxp)) { sum += dist; ok++; }
        if ((s & 0xF) == 0xF) {
          snprintf(oledL2, sizeof(oledL2), "%u/%u ok=%u", s+1, SURVEY_SAMPLES, ok);
          oled.showSplash("SURVEY", oledL1, oledL2);
          Serial.printf("    [%u/%u ok=%u]\n", s + 1, SURVEY_SAMPLES, ok);
        }
        delay(5);
      }

      if (ok >= SURVEY_SAMPLES / 4) {
        uint32_t mm = (uint32_t)lroundf(sum / ok * 1000.0f);
        snprintf(buf, sizeof(buf), "SURVEY,v1,%u,%u,%lu,%u\n", a, b, mm, ok);
        host.sendRaw(buf);
        Serial.printf("  => %.3f m  (%u/%u ok)\n", sum / ok, ok, SURVEY_SAMPLES);
      } else {
        Serial.printf("  FAILED (ok=%u/%u) — check link 0x%02X<->0x%02X\n",
                      ok, SURVEY_SAMPLES, a, b);
      }
    }
  }
  host.sendRaw("SURVEY_DONE,v1\n");
  Serial.println("Survey complete.");
  oled.showSplash("SURVEY", "DONE", "See host PC");
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);

  // ── UWB radio FIRST — must precede WiFi (mirrors working Tag.ino order) ───
  // engine.printDeviceId() prints the DW1000 chip ID.
  // "DECA 01302001" → CS pin correct and SPI talking.
  // All zeros or garbage → CS pin wrong for this board.
  engine.begin(TWR_TAG, TAG_ID, ANTENNA_DELAY);
  scheduler.begin(&engine, ANCHORS, N_ANCHORS);
  engine.printDeviceId();

  // ── OLED on Wire (pins 4, 5) ──────────────────────────────────────────────
  oled.begin();
  oled.showSplash("TAG WROVER", "UWB OK", "BNO085...");

  // ── BNO085 on Wire1 (pins BNO_SDA, BNO_SCL) ──────────────────────────────
  Wire1.begin(BNO_SDA, BNO_SCL);
  Serial.printf("[IMU] probing BNO085 at 0x%02X on SDA=%d SCL=%d ...\n",
                BNO_ADDR, BNO_SDA, BNO_SCL);

  if (bno085.begin_I2C(BNO_ADDR, &Wire1)) {
    imuPresent = true;
    setImuReports();
    Serial.println("[IMU] BNO085 found and configured");
    oled.showSplash("TAG WROVER", "BNO085 OK", "WiFi...");
  } else {
    imuPresent = false;
    Serial.println("[IMU] BNO085 NOT FOUND — check wiring and BNO_SDA/SCL above.");
    oled.showSplash("TAG WROVER", "IMU MISSING", "Check pins");
    delay(2000);
  }

  // ── WiFi / transport init (AFTER DW1000) ─────────────────────────────────
#if defined(UWB_HOSTLINK_UDP)
  host.begin(WIFI_SSID, WIFI_PASS, HOST_IP, HOST_PORT);
  // WiFi association can glitch I2C buses; re-init both afterwards.
  Wire.end();
  delay(50);
  oled.begin();
  if (imuPresent) {
    Wire1.end();
    delay(20);
    Wire1.begin(BNO_SDA, BNO_SCL);
  }
#else
  host.begin(115200);
#endif

  Serial.printf("Tag 0x%02X ready — CS=%d  %u anchors  IMU:%s\n",
                TAG_ID, UWB_PIN_SS, N_ANCHORS, imuPresent ? "YES" : "NO");
  char l2[20];
  snprintf(l2, 20, "%u anch  IMU:%s", N_ANCHORS, imuPresent ? "OK" : "NO");
  oled.showSplash("TAG WROVER", "Ready", l2, "Ranging...");
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // ── Serial command check (survey trigger) ─────────────────────────────────
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

  // ── IMU poll before sweep (drain any queued BNO085 events) ───────────────
  pollImu();

  // ── UWB ranging sweep (~100 ms for 4 anchors at 10 Hz) ───────────────────
  uint8_t nGood = scheduler.sweep();

  // ── IMU poll after sweep (more data accumulated during the ranging wait) ──
  pollImu();

  // ── Build ImuSample for the RTLS host packet ──────────────────────────────
  ImuSample imuSamp;
  if (imuPresent && imuData.valid) {
    imuSamp.valid  = true;
    imuSamp.status = imuData.status;
    imuSamp.qw = imuData.qw;  imuSamp.qx = imuData.qx;
    imuSamp.qy = imuData.qy;  imuSamp.qz = imuData.qz;
    imuSamp.ax = imuData.ax;  imuSamp.ay = imuData.ay;  imuSamp.az = imuData.az;
    imuSamp.gx = imuData.gx;  imuSamp.gy = imuData.gy;  imuSamp.gz = imuData.gz;
  }

  // ── Send RTLS packet to host (IMU tail appended when valid) ───────────────
  host.sendSweep(millis(), TAG_ID, scheduler,
                 (imuPresent && imuData.valid) ? &imuSamp : nullptr);

  // ── Human-readable outputs ────────────────────────────────────────────────
  printImuSerial();
  updateOled(nGood);
}
