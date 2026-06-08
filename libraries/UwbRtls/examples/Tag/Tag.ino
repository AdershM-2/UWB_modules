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
  Serial.printf("Tag 0x%02X ready, %u anchors\n", TAG_ID, N_ANCHORS);
}

void loop() {
  uint8_t good = scheduler.sweep();

  ImuSample s;
  bool hasImu = imu.read(s);     // false in the stub
  host.sendSweep(millis(), TAG_ID, scheduler, hasImu ? &s : nullptr);

  char l0[24], l1[24];
  snprintf(l0, sizeof(l0), "TAG sweep %lu", (unsigned long)scheduler.sweepSeq());
  snprintf(l1, sizeof(l1), "%u/%u anchors", good, N_ANCHORS);
  oled.show(l0, l1);
}
