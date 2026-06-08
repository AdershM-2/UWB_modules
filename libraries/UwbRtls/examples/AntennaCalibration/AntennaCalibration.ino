/*
 * AntennaCalibration.ino - Per-board antenna-delay calibration.
 *
 * Antenna delay is the single biggest error source in DW1000 ranging. Skipping
 * this typically leaves 30-50 cm of bias; doing it gets you to ~10 cm.
 *
 * Procedure:
 *   1. Flash one OTHER board as a normal Anchor (examples/Anchor) with a fixed
 *      antenna delay (the default 16384 is fine for the reference).
 *   2. Place the two boards a known, accurately measured LOS distance apart
 *      (7.0 m is a good choice). Antennas facing each other, clear line of sight.
 *   3. Flash THIS sketch to the board under test, set TRUE_DISTANCE_M and
 *      REF_ANCHOR_ID below, and open the Serial Monitor at 115200.
 *   4. It binary-searches this board's antenna delay until the measured mean
 *      matches the true distance, then prints the value to paste into the
 *      board's Anchor/Tag sketch (ANTENNA_DELAY).
 *
 * Note: relationship is monotonic - increasing antenna delay DECREASES the
 * reported distance.
 */
#define UWB_HOSTLINK_SERIAL
#include <UwbRtls.h>

// >>>>>>>>>>>>>>>>> CONFIGURE <<<<<<<<<<<<<<<<<<
static const float    TRUE_DISTANCE_M = 7.00f;   // measured tape distance
static const uint8_t  REF_ANCHOR_ID   = 0x01;    // the reference board's ID
static const uint8_t  THIS_ID         = UWB_ADDR_TAG_BASE;  // this board's addr
static const uint16_t DELAY_LOW       = 15800;   // search bounds (ticks)
static const uint16_t DELAY_HIGH      = 16900;
static const uint8_t  SAMPLES_PER_STEP = 40;     // averaged measurements/step
static const uint8_t  SEARCH_ITERS     = 12;     // ~ resolves to <1 tick
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

TwrEngine engine;

// Average measured distance to the reference, at the current antenna delay.
// Returns NAN if too few exchanges succeeded.
static float measureMean(uint8_t samples) {
  float sum = 0.0f;
  uint16_t ok = 0;
  for (uint8_t i = 0; i < samples; i++) {
    float d, q;
    if (engine.rangeTo(REF_ANCHOR_ID, d, q)) {
      sum += d;
      ok++;
    }
    delay(5);
  }
  if (ok < samples / 4) return NAN;
  return sum / ok;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  engine.begin(TWR_TAG, THIS_ID, DELAY_LOW);
  engine.printDeviceId();
  Serial.printf("Calibrating against anchor 0x%02X at %.3f m\n",
                REF_ANCHOR_ID, TRUE_DISTANCE_M);

  uint16_t low = DELAY_LOW, high = DELAY_HIGH;
  uint16_t mid = (low + high) / 2;

  for (uint8_t it = 0; it < SEARCH_ITERS; it++) {
    mid = (low + high) / 2;
    engine.setAntennaDelay(mid);
    float mean = measureMean(SAMPLES_PER_STEP);
    if (isnan(mean)) {
      Serial.println(F("  too few replies - check link / distance"));
      delay(500);
      continue;
    }
    float err = mean - TRUE_DISTANCE_M;
    Serial.printf("  delay=%u  mean=%.3f m  err=%+.3f m\n", mid, mean, err);
    if (mean > TRUE_DISTANCE_M) {
      low = mid;     // reported too far -> need MORE delay
    } else {
      high = mid;    // reported too near -> need LESS delay
    }
  }

  Serial.println();
  Serial.printf("==> Calibrated ANTENNA_DELAY = %u\n", mid);
  Serial.println(F("Paste this into this board's Anchor.ino / Tag.ino."));
}

void loop() {
  // Continuous readout at the converged delay for a final sanity check.
  float d, q;
  if (engine.rangeTo(REF_ANCHOR_ID, d, q)) {
    Serial.printf("d=%.3f m  (true %.3f)  rxPower=%.1f dBm\n",
                  d, TRUE_DISTANCE_M, q);
  }
  delay(100);
}
