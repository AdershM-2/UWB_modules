/*
 * SensorImu.h - BNO085 IMU interface (STUB for now; wired up in a later phase).
 *
 * Designed-in so the rest of the system (host packet schema, Python EKF) is
 * already IMU-ready. When you add the BNO085 to a tag, use the TagWrover sketch
 * which drives the BNO085 on Wire1 (GPIO 25/26) separately from the OLED bus,
 * and populates an ImuSample before calling host.sendSweep().
 */
#ifndef UWBRTLS_SENSORIMU_H
#define UWBRTLS_SENSORIMU_H

#include <Arduino.h>

struct ImuSample {
  bool    valid  = false;
  uint8_t status = 0;               // BNO085 fusion accuracy 0-3 (3 = highest)
  // Orientation as a unit quaternion (rotation vector, body->world).
  float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
  // Linear acceleration (gravity removed), m/s^2, body frame.
  float ax = 0.0f, ay = 0.0f, az = 0.0f;
  // Calibrated angular velocity, rad/s, body frame.
  float gx = 0.0f, gy = 0.0f, gz = 0.0f;
};

class SensorImu {
public:
  // Returns false until the BNO085 driver is implemented.
  bool begin() { _present = false; return _present; }

  // Populate `out` with the latest sample. Returns false in the stub.
  bool read(ImuSample& out) {
    out.valid = false;
    return false;
  }

  bool present() const { return _present; }

private:
  bool _present = false;
};

#endif // UWBRTLS_SENSORIMU_H
