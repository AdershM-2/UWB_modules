/*
 * HostLink.h - Stream one range-sweep packet to the MATLAB host.
 *
 * Transport is selected by a COMPILE-TIME flag defined in the sketch BEFORE
 * including this library:
 *     #define UWB_HOSTLINK_UDP      // WiFi/UDP to the host
 *  or #define UWB_HOSTLINK_SERIAL   // USB serial (default if neither defined)
 *
 * This class is header-only on purpose: Arduino compiles library .cpp files in
 * their own translation units, where a #define from the .ino is NOT visible.
 * Being header-only, HostLink is compiled inside the sketch's translation unit,
 * so the sketch's flag actually controls which transport is built.
 *
 * Wire format (versioned ASCII line, trivial bandwidth, easy MATLAB parse):
 *   RTLS,v1,<t_ms>,<tag_id>,<n>,<id1>,<d1_mm>,<q1>,...,<idN>,<dN_mm>,<qN>
 *        [,IMU,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>]\n
 *   - <n> is the count of VALID anchor measurements that follow.
 *   - The IMU,... tail is appended only when an IMU sample is present, so the
 *     format never breaks when the BNO085 is added later.
 */
#ifndef UWBRTLS_HOSTLINK_H
#define UWBRTLS_HOSTLINK_H

#include <Arduino.h>
#include <math.h>
#include "UwbScheduler.h"
#include "SensorImu.h"

// Default to serial if the sketch did not pick a transport.
#if !defined(UWB_HOSTLINK_UDP) && !defined(UWB_HOSTLINK_SERIAL)
#define UWB_HOSTLINK_SERIAL
#endif

#if defined(UWB_HOSTLINK_UDP)
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

class HostLink {
public:
#if defined(UWB_HOSTLINK_UDP)
  // Connect to WiFi and target the MATLAB host's UDP port.
  void begin(const char* ssid, const char* pass, IPAddress host, uint16_t port,
             unsigned long serialBaud = 115200) {
    Serial.begin(serialBaud);
    _host = host;
    _port = port;
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    Serial.print(F("WiFi connecting"));
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print(F("WiFi IP: "));
      Serial.println(WiFi.localIP());
      _udp.begin(_port);
    } else {
      Serial.println(F("WiFi FAILED - packets will be dropped"));
    }
  }
#else
  void begin(unsigned long serialBaud = 115200) {
    Serial.begin(serialBaud);
  }
#endif

  // Send a pre-formatted line as-is (survey control lines, diagnostics, etc.).
  // Caller must include the trailing '\n'.
  void sendRaw(const char* line) {
    size_t len = strlen(line);
#if defined(UWB_HOSTLINK_UDP)
    if (WiFi.status() == WL_CONNECTED) {
      _udp.beginPacket(_host, _port);
      _udp.write(reinterpret_cast<const uint8_t*>(line), len);
      _udp.endPacket();
    }
#endif
    Serial.write(line, len);
  }

  // Format and send one sweep. imu may be nullptr (or invalid) to omit IMU data.
  void sendSweep(uint32_t tMs, uint8_t tagId, const UwbScheduler& sched,
                 const ImuSample* imu = nullptr) {
    char buf[512];
    int len = format(buf, sizeof(buf), tMs, tagId, sched, imu);
    if (len <= 0) return;

#if defined(UWB_HOSTLINK_UDP)
    if (WiFi.status() == WL_CONNECTED) {
      _udp.beginPacket(_host, _port);
      _udp.write(reinterpret_cast<const uint8_t*>(buf), len);
      _udp.endPacket();
    }
    // Mirror to serial too, handy while debugging.
    Serial.write(buf, len);
#else
    Serial.write(buf, len);
#endif
  }

private:
  // Build the line into buf; returns number of bytes written (incl. '\n').
  static int format(char* buf, size_t size, uint32_t tMs, uint8_t tagId,
                    const UwbScheduler& sched, const ImuSample* imu) {
    // Count valid measurements first.
    uint8_t nValid = 0;
    for (uint8_t i = 0; i < sched.anchorCount(); i++)
      if (sched.result(i).valid) nValid++;

    int p = snprintf(buf, size, "RTLS,v1,%lu,%u,%u",
                     (unsigned long)tMs, (unsigned)tagId, (unsigned)nValid);
    if (p < 0 || (size_t)p >= size) return -1;

    for (uint8_t i = 0; i < sched.anchorCount(); i++) {
      const RangeResult& r = sched.result(i);
      if (!r.valid) continue;
      long  mm = lround(r.distance * 1000.0f);
      int   q  = (int)lround(r.rxPower);
      int w = snprintf(buf + p, size - p, ",%u,%ld,%d", (unsigned)r.id, mm, q);
      if (w < 0 || (size_t)(p + w) >= size) return -1;
      p += w;
    }

    if (imu && imu->valid) {
      int w = snprintf(buf + p, size - p,
                       ",IMU,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f",
                       imu->qw, imu->qx, imu->qy, imu->qz,
                       imu->ax, imu->ay, imu->az);
      if (w < 0 || (size_t)(p + w) >= size) return -1;
      p += w;
    }

    if ((size_t)(p + 1) >= size) return -1;
    buf[p++] = '\n';
    return p;
  }

#if defined(UWB_HOSTLINK_UDP)
  WiFiUDP   _udp;
  IPAddress _host;
  uint16_t  _port = 0;
#endif
};

#endif // UWBRTLS_HOSTLINK_H
