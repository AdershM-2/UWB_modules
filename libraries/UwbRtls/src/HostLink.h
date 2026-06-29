/*
 * HostLink.h - Stream one range-sweep packet to the host (Python or MATLAB).
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
 * Wire format (versioned ASCII line, trivial bandwidth, easy parse):
 *   RTLS,v3,<t_ms>,<tag_id>,<n>,<id1>,<d1_mm>,<rx1_dbm>,<fp1_dbm>,<q1>,...\n
 *        [,IMU,<status>,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>]
 *   - v3 adds first-path power (fp_dbm) and quality per anchor, plus BNO085
 *     accuracy status and gyroscope (gx,gy,gz) to the IMU tail.
 *   - v1/v2 are accepted by the Python parser for backward compatibility.
 *   - The IMU,... tail is appended only when an IMU sample is present.
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
  // Connect to WiFi and target the host's UDP port.
  void begin(const char* ssid, const char* pass, IPAddress host, uint16_t port,
             unsigned long serialBaud = 115200) {
    Serial.begin(serialBaud);
    _host = host;
    _port = port;
    _ssid = ssid;
    _pass = pass;
    _wifiConnect();
  }

  // Reconnect WiFi if it has dropped. Called lazily from sendSweep/sendRaw.
  void checkWifi() {
    if (WiFi.status() != WL_CONNECTED) {
      uint32_t now = millis();
      if (now - _lastReconnectMs < 10000) return;   // back off: try every 10 s
      _lastReconnectMs = now;
      Serial.println(F("[WIFI] disconnected — reconnecting..."));
      _wifiConnect();
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
    checkWifi();
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
    char buf[768];
    int len = format(buf, sizeof(buf), tMs, tagId, sched, imu);
    if (len <= 0) return;

#if defined(UWB_HOSTLINK_UDP)
    checkWifi();
    if (WiFi.status() == WL_CONNECTED) {
      _udp.beginPacket(_host, _port);
      _udp.write(reinterpret_cast<const uint8_t*>(buf), len);
      _udp.endPacket();
      _udpDrops = 0;
    } else {
      _udpDrops++;
      if (_udpDrops == 1 || (_udpDrops & 0x3F) == 0)
        Serial.printf("[WIFI] UDP drop #%u (not connected) host=%d.%d.%d.%d\n",
                      _udpDrops, _host[0], _host[1], _host[2], _host[3]);
    }
    // Mirror to serial too, handy while debugging.
    Serial.write(buf, len);
#else
    Serial.write(buf, len);
#endif
  }

private:
  // Build the v3 line into buf; returns number of bytes written (incl. '\n').
  static int format(char* buf, size_t size, uint32_t tMs, uint8_t tagId,
                    const UwbScheduler& sched, const ImuSample* imu) {
    uint8_t nValid = 0;
    for (uint8_t i = 0; i < sched.anchorCount(); i++)
      if (sched.result(i).valid) nValid++;

    int p = snprintf(buf, size, "RTLS,v3,%lu,%u,%u",
                     (unsigned long)tMs, (unsigned)tagId, (unsigned)nValid);
    if (p < 0 || (size_t)p >= size) return -1;

    for (uint8_t i = 0; i < sched.anchorCount(); i++) {
      const RangeResult& r = sched.result(i);
      if (!r.valid) continue;
      long  mm = lround(r.distance * 1000.0f);
      int   q  = (int)lround(r.rxPower);
      int w = snprintf(buf + p, size - p, ",%u,%ld,%d,%.1f,%.2f",
                       (unsigned)r.id, mm, q, r.fpPower, r.quality);
      if (w < 0 || (size_t)(p + w) >= size) return -1;
      p += w;
    }

    if (imu && imu->valid) {
      int w = snprintf(buf + p, size - p,
                       ",IMU,%u,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f",
                       (unsigned)imu->status,
                       imu->qw, imu->qx, imu->qy, imu->qz,
                       imu->ax, imu->ay, imu->az,
                       imu->gx, imu->gy, imu->gz);
      if (w < 0 || (size_t)(p + w) >= size) return -1;
      p += w;
    }

    if ((size_t)(p + 1) >= size) return -1;
    buf[p++] = '\n';
    return p;
  }

#if defined(UWB_HOSTLINK_UDP)
  WiFiUDP      _udp;
  IPAddress    _host;
  uint16_t     _port          = 0;
  const char*  _ssid          = nullptr;
  const char*  _pass          = nullptr;
  uint32_t     _lastReconnectMs = 0;
  uint32_t     _udpDrops      = 0;

  void _wifiConnect() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(_ssid, _pass);
    Serial.printf("[WIFI] connecting to \"%s\"...", _ssid);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WIFI] connected  IP=%s  host=%d.%d.%d.%d:%u\n",
                    WiFi.localIP().toString().c_str(),
                    _host[0], _host[1], _host[2], _host[3], _port);
      _udp.begin(_port);
      _udpDrops = 0;
    } else {
      Serial.println(F("[WIFI] FAILED — UDP packets will be dropped until reconnect"));
    }
    _lastReconnectMs = millis();
  }
#endif
};

#endif // UWBRTLS_HOSTLINK_H
