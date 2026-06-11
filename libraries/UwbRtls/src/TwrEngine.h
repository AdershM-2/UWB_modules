/*
 * TwrEngine.h - Double-sided two-way ranging (DS-TWR) over the DW1000.
 *
 * One class, two roles:
 *   - TAG (initiator):  call rangeTo(anchorAddr, ...) to measure one distance.
 *   - ANCHOR (responder): call serviceResponder() every loop() iteration.
 *
 * The ranging math is the proven asymmetric DS-TWR estimator (clock-offset
 * cancelling); only the addressing/scheduling around it is ours, which is what
 * removes the upstream library's ~4-anchor and single-tag limits. The number of
 * anchors a tag can range to is unbounded here - the tag simply addresses each
 * anchor in turn (see UwbScheduler).
 *
 * Threading model mirrors the DW1000 driver: the IRQ handler only sets volatile
 * flags; all SPI work happens in our (polled) context.
 */
#ifndef UWBRTLS_TWRENGINE_H
#define UWBRTLS_TWRENGINE_H

#include <Arduino.h>
#include "UwbConfig.h"
#include "UwbFrame.h"
#include "dw1000/DW1000.h"

enum TwrRole : uint8_t { TWR_TAG, TWR_ANCHOR };

class TwrEngine {
public:
  // Configure the radio and this device's identity. antennaDelay is the
  // per-board CALIBRATED value (DW1000 ticks).
  void begin(TwrRole role, uint8_t myAddr,
             uint16_t antennaDelay = UWB_DEFAULT_ANTENNA_DELAY);

  // Re-apply configuration with a new antenna delay (used by calibration).
  void setAntennaDelay(uint16_t antennaDelay);

  // ---- TAG role -----------------------------------------------------------
  // Perform one full DS-TWR exchange with the given anchor.
  // Returns true on success; outputs distance in metres and RX power in dBm.
  bool rangeTo(uint8_t anchorAddr, float& distanceMeters, float& rxPowerDbm);

  // ---- ANCHOR role --------------------------------------------------------
  // Non-blocking: handle any pending POLL / RANGE and reply. Call from loop().
  void serviceResponder();

  // Last successful responder distance/peer (for status display).
  float   lastDistance() const { return _lastDistance; }
  uint8_t lastPeer()     const { return _lastPeer; }

  // Print the chip's device identifier to Serial (bring-up / SPI sanity check).
  void printDeviceId();

private:
  void configure();           // applies role-independent radio config
  void startRx();             // (re-)arm permanent receive
  bool waitSent(uint32_t timeoutMs);
  bool waitReceived(uint32_t timeoutMs);
  uint16_t readFrame();        // pull RX buffer into _rx, return length

  TwrRole  _role     = TWR_ANCHOR;
  uint8_t  _myAddr   = UWB_ADDR_INVALID;
  uint16_t _antDelay = UWB_DEFAULT_ANTENNA_DELAY;
  uint8_t  _seq      = 0;

  byte _tx[UWB_FRAME_MAXLEN];
  byte _rx[UWB_FRAME_MAXLEN];

  // Exchange timestamps.
  DW1000Time _timePollSent, _timePollAckReceived, _timeRangeSent;          // tag side
  DW1000Time _timePollReceived, _timePollAckSent, _timeRangeReceived;      // anchor side

  uint8_t _peer        = UWB_ADDR_INVALID; // current responder's partner
  float   _lastDistance = 0.0f;
  uint8_t _lastPeer     = UWB_ADDR_INVALID;

  // TAG watchdog: consecutive failures → full radio reset.
  static constexpr uint8_t FAIL_STREAK_RESET = 5;
  uint8_t _failStreak = 0;

  // ANCHOR watchdog: if no frame received for this many ms, reset the radio.
  // The DW1000 RXAUTR erratum means the receiver can wedge after a CRC/LDE
  // error and never fire the ISR again — a time-based reset catches it.
  static constexpr uint32_t ANCHOR_RX_WATCHDOG_MS = 500;
  uint32_t _lastRxMs = 0;

  // ISR trampolines + flags (single engine instance per device).
  static TwrEngine* _instance;
  static volatile bool _sentFlag;
  static volatile bool _receivedFlag;
  static void onSent();
  static void onReceived();
};

#endif // UWBRTLS_TWRENGINE_H
