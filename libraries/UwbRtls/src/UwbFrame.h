/*
 * UwbFrame.h - Minimal application framing for the UwbRtls protocol.
 *
 * We do NOT use 802.15.4 MAC framing (that, plus the auto-discovery state
 * machine, is exactly where the upstream library's 4-anchor / single-tag
 * limits live). Instead every UWB payload starts with a tiny fixed header and
 * the receiver filters by destination address in software, so the number of
 * anchors is bounded only by your address space, not by the protocol.
 *
 *   byte[0] = message type   (UwbMsgType)
 *   byte[1] = source short address
 *   byte[2] = destination short address (UWB_ADDR_BROADCAST = 0xFF)
 *   byte[3] = sequence number
 *   byte[4..] = type-specific payload
 *
 * The DW1000's hardware frame-check (2-byte CRC) is left enabled, so corrupt
 * frames are dropped by the radio before we ever see them.
 */
#ifndef UWBRTLS_UWBFRAME_H
#define UWBRTLS_UWBFRAME_H

#include <Arduino.h>
#include "dw1000/DW1000Time.h"

// Message types (kept numerically compatible with the classic POLL/RANGE flow).
enum UwbMsgType : uint8_t {
  MSG_POLL         = 0,  // tag  -> anchor : start of an exchange
  MSG_POLL_ACK     = 1,  // anchor -> tag  : acknowledge
  MSG_RANGE        = 2,  // tag  -> anchor : carries tag's 3 timestamps
  MSG_RANGE_REPORT = 3,  // anchor -> tag  : carries computed distance + rx power
  MSG_RANGE_FAILED = 255,
  // Reserved for the future multi-tag superframe (not used yet):
  MSG_ANNOUNCE     = 10,
  MSG_SLOT_GRANT   = 11,
};

// Header / payload layout.
static const uint8_t UWB_HDR_LEN        = 4;   // type, src, dst, seq
static const uint8_t UWB_TS_LEN         = 5;   // one DW1000 40-bit timestamp
static const uint8_t UWB_FRAME_MAXLEN   = 32;  // generous upper bound

// RANGE payload = 3 timestamps (pollSent, pollAckReceived, rangeSent).
static const uint8_t UWB_RANGE_PAYLOAD_LEN = 3 * UWB_TS_LEN;          // 15
static const uint8_t UWB_RANGE_LEN         = UWB_HDR_LEN + UWB_RANGE_PAYLOAD_LEN; // 19

// RANGE_REPORT payload = float distance(m) + float rx power(dBm).
static const uint8_t UWB_REPORT_PAYLOAD_LEN = 2 * sizeof(float);      // 8
static const uint8_t UWB_REPORT_LEN         = UWB_HDR_LEN + UWB_REPORT_PAYLOAD_LEN; // 12

// --- header accessors -------------------------------------------------------
inline uint8_t frameType(const byte* f) { return f[0]; }
inline uint8_t frameSrc (const byte* f) { return f[1]; }
inline uint8_t frameDst (const byte* f) { return f[2]; }
inline uint8_t frameSeq (const byte* f) { return f[3]; }

// Build the 4-byte header into f. Returns bytes written (UWB_HDR_LEN).
uint8_t writeHeader(byte* f, uint8_t type, uint8_t src, uint8_t dst, uint8_t seq);

// True if this frame is addressed to us (exact match or broadcast).
inline bool frameIsForUs(const byte* f, uint8_t myAddr) {
  return frameDst(f) == myAddr || frameDst(f) == UWB_ADDR_BROADCAST;
}

// --- RANGE payload pack/unpack (3 timestamps) ------------------------------
void packRangePayload(byte* f, const DW1000Time& pollSent,
                      const DW1000Time& pollAckReceived,
                      const DW1000Time& rangeSent);
void unpackRangePayload(const byte* f, DW1000Time& pollSent,
                        DW1000Time& pollAckReceived,
                        DW1000Time& rangeSent);

// --- RANGE_REPORT payload pack/unpack --------------------------------------
void packReportPayload(byte* f, float distanceMeters, float rxPowerDbm);
void unpackReportPayload(const byte* f, float& distanceMeters, float& rxPowerDbm);

#endif // UWBRTLS_UWBFRAME_H
