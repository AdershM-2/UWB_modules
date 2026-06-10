#include "TwrEngine.h"

// ---- static members --------------------------------------------------------
TwrEngine*      TwrEngine::_instance     = nullptr;
volatile bool   TwrEngine::_sentFlag     = false;
volatile bool   TwrEngine::_receivedFlag = false;

void TwrEngine::onSent()     { _sentFlag = true; }
void TwrEngine::onReceived() { _receivedFlag = true; }

// ---- setup -----------------------------------------------------------------
void TwrEngine::begin(TwrRole role, uint8_t myAddr, uint16_t antennaDelay) {
  _role     = role;
  _myAddr   = myAddr;
  _antDelay = antennaDelay;
  _instance = this;

  DW1000.begin(UWB_PIN_IRQ, UWB_PIN_RST);
  DW1000.select(UWB_PIN_SS);
  configure();

  DW1000.attachSentHandler(TwrEngine::onSent);
  DW1000.attachReceivedHandler(TwrEngine::onReceived);

  startRx();
}

void TwrEngine::configure() {
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(_myAddr);
  DW1000.setNetworkId(UWB_NETWORK_ID);
  DW1000.enableMode(UWB_RADIO_MODE);
  DW1000.setChannel(UWB_CHANNEL);
  // Antenna delay is cached and written to the chip by commitConfiguration().
  DW1000.setAntennaDelay(_antDelay);
  DW1000.commitConfiguration();
}

void TwrEngine::setAntennaDelay(uint16_t antennaDelay) {
  _antDelay = antennaDelay;
  configure();   // re-tune with the new delay
  startRx();
}

void TwrEngine::startRx() {
  _receivedFlag = false;
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(true);  // chip re-arms RX automatically after a frame
  DW1000.startReceive();
}

bool TwrEngine::waitSent(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (!_sentFlag) {
    if (millis() - t0 > timeoutMs) return false;
    yield();
  }
  _sentFlag = false;
  return true;
}

bool TwrEngine::waitReceived(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (!_receivedFlag) {
    if (millis() - t0 > timeoutMs) return false;
    yield();
  }
  _receivedFlag = false;
  return true;
}

uint16_t TwrEngine::readFrame() {
  uint16_t n = DW1000.getDataLength();
  if (n > UWB_FRAME_MAXLEN) n = UWB_FRAME_MAXLEN;
  DW1000.getData(_rx, n);
  return n;
}

// ===========================================================================
// TAG (initiator)
// ===========================================================================
bool TwrEngine::rangeTo(uint8_t anchorAddr, float& distanceMeters, float& rxPowerDbm) {
  _seq++;

  // 1) POLL (immediate). Record our TX timestamp afterwards.
  DW1000.newTransmit();
  DW1000.setDefaults();
  writeHeader(_tx, MSG_POLL, _myAddr, anchorAddr, _seq);
  DW1000.setData(_tx, UWB_HDR_LEN);
  DW1000.startTransmit();
  if (!waitSent(20)) { startRx(); return false; }
  DW1000.getTransmitTimestamp(_timePollSent);

  // Listen for POLL_ACK.
  startRx();
  if (!waitReceived(30)) { startRx(); return false; }
  readFrame();
  if (frameType(_rx) != MSG_POLL_ACK || frameSrc(_rx) != anchorAddr ||
      !frameIsForUs(_rx, _myAddr)) {
    startRx();
    return false;
  }
  DW1000.getReceiveTimestamp(_timePollAckReceived);

  // 2) RANGE (delayed TX so we know our exact send time), carrying our 3 stamps.
  DW1000.newTransmit();
  DW1000.setDefaults();
  writeHeader(_tx, MSG_RANGE, _myAddr, anchorAddr, _seq);
  DW1000Time delay = DW1000Time(UWB_REPLY_DELAY_US, DW1000Time::MICROSECONDS);
  _timeRangeSent = DW1000.setDelay(delay);   // returns the scheduled TX time
  packRangePayload(_tx, _timePollSent, _timePollAckReceived, _timeRangeSent);
  DW1000.setData(_tx, UWB_RANGE_LEN);
  DW1000.startTransmit();
  if (!waitSent(30)) { startRx(); return false; }

  // Listen for RANGE_REPORT (anchor computed the distance).
  startRx();
  if (!waitReceived(40)) { startRx(); return false; }
  readFrame();
  if (frameType(_rx) != MSG_RANGE_REPORT || frameSrc(_rx) != anchorAddr) {
    startRx();
    return false;
  }
  unpackReportPayload(_rx, distanceMeters, rxPowerDbm);
  startRx();
  return true;
}

// ===========================================================================
// ANCHOR (responder)
// ===========================================================================
void TwrEngine::serviceResponder() {
  if (!_receivedFlag) return;
  _receivedFlag = false;

  readFrame();
  if (!frameIsForUs(_rx, _myAddr)) { startRx(); return; }

  const uint8_t type = frameType(_rx);
  const uint8_t src  = frameSrc(_rx);
  const uint8_t seq  = frameSeq(_rx);

  if (type == MSG_POLL) {
    DW1000.getReceiveTimestamp(_timePollReceived);
    _peer = src;

    // Reply POLL_ACK (delayed), then capture our actual TX time.
    DW1000.newTransmit();
    DW1000.setDefaults();
    writeHeader(_tx, MSG_POLL_ACK, _myAddr, src, seq);
    DW1000Time delay = DW1000Time(UWB_REPLY_DELAY_US, DW1000Time::MICROSECONDS);
    DW1000.setDelay(delay);
    DW1000.setData(_tx, UWB_HDR_LEN);
    DW1000.startTransmit();
    if (waitSent(30)) DW1000.getTransmitTimestamp(_timePollAckSent);
    startRx();

  } else if (type == MSG_RANGE && src == _peer) {
    DW1000.getReceiveTimestamp(_timeRangeReceived);

    DW1000Time tPollSent, tPollAckReceived, tRangeSent;
    unpackRangePayload(_rx, tPollSent, tPollAckReceived, tRangeSent);

    // Asymmetric double-sided TWR (cancels clock-frequency offset).
    DW1000Time round1 = (tPollAckReceived   - tPollSent).wrap();
    DW1000Time reply1 = (_timePollAckSent    - _timePollReceived).wrap();
    DW1000Time round2 = (_timeRangeReceived  - _timePollAckSent).wrap();
    DW1000Time reply2 = (tRangeSent          - tPollAckReceived).wrap();

    DW1000Time tof;
    tof.setTimestamp((round1 * round2 - reply1 * reply2) /
                     (round1 + round2 + reply1 + reply2));
    float dist = tof.getAsMeters();
    float rxp  = DW1000.getReceivePower();

    // Reply RANGE_REPORT (immediate) with the computed distance.
    DW1000.newTransmit();
    DW1000.setDefaults();
    writeHeader(_tx, MSG_RANGE_REPORT, _myAddr, src, seq);
    packReportPayload(_tx, dist, rxp);
    DW1000.setData(_tx, UWB_REPORT_LEN);
    DW1000.startTransmit();
    waitSent(30);

    _lastDistance = dist;
    _lastPeer     = src;
    startRx();

  } else {
    startRx();
  }
}

void TwrEngine::printDeviceId() {
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print(F("DW1000 device id: "));
  Serial.println(msg);
}
