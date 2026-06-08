#include "UwbScheduler.h"

void UwbScheduler::begin(TwrEngine* engine, const uint8_t* anchorAddrs, uint8_t n) {
  _engine = engine;
  _n = (n > UWB_MAX_ANCHORS) ? UWB_MAX_ANCHORS : n;
  for (uint8_t i = 0; i < _n; i++) {
    _addrs[i]        = anchorAddrs[i];
    _results[i].id   = anchorAddrs[i];
    _results[i].valid = false;
  }
}

uint8_t UwbScheduler::sweep() {
  uint8_t good = 0;
  _sweepSeq++;
  for (uint8_t i = 0; i < _n; i++) {
    float dist = 0.0f, rxp = 0.0f;
    bool ok = _engine->rangeTo(_addrs[i], dist, rxp);
    _results[i].id      = _addrs[i];
    _results[i].valid   = ok;
    _results[i].distance = ok ? dist : 0.0f;
    _results[i].rxPower  = ok ? rxp  : 0.0f;
    if (ok) good++;
  }
  return good;
}
