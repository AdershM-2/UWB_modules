# 02 — Protocol

## Addressing
1-byte short addresses (our own scheme, not 802.15.4):
- `0x01..0xEF` anchors, `0xF0..0xFE` tags, `0xFF` broadcast (reserved for the
  future multi-tag superframe). Filtering is done in software, so the anchor
  count is bounded only by the address space and sweep time.

## Frame
Every UWB payload starts with a 4-byte header (`UwbFrame.h`):
```
[0] type   [1] src   [2] dst   [3] seq   [4..] payload
```
The DW1000 hardware CRC stays enabled, so corrupt frames are dropped by the radio.

## Ranging: asymmetric double-sided TWR
Per anchor, the tag (initiator) and anchor (responder) exchange 4 messages.
Only the addressed anchor replies, so exchanges never collide.

```
TAG                                   ANCHOR
 │  POLL ───────────────────────────►  │   t: pollSent          r: pollReceived
 │  ◄─────────────────────── POLL_ACK  │   r: pollAckReceived   t: pollAckSent
 │  RANGE (carries 3 tag stamps) ────►  │   t: rangeSent         r: rangeReceived
 │                                      │   → computes ToF/distance
 │  ◄──────────────────── RANGE_REPORT  │   (distance, rx power)
```

The RANGE message embeds the tag’s three timestamps via the DW1000 **delayed
transmit** (so the tag knows its exact send time before sending). The anchor
then has all six timestamps and computes time-of-flight with the standard
clock-offset-cancelling estimator:

```
round1 = pollAckReceived − pollSent      (tag)
reply1 = pollAckSent     − pollReceived  (anchor)
round2 = rangeReceived   − pollAckSent   (anchor)
reply2 = rangeSent       − pollAckReceived(tag)
ToF = (round1·round2 − reply1·reply2) / (round1 + round2 + reply1 + reply2)
```

## Scheduling (tag)
`UwbScheduler` ranges to each anchor in `ANCHORS[]` in turn, one exchange at a
time, then emits one host packet. Sweep rate ≈ several Hz for ≤6 anchors.

## Host packet (tag → MATLAB)
Versioned ASCII line (`HostLink.h`):
```
RTLS,v1,<t_ms>,<tag_id>,<n>,<id1>,<d1_mm>,<q1>,...,<idN>,<dN_mm>,<qN>[,IMU,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>]\n
```
- `<n>` = number of **valid** anchor measurements that follow.
- `dist` in **mm**, `q` = RX power (dBm).
- The `IMU,…` tail is appended **only** when an IMU sample is present, so the
  format is forward-compatible with the BNO085 addition.

## Reserved for the future (not active)
- Message types `MSG_ANNOUNCE` / `MSG_SLOT_GRANT` and the broadcast address for a
  multi-tag TDMA superframe (each tag gets a time slot keyed by `TAG_ADDR`).
