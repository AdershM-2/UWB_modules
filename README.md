# UWB RTLS — microcontroller-level indoor XYZ + RPY positioning

**Goal:** a self-contained indoor positioning system that produces **X, Y, Z coordinates
and full orientation (roll, pitch, yaw)** using only UWB radios and an IMU — no cameras,
no external infrastructure beyond a few anchors, running on off-the-shelf ESP32 hardware.

**Hardware:** 4× Makerfabs "ESP32 UWB Pro with Display" (Decawave DW1000 chip)
— 3+ boards as fixed anchors, 1 as the mobile tag. BNO085 IMU to be added to the tag.

**Current accuracy:** ~3–10 cm XY, 2D only.
**Target:** ≤ 3 cm XY, ≤ 5 cm Z, ≤ 5° RPY.

---

## How it works

The DW1000 chip measures time-of-flight between two radios at 15.65 ps resolution
(≈ 4.7 mm per tick). The tag performs a **double-sided two-way ranging (DS-TWR)** exchange
with each anchor in sequence, getting a distance measurement to each. Those distances feed
a **weighted multilateration solver** on the MATLAB host, which produces an XY (or XYZ)
position. A **Kalman filter** smooths the trajectory. An IMU on the tag will eventually
fill in orientation and smooth position between radio updates.

The tag firmware is a dumb ranging sensor — it streams `{anchor_id, distance}` tuples
over UDP or USB serial and does nothing else. All algorithm work runs in MATLAB. This means
adding anchors, switching estimators, or adding 3D never requires reflashing the tag.

---

## Repository layout

```
libraries/UwbRtls/        Arduino library — the firmware stack
  src/
    dw1000/               Low-level DW1000 register driver (vendored, Apache-2.0)
    UwbFrame.{h,cpp}      Frame format: type + src + dst + seq + payload
    TwrEngine.{h,cpp}     DS-TWR initiator (tag) and responder (anchor)
    UwbScheduler.{h,cpp}  Round-robin ranging over an anchor address list
    HostLink.{h,cpp}      UDP / serial output with versioned packet format
    SensorImu.{h,cpp}     BNO085 interface stub (wired, not yet implemented)
    OledStatus.{h,cpp}    SSD1306 status display
  examples/
    Anchor/               Flash on each fixed board (set unique ANCHOR_ID)
    Tag/                  Flash on the mobile board
    AntennaCalibration/   Per-board antenna delay tuning

matlab/
  +rtls/
    UwbReceiver.m         UDP / serial reader
    FrameParser.m         Packet → struct {t, tagId, ranges, imu}
    AnchorConfig.m        Anchor geometry (id → [x y z]) — edit this to add anchors
    Multilaterator.m      Levenberg–Marquardt multilateration + outlier gating
    PositionEKF.m         Constant-velocity EKF; pluggable measurement update
    LivePlotter.m         Live anchor + tag + trail + covariance ellipse display
  run_localization.m      Main entry point
  config/anchors.json     Anchor coordinates

docs/
  uwb_rtls_design.pdf     Full mathematical design document
  uwb_rtls_plan.pdf       Implementation roadmap (all phases, with library analysis)
  uwb_rtls_for_dummies.pdf  Plain-English companion to the math
```

---

## Quick start

1. **Install** — copy `libraries/UwbRtls` into your Arduino `libraries/` folder.
2. **Calibrate** — run `examples/AntennaCalibration` on each board to find its
   antenna delay. Record the result in `UwbConfig.h`.
3. **Flash anchors** — open `examples/Anchor`, set a unique `ANCHOR_ID` (0x01, 0x02, …),
   flash to each fixed board.
4. **Flash tag** — open `examples/Tag`, flash to the mobile board.
5. **Set geometry** — measure anchor positions (or run the planned self-survey) and
   fill in `matlab/config/anchors.json`.
6. **Run** — open MATLAB, run `matlab/run_localization.m`. The live plot appears
   and raw data is logged to a timestamped CSV.

---

## Implementation roadmap

The path from "ranging works" to "full 6-DOF at ≤ 3 cm" in six phases.
See `docs/uwb_rtls_plan.pdf` for the full technical breakdown.

### Phase 0 — Robust foundation ✓ Done
- MATLAB watchdog + UDP auto-reconnect
- EKF divergence guard and auto-reset
- Live 5-slider tuning panel (process noise, measurement noise, range bias)
- Age-gated anchor eviction; inner try-catch so bad packets never crash the session

### Phase 1 — Calibration and configuration
- Switch to 64 MHz PRF (`MODE_LONGDATA_RANGE_ACCURACY`) for sharper first-path detection
- Increase calibration sample count from 40 → 200 per step; validate at multiple distances
- Adaptive polling: skip non-responding anchors with exponential backoff
- Faster sweep rate: reduce reply delay or switch to 6.8 Mbps mode (target 8–10 Hz)
- **Anchor self-survey** — anchors range to each other; MATLAB computes layout via MDS
  and writes `anchors.json` automatically (no tape measure)

*Expected: 2–6 cm XY*

### Phase 2 — NLOS detection and weighted solving
- Read DW1000 FP_AMPL1/2/3 + RXPACC registers after each receive and send in the
  range report. Compute per-packet NLOS score: ΔP = P_RX − P_FP (Decawave APS006).
  < 3 dB → LOS, 3–6 dB → soft NLOS, > 6 dB → hard NLOS.
- Weight the multilateration solver by NLOS score (σᵢ ∝ 10^(ΔPᵢ/20))
- Per-anchor range-bias correction as a function of NLOS score
- Live per-anchor diagnostics panel (residual bar chart, colour-coded NLOS score)
- GPS-style common-bias solve: augment state with δ (requires 4+ anchors)

*Expected: 1–4 cm XY*

### Phase 3 — Better state estimation
- Upgrade EKF from constant-velocity to **constant-acceleration** (Singer model Q)
- Add δ (range bias) as an EKF state — works with 3 anchors, temporally smoothed
- **Moving Horizon Estimator (MHE)**: sliding window over N=6 sweeps, Huber loss on
  range residuals, hard room boundary + speed limit constraints via `fmincon`
- Hybrid: EKF every sweep (real-time), MHE every 5 sweeps (accuracy pass)

*Expected: 1–3 cm XY*

### Phase 4 — 3D positioning and full orientation
- Mount 4th anchor at a different height; set `"dim": 3` in anchor config
- Implement BNO085 over SPI (`SensorImu`): ARVR stabilised quaternion at 100 Hz +
  linear acceleration at 100 Hz
- Extract roll, pitch, yaw from quaternion (ZYX convention); log alongside XYZ
- **Loose coupling**: IMU predicts at 100 Hz, UWB corrects at 3 Hz —
  11-state EKF: [p, v, q, δ]
- **Tight coupling** (future): raw UWB ranges + IMU in a single MHE with
  quaternion unit-norm constraint

*Expected: 1–3 cm XY, 2–5 cm Z, 3–5° RPY at 100 Hz*

### Phase 5 — Hardening
- Deep sleep on anchors between ranging cycles (20 mA average vs 100 mA continuous)
- DW1000 driver error recovery (SPI lockup, PLL failure, LDE timeout)
- On-chip temperature compensation (0.5 mm/°C drift correction)
- Online self-calibration via batch optimisation over accumulated trajectory data

### Phase 6 — Scale
- Multi-tag TDMA superframe (time-slotted, master anchor broadcasts slot assignments)
- On-device 2D solve for standalone use (OLED display, no host PC required)

---

## Accuracy summary

| After phase | XY | Z | RPY | Output rate |
|---|---|---|---|---|
| Current | 3–10 cm | — | — | 3 Hz |
| Phase 1 | 2–6 cm | — | — | 4–8 Hz |
| Phase 2 | 1–4 cm | — | — | 4–8 Hz |
| Phase 3 | 1–3 cm | — | — | 4–8 Hz |
| Phase 4 | 1–3 cm | 2–5 cm | 3–5° | 100 Hz |

---

## Attribution

`libraries/UwbRtls/src/dw1000/` is an unmodified copy of the low-level register driver
from [thotro/arduino-dw1000](https://github.com/thotro/arduino-dw1000) (Apache-2.0).
See [libraries/UwbRtls/NOTICE](libraries/UwbRtls/NOTICE).
