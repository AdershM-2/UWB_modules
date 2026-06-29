# UWB RTLS — Full Improvement Plan (Rev 3)
*Last updated: 2026-06-29*

> **This file is the live, portable progress tracker** (committed to git, so it is
> readable from any clone on any PC). Per-item status markers below are the source
> of truth. The `.tex`/`.pdf` siblings are formal snapshots — regenerate them from
> this when convenient. See the root `CLAUDE.md` for repo orientation + gotchas.

---

## Context

**Hardware:** 4× anchors (Makerfabs "ESP32 UWB Pro with Display", DW1000, PA/LNA) + 1 tag (ESP32-Wrover + DW1000 module + BNO085 IMU)
**Configuration:** 4 anchors + 1 tag with IMU
**Goal:** Indoor positioning at ≤3 cm XY, ≤5 cm Z, ≤5° RPY
**Current accuracy:** 3–10 cm XY (calibration noise, NLOS, limited height separation of anchors)

**Host pipeline (as of 2026-06-29):** Python is the primary host. MATLAB is shelved — it lags significantly behind the Python pipeline (no IMU, no NLOS, parses v1 only, single-tag only, no ZUPT/NIS/adaptive-R/per-anchor calibration). See `python/` for the active host pipeline.

**Board variants in the fleet:**
- **Anchor boards:** Makerfabs "ESP32 UWB Pro with Display" (CS=21 default in `UwbConfig.h`)
- **Tag board:** ESP32-Wrover + standalone DW1000 module + BNO085 IMU (CS=4, OLED on 32/33, BNO085 on Wire1 25/26 — see `sketches/TagWrover/`)

---

## Python Pipeline (`python/`)

Entry points:
- `python/run_localization.py` — main live localisation (UDP or serial, FusionEKF, ZUPT, live display, JSONL logging)
- `python/run_survey.py` — trigger anchor self-survey, collect SURVEY lines, run MDS, write `anchors.json`
- `python/collect_calibration_data.py` — collect range samples at a known position for Tier-1 bias calibration
- `python/calibration_manager.py` — compute and store per-anchor bias corrections → `calibration.json`
- `python/multipoint_survey.py` — 9-position grid calibration, jointly solves anchor + tag positions

Library (`python/rtls/`):
- `frame_parser.py` — v1/v2/v3 wire format parser (v3: fp_dbm, quality per anchor + gyro in IMU tail)
- `fusion_ekf.py` — 6D EKF `[px, py, vx, vy, bx, by]` with IMU-driven prediction, ZUPT injection, NIS chi-squared gate, adaptive R covariance
- `zupt_detector.py` — multi-threshold stationarity detection (accel variance + gyro Z + speed gate + debounce, sliding window 8 packets)
- `multilaterator.py` — Levenberg-Marquardt multilateration, any N≥dim+1
- `position_ekf.py` — simple CV-EKF (no IMU, fallback / reference)
- `anchor_config.py` — loads `matlab/config/anchors.json`
- `fusion_config.py` — FusionEKF tuning parameters
- `system_health.py` — health monitoring / diagnostics
- `imu_integrator.py` — IMU integration helpers
- `analysis/` — post-hoc log analysis scripts and figures

**NLOS scoring (FusionEKF):**
`nlos_score = max(0, (rx_dbm − fp_dbm) / 6)` — DW1000 first-path vs total power diagnostic.
`nlos_factor = 1 + mean(nlos_scores)` inflates the measurement noise covariance R per sweep.

**ZUPT:** injects zero-velocity pseudo-measurement when `accel_var < 0.005`, `accel_mean < 0.10 m/s²`, `|gyro_z| < 0.087 rad/s`, `speed < 0.05 m/s` for ≥3 consecutive packets.

**NIS gate:** chi-squared gate at 95th percentile — rejects measurements with `NIS > chi2.ppf(0.95, df=m)`.

---

## Existing Library Comparison

| Capability | thotro | Makerfabs | jremington | pizzo00 | UwbRtls (ours) |
|---|---|---|---|---|---|
| Anchor ceiling | 4 (frame limit) | 4 | 7* | 6 | **Unlimited** |
| RF collisions | Yes | Yes | Yes | Yes | **No (addressed-only)** |
| Update rate | ~0.6 Hz | ~0.6 Hz | ~0.6 Hz | ~1 Hz | **~10 Hz** |
| Multi-tag | No | No | No | Partial | **Designed-in** |
| NLOS detection | No | No | No | No | **Done (v3 wire format)** |
| IMU fusion | No | No | No | No | **Done (Python FusionEKF)** |
| 3D + RPY | No | No | Fragile | No | **Partial (RPY done, 3D Z pending)** |
| Anchor self-survey | No | No | No | No | **Done** |

---

## Implementation Status

### Done ✓ (all in working tree)

**Phase 0 — Python + MATLAB robustness:**
- Python FusionEKF with IMU-driven prediction, ZUPT, NIS chi-squared gate, adaptive R
- MATLAB watchdog + UDP auto-reconnect, EKF divergence guard, age-gated eviction, bias slider, tuning panel (*MATLAB now shelved*)

**Phase 1 — Foundation:**
- 1.0: `UwbConfig.h` → 64 MHz PRF accuracy mode, reply delay 7000 → 6000 µs. *HW: reflash+recal pending.*
- 1.1: `AntennaCalibration.ino` → 200 samples / 14 iters, uint16_t-safe. *HW: run+validate pending.*
- 1.2: `UwbScheduler` per-anchor `_failStreak`/`_skipSweeps`/`_backoffMult`; 3 fails → skip 5 sweeps, doubling (5→10→20→40).
- 1.3A: `UWB_REPLY_DELAY_US` 6000 → 5000 µs. *HW: reflash+confirm.*
- 1.3B: 6.8 Mbps fast mode — **CONFIRMED FAILED** (2026-06-17, all 4 boards produced garbage frames, reverted). Root cause unknown. No longer pending.
- 1.5: Anchor self-survey via `MSG_SURVEY_REQ/RESP`; `python/run_survey.py` collects, runs MDS, writes `anchors.json`.

**Phase 2 — Ranging Quality (firmware side):**
- 2.1 firmware: `TwrEngine` reads `getFirstPathPower()` + `getReceiveQuality()` after each rangeTo(). `UwbScheduler` carries `fpPower` + `quality` in `RangeResult`. `HostLink` outputs v3 wire format (`fp_dbm`, `quality` per anchor). *HW: reflash all boards to get fp_dbm in packets.*
- 2.1 Python: FusionEKF uses fp_dbm for adaptive R (nlos_factor). `frame_parser.py` parses v3.
- 2.2 Python: Weighted measurement noise via `nlos_factor × R_base`.

**Phase 4 — IMU + Orientation:**
- 4.1: 4 anchors in `ANCHORS[]` for both Tag.ino and TagWrover.ino. `anchors.json` has 4 anchors.
- 4.2: `TagWrover.ino` — full BNO085 implementation via Adafruit BNO08x on Wire1 (GPIO 25/26, I2C 0x4A). Reports: `SH2_ROTATION_VECTOR` + `SH2_LINEAR_ACCELERATION` + `SH2_GYROSCOPE_CALIBRATED` at 100 Hz. IMU tail in v3 wire format. Reset detection + re-enable. WiFi RF glitch workaround (re-init Wire1 after WiFi connect).
- 4.3: RPY from quaternion in `TagWrover.ino` (full `quatToRPY()`). OLED shows R/P/Y/accel. Serial prints every 200 ms.
- 4.4: `python/rtls/fusion_ekf.py` — FusionEKF `[px, py, vx, vy, bx, by]`. Dual-mode prediction: IMU-driven (ax, ay world-frame) or CV fallback. ZUPT injection. NIS gate. Adaptive R.

**Calibration (Python — new items):**
- Tier-1 per-anchor bias: `calibration_manager.py` computes mean bias at circumcenter, saves `calibration.json` v2 (per-anchor `bias_mm`, `bias_std_mm`, `n_samples`). Current data: A1=+445mm, A2=+708mm, A3=+415mm, A4=+378mm. Large values are primarily height-offset error (anchors at ~2m, tag at floor, 2D model treats z=0 for all).
- Multipoint survey: `python/multipoint_survey.py` — 9-position grid, jointly solves anchor + tag positions via `scipy.optimize.least_squares`.

**System fixes (2026-06-29):**
- **DW1000 receiver-wedge bug fix:** `TwrEngine::begin()` now registers `onReceiveFailed` and `onReceiveTimeout` no-op handlers. The DW1000 driver only executes its clearReceiveStatus() → newReceive() → startReceive() re-arm sequence when a handler is registered — without them, any CRC/LDE error leaves the receiver permanently wedged. **This was the root cause of 4-anchor disconnections.** *HW: reflash all boards.*
- WiFi reconnect in `HostLink`: lazy 10-second back-off reconnect on `checkWifi()`, UDP drop counter.
- Stray POLL_ACK warning in `TwrEngine::rangeTo()` (detects duplicate anchor IDs).
- `SensorImu.h` ImuSample: added `status` (BNO085 accuracy 0–3) and `gx/gy/gz` (calibrated gyroscope) fields.
- TagWrover + AnchorWrover sketches: `sketches/TagWrover/` and `sketches/AnchorWrover/`.

---

## Phase 1 — Foundation Fixes

### 1.0 Switch to 64 MHz PRF accuracy mode  — ✓ CODE DONE (2026-06-10)
> **HW pending:** reflash all boards, then recalibrate (1.1) — delay values differ between PRF modes.

### 1.1 Better calibration  — ✓ CODE DONE (2026-06-10)
> **HW pending:** run calibration, validate at two distances (agree within ±2 cm).

### 1.1a Multi-distance calibration mode (AntennaCalibration extension)
**What:** Run binary search at 3 known distances (e.g. 2 m, 4 m, 7 m); sketch prompts between positions; final delay = average of three converged values.
**Why:** Washes out location-specific multipath at the calibration spot.
**Complexity:** Easy

### 1.1b Cross-calibration (Python or MATLAB: all anchor boards at once)
**What:** Place all 4 anchors at a measured test fixture. Use survey data. Solve: `measured(A,B) = true(A,B) + bias_A + bias_B`. Pin board 0x01 as reference (bias=0), least-squares solve 3 unknowns from 6 pairs. Convert metres → ticks (1 tick ≈ 4.69 mm).
**Note:** The large per-anchor biases (~400–700 mm) seen in Tier-1 calibration are dominated by height offset (anchors at 2 m, tag at floor, 2D model). Do this after fixing the height problem (either physically or by moving to 3D mode).
**Complexity:** Easy (Python only, no reflash)

### 1.2 Adaptive polling — ✓ CODE DONE (2026-06-11)

### 1.3A Faster sweep rate — ✓ CODE DONE (2026-06-11)
`UWB_REPLY_DELAY_US` 5000 µs. **HW pending:** reflash + confirm no increase in failures.

### 1.3B 6.8 Mbps fast mode — **CONFIRMED FAILED (2026-06-17)**
All 4 boards produced garbage frames with `MODE_SHORTDATA_FAST_ACCURACY`. Reverted to 110 kbps. Root cause unknown. Not to be retried without diagnosis.

### 1.5 Anchor self-survey — ✓ CODE DONE (2026-06-11); Python host done

---

## Phase 2 — Ranging Quality

### 2.1 FP_POWER NLOS detection — ✓ FIRMWARE DONE (2026-06-29); ✓ PYTHON DONE
- Firmware: `TwrEngine` reads `getFirstPathPower()`/`getReceiveQuality()` after each rangeTo(). v3 wire format: `fp_dbm` + `quality` per anchor. **HW: reflash all boards to activate.**
- Python: `fusion_ekf.py` uses `nlos_factor = 1 + mean(max(0, (rx_dbm−fp_dbm)/6))` to inflate R.

### 2.2 Weighted multilateration — ✓ PYTHON DONE (implicit via adaptive R in FusionEKF)
True weighted LM (direct range residual weighting in multilaterator) is still pending for non-EKF paths.

### 2.3 NLOS range bias correction
**What:** `d_corrected = d_measured − k_nlos × max(0, nlos_score − 3.0)`. Tunable slider.
**Removes:** Systematic positive through-wall ranging bias (~20–50 cm per wall).
**Complexity:** Easy (once 2.1 active on hardware)

### 2.4 Per-anchor diagnostics panel
**What:** Per-anchor range residual bar chart + NLOS score colour coding updated each sweep.
**Complexity:** Easy (Python only)

### 2.5 GPS-style δ-solve (4+ anchors)
**What:** Add common range bias δ to multilateration: `ρᵢ = ‖pos − aᵢ‖ + δ`. Absorbs tag delay miscalibration + average multipath bias.
**Warning:** Only enable with 4+ anchors (degenerate with 3).
**Complexity:** Medium

---

## Phase 3 — State Estimation
*Deprioritised — Python FusionEKF (Phase 4.4) already covers the motion-model gap better than a pure-UWB CA/EKF. Revisit if IMU-fused accuracy is still insufficient.*

### 3.1 Constant-acceleration EKF (CV → CA model) — deferred
### 3.2 δ as EKF state — deferred
### 3.3 MHE (sliding window, Huber loss, room constraints) — future
**What:** Constrained optimisation over N past sweeps. Huber loss on ranges. Room bounds + speed limit as hard constraints.
**Complexity:** Hard

---

## Phase 4 — 3D + Orientation

### 4.1 4th anchor — ✓ DONE (config only)
All 4 anchors in ANCHORS[]. Keep `"dim": 2` for 2D rover; switch to `"dim": 3` when anchors are at different heights.

### 4.2 BNO085 IMU — ✓ CODE DONE (TagWrover.ino, 2026-06-18)
Wire1 (GPIO 25/26), I2C 0x4A, Adafruit BNO08x library. Reports at 100 Hz. WiFi reinit workaround included.
**HW: verified working on Wrover board.**

### 4.3 RPY from quaternion — ✓ CODE DONE (TagWrover.ino)
Full `quatToRPY()`, OLED display, serial 200 ms. Pitch/roll range correction: `d_horiz ≈ d_meas × cos(pitch) × cos(roll)` — pending integration into Python host.

### 4.4 Loose coupling — IMU-aided EKF — ✓ PYTHON DONE (FusionEKF)
6D state `[px, py, vx, vy, bx, by]` with accelerometer bias. IMU-driven prediction at packet rate. ZUPT injection. NIS gate. Adaptive R. **Pitch/roll range correction from 4.3 not yet wired into FusionEKF.**

### 4.5 Tight coupling — raw ranges + IMU in MHE
**State:** `[x, y, z, vx, vy, vz, qw, qx, qy, qz, δ]`. IMU-driven dynamics + raw UWB ranges as measurements. Room bounds + unit quaternion constraint.
**Complexity:** Very Hard

---

## Phase 5 — System Hardening

### 5.1 Deep sleep on anchors
DW1000 deep sleep between exchanges: ~100 mA → ~20 mA average.
**Complexity:** Medium

### 5.2 DW1000 driver error handling — partial (watchdog done; CRC re-arm done 2026-06-29)
40+ `// TODO` in vendored driver. CRC/LDE re-arm now works (onReceiveFailed handler). Full recovery for SPI lockup, PLL failure, LDE timeout still needed.
**Complexity:** Medium

### 5.3 Temperature compensation
DW1000 on-chip temperature sensor already read but never applied. Drift ~0.1 ticks/°C → ~0.5 mm/°C per board.
**Complexity:** Medium

### 5.4 Online self-calibration
Batch optimisation over accumulated tag trajectory + range data. Requires spatial diversity.
**Complexity:** Hard

---

## Phase 6 — Scale and Future

### 6.1 Multi-tag TDMA superframe — Python already multi-tag capable
Python `run_localization.py` uses `dict[int, TagState]` auto-discovery (independent EKF/ZUPT/IMU per tag). Firmware side: ANNOUNCE/SLOT_GRANT frames defined in UwbFrame.h but not yet implemented.
**Complexity:** Hard (firmware side)

### 6.2 Differential timestamp encoding (pizzo00)
12-byte/device vs 17-byte encoding → 6 anchors per RANGE frame. For future fast multi-anchor mode.
**Complexity:** Hard

### 6.3 On-device 2D solve (OLED display)
Precompute (AᵀA)⁻¹ at startup. Display position on OLED without host.
**Complexity:** Medium

---

## Full Item Index

```
Phase 0 — Already Done
  ✓  Python FusionEKF (IMU-driven, ZUPT, NIS, adaptive R)
  ✓  Python calibration infrastructure (Tier-1 + multipoint survey)
  ✓  Python multi-tag auto-discovery (dict[tagId] → independent EKF per tag)
  ✓  MATLAB robustness (watchdog, divergence guard, etc.) [MATLAB now SHELVED]

Phase 1 — Foundation
  1.0   64 MHz PRF accuracy mode                                Easy     ✓ code done (HW: reflash+recal)
  1.1   Better calibration (200 samples, 14 iters)              Easy     ✓ code done (HW: run+validate)
  1.1a  Multi-distance calibration (3 positions, averaged)      Easy     pending
  1.1b  Cross-calibration Python (all boards at once)           Easy     pending (height offset dominates — fix 3D first)
  1.2   Adaptive polling — skip dead anchors                    Easy     ✓ code done
  1.3A  Faster sweep rate (5000 µs reply delay)                 Easy     ✓ code done (HW: reflash+validate)
  1.3B  6.8 Mbps fast mode                                      -        CONFIRMED FAILED — do not retry without diagnosis
  1.5   Anchor self-survey + auto anchors.json                  Medium   ✓ code done (Python host done)

Phase 2 — Ranging Quality
  2.1  FP_POWER NLOS detection (firmware v3 + Python adaptive R) Medium  ✓ code done (HW: reflash to activate fp_dbm)
  2.2  Weighted multilateration                                  Medium  ✓ Python done (implicit via adaptive R)
  2.3  NLOS range bias correction slider                         Easy    pending
  2.4  Per-anchor diagnostics panel                              Easy    pending (Python)
  2.5  GPS-style δ-solve (4+ anchors)                           Medium  pending

Phase 3 — State Estimation [DEPRIORITISED]
  3.1  CA-EKF (CV → CA model)                                   Medium  deferred
  3.2  δ as EKF state                                           Medium-Hard  deferred
  3.3  MHE (sliding window, Huber, room constraints)             Hard    future
  3.4  Hybrid EKF + MHE                                         Medium  future

Phase 4 — 3D + RPY
  4.1  4th anchor + anchors.json update                          Easy    ✓ done
  4.2  BNO085 SensorImu (TagWrover, Wire1, 100 Hz)              Medium  ✓ code done (HW: verified)
  4.3  RPY from quaternion (TagWrover OLED + serial)             Easy    ✓ code done
  4.4  Loose coupling — IMU-aided EKF                           Medium  ✓ Python done (FusionEKF)
       Remaining: wire pitch/roll correction from 4.3 into FusionEKF range pre-processing
  4.5  Tight coupling — raw ranges + IMU in MHE                 Very Hard  future

Phase 5 — System Hardening
  5.1  Deep sleep on anchors                                    Medium  pending
  5.2  DW1000 driver error handling (CRC re-arm done)           Medium  partial
  5.3  Temperature compensation                                  Medium  pending
  5.4  Online self-calibration                                   Hard    future

Phase 6 — Scale
  6.1  Multi-tag TDMA superframe (Python done; firmware pending) Hard    firmware pending
  6.2  Differential timestamp encoding                           Hard    future
  6.3  On-device 2D solve (OLED)                                Medium  future
```

---

## Expected Accuracy at Each Phase

| State | XY accuracy | Z accuracy | RPY | Notes |
|---|---|---|---|---|
| Current (pre-reflash) | 3–10 cm | — | ✓ tag only | Calibration noise + CRC-wedge bug active |
| After reflash (Phase 1+2.1) | 2–6 cm | — | ✓ | fp_dbm active, wedge bug fixed |
| After calibration (1.1a+1.1b) | 1–4 cm | — | ✓ | Delay biases corrected |
| After NLOS weighting (2.2+2.3) | 1–3 cm | — | ✓ | NLOS ranges down-weighted |
| 3D anchors (varied heights) | 1–3 cm | 2–5 cm | ✓ | Z coordinate unlocked |

---

## Immediate Next Actions

```
Priority 1 — reflash (all boards, same firmware):
  reflash Anchor × 4 with: 1.0 (64 MHz PRF) + 1.3A (5000 µs) + 5.2 (CRC re-arm fix)
  reflash TagWrover with:   same + v3 wire format (fp_dbm active)
  then: run AntennaCalibration on each board (1.1 — 200 samples, 14 iters)

Priority 2 — validate ranging:
  run_localization.py → confirm all 4 anchors stable
  check Serial for [TWR]/[SCHED]/[WIFI] prints — should see no wedge resets
  verify v3 packets in Python log (fp_dbm values appear)

Priority 3 — calibration:
  1.1a: multi-distance calibration per board (3 distances)
  collect_calibration_data.py at known position → update calibration.json

Priority 4 — next code items:
  2.3: NLOS range bias correction slider (Easy)
  2.4: per-anchor diagnostics panel (Easy)
  4.4: wire pitch/roll tilt correction into FusionEKF range pre-processing (Medium)
  1.3B: diagnose garbage-frame root cause (Medium — scope session?)
```
