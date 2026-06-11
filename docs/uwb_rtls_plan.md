# UWB RTLS — Full Improvement Plan (Rev 2)
*Last updated: 2026-06-10*

> **This file is the live, portable progress tracker** (committed to git, so it is
> readable from any clone on any PC). Per-item status markers below are the source
> of truth. The `.tex`/`.pdf` siblings are formal snapshots — regenerate them from
> this when convenient. See the root `CLAUDE.md` for repo orientation + gotchas.

---

## Context

**Hardware:** 4× Makerfabs "ESP32 UWB Pro with Display" (DW1000 chip, PA/LNA amplifier, ESP32, SSD1306 OLED)  
**Configuration:** 3 anchors + 1 tag (extendable)  
**Goal:** Low-cost microcontroller-level indoor positioning system delivering XYZ + RPY without camera-based systems  
**Current accuracy:** 3–10 cm (dominated by calibration noise, no NLOS detection, constant-velocity EKF, no IMU)

---

## Existing Library Comparison

| Capability | thotro | Makerfabs | jremington | pizzo00 | dw1000-ng | UwbRtls (ours) |
|---|---|---|---|---|---|---|
| Anchor ceiling | 4 (frame limit) | 4 | 7* | 6 | Unlimited | **Unlimited** |
| RF collisions | Yes (broadcast POLL) | Yes | Yes | Yes | No | **No (addressed-only)** |
| Update rate | ~0.6 Hz | ~0.6 Hz | ~0.6 Hz | ~1 Hz | ~2 Hz | **~3 Hz** |
| Multi-tag | No | No | No | Partial | No | **Designed-in** |
| NLOS detection | No | No | No | No | No | **Planned** |
| IMU fusion | No | No | No | No | No | **Planned** |
| 3D + RPY | No | No | Fragile | No | No | **Planned** |
| Anchor self-survey | No | No | No | No | No | **Planned** |
| Session robustness | N/A | N/A | N/A | N/A | N/A | **Done** |

*jremington raises the MAX_DEVICES counter to 7 but the RANGE frame encoding still overflows at 5 anchors.

### Why existing libraries are limited

**thotro/arduino-dw1000 (canonical, abandoned 2019):**
- 4-anchor ceiling: RANGE frame encoding uses 17 bytes/device × 4 = 68 bytes in a 90-byte buffer. Any 5th anchor overflows silently (corrupted ranges, then silence — Issue #81).
- Broadcast POLL causes RF collisions: all anchors reply to POLL_ACK simultaneously with no slot assignment.
- `_networkDevicesNumber` not declared `volatile` — compiler caches stale value, corrupting device list (Issue #179, never fixed in main repo).
- ~0.6 Hz update rate (21-tick cycle × 80 ms default timer delay).
- `_FPPower` and `_RXPower` fields computed and stored but never used for NLOS detection.

**Makerfabs mf_DW1000:** Essentially thotro with ESP32 pin remapping. Law-of-cosines solver hardcoded for exactly 3 anchors.

**jremington fork:** Adds volatile fix and bumps MAX_DEVICES to 7, but frame encoding unchanged — still overflows at 5 anchors. On-device LLS solver with precomputed normal matrix (good technique).

**pizzo00 fork:** Encoding trick: differential timestamps (12 bytes/device vs 17) → 6 anchors per RANGE frame. Best existing solution but still uses broadcast POLL so collisions remain.

**dw1000-ng (archived 2023):** Infrastructure-coordinated (no fixed anchor array), deep sleep support, still single-tag only.

### What to borrow from existing libraries

| Source | Technique | Integrated in |
|---|---|---|
| jremington | Age-gated anchor eviction | Phase 1.2 (done) |
| jremington | Precomputed (AᵀA)⁻¹ for on-device solve | Phase 6.3 |
| pizzo00 | Differential timestamp encoding (12 B vs 17 B/device) | Phase 6.2 |
| dw1000-ng | Deep sleep on anchors between ranging cycles | Phase 5.1 |
| Nobody | FP_POWER / RX_POWER NLOS detection | Phase 2.1 |

---

## Implementation Status

### Already Done ✓
- MATLAB watchdog + UDP auto-reconnect (10 s timeout → recreates udpport)
- EKF divergence guard + auto-reset (jump >2 m or trace(P) >25 triggers reinit)
- Age-gated anchor eviction in MATLAB (AGE_GATE_SEC = 3.0 s)
- RANGE_BIAS_M tuning slider (0–0.10 m global multipath bias subtraction)
- Inner try-catch (bad sweeps skip silently, session survives)
- Status label in tuning panel (packet count, EKF resets, current bias)

---

## Phase 1 — Foundation Fixes

*No algorithm changes. Highest ROI for effort.*

### 1.0 Switch to 64 MHz PRF accuracy mode  — ✓ CODE DONE (2026-06-10)
> **Status:** Code complete in `libraries/UwbRtls/src/UwbConfig.h` — `UWB_RADIO_MODE` → `MODE_LONGDATA_RANGE_ACCURACY`, `UWB_REPLY_DELAY_US` 7000 → 6000 µs (shared header, applies to all boards). **Pending on hardware:** reflash all 4 boards, then re-run calibration — antenna-delay values differ between PRF modes, so existing delays are now stale.

**What:** Change `DW1000.enableMode(MODE_LONGDATA_RANGE_LOWPOWER)` → `MODE_LONGDATA_RANGE_ACCURACY` in `UwbConfig.h`.  
**Why:** 64 MHz PRF produces a sharper correlation peak in the channel impulse response → better first-path detection → less multipath ambiguity. The Pro boards already have a PA/LNA for range; this improves accuracy not range.  
**Note:** Must reflash all 4 boards. Re-calibrate after (step 1.1) — delay values will differ. Reduce reply delay from 7000 → 6000 µs.  
**Removes:** ~1–3 cm systematic ranging error from multipath peak ambiguity.  
**Complexity:** Easy

### 1.1 Better calibration  — ✓ CODE DONE (2026-06-10)
> **Status:** Code complete — `SAMPLES_PER_STEP` 40 → 200, `SEARCH_ITERS` 12 → 14 in **both** `AntennaCalibration.ino` copies (`sketches/` active + `libraries/UwbRtls/examples/` reference); also widened the sample count + loop counter to `uint16_t` so higher counts can't wrap. **Pending on hardware:** run the calibration, validate at two distances (≈1.5 m and ≈5 m, agree within ±2 cm).

**What:** Change `SAMPLES_PER_STEP` 40 → 200, `SEARCH_ITERS` 12 → 14 in `AntennaCalibration.ino`.  
**Why:** 40 samples gives ±1 cm statistical uncertainty on the mean → ±3 delay units systematic error. 200 samples reduces to ±0.3 cm.  
**Validation:** After convergence, run loop() for 60 s and record mean ± std. Validate at two distances (e.g. 1.5 m and 5 m); both should agree within ±2 cm.  
**Removes:** ±1–3 cm systematic calibration bias per board.  
**Complexity:** Easy

### 1.2 Adaptive polling + age-gated anchor eviction

> **Status:** ✓ CODE DONE (2026-06-11) — `UwbScheduler` now tracks per-anchor `_failStreak`/`_skipSweeps`/`_backoffMult`; 3 consecutive fails → skip 5 sweeps, doubling each subsequent backoff (5→10→20→40, capped). Serial prints `[SCHED]` on skip and on recovery. MATLAB side was already done. No hardware reflash required (library change).

**Firmware (remaining):** Add `failCount[N]` per anchor in `UwbScheduler`; after 3 consecutive fails, skip for 5 sweeps then retry with exponential backoff.  
**MATLAB:** ✓ Done — `anchorLastSeen` + `AGE_GATE_SEC = 3.0` clears stale range history automatically.  
**Removes:** 100 ms wasted per dead anchor per sweep; prevents stale ranges poisoning the solver.  
**Complexity:** Easy

### 1.3 Faster sweep rate

> **Status (Option A):** ✓ CODE DONE (2026-06-11) — `UWB_REPLY_DELAY_US` 6000 → 5000 µs in `UwbConfig.h`. Saves 2 ms per exchange / 8 ms per sweep (4 anchors). Estimated sweep rate: 8 Hz → ~10 Hz. Requires reflash of all boards. Validate by checking Serial Monitor for exchange failures; if drops increase, revert to 6000 µs.

**Option A:** ✓ Done — Reduced `UWB_REPLY_DELAY_US` 6000 → 5000 µs (was 7000 µs before Phase 1.0). Minimum safe value for 110 kbps / 64 MHz PRF mode is ~3500 µs; 5000 µs leaves ~1.5 ms margin above ESP32 SPI + DW1000 processing time.

> 🔴 **Option B — PENDING:** Switch to `MODE_SHORTDATA_FAST_LOWPOWER` (6.8 Mbps, 64-symbol preamble, reply delay ~1500 µs). Estimated sweep rate: ~35–45 Hz for 4 anchors. Trades 64 MHz PRF accuracy for speed — increases multipath sensitivity in small rooms. Defer until Phase 2.1 NLOS detection is in place to compensate. Only needed if >15 Hz is required for the application.

**Effect:** Higher sweep rate directly improves every filter layer downstream.  
**Complexity:** Option A: Easy | Option B: Medium (needs mode switch + recalibration)

### 1.4 Multipath bias correction (MATLAB slider) ✓ Done
Global `RANGE_BIAS_M` subtracted from all filtered ranges before multilaterator. Tunable 0–0.10 m via slider. Tune by placing tag at a known point and minimising `info.rms`.

### 1.5 Anchor self-survey (auto-localization)

> **Status:** ✓ CODE DONE (2026-06-11) — No extra sketch or reflash needed. The tag firmware now accepts a `SURVEY\n` serial command; it loops over all anchor pairs and sends `MSG_SURVEY_REQ` to each anchor, which temporarily calls `rangeTo()` to the target anchor and replies with `MSG_SURVEY_RESP`. 100 samples averaged per pair. `matlab/runSurvey.m` collects results, runs classical MDS, fixes the coordinate frame, and writes `anchors.json` in the format expected by `AnchorConfig.fromJson()`. Run `run_localization.m` immediately after — no reflash of any board needed.

**What:** Anchors range to each other; MATLAB computes their layout via MDS; `anchors.json` is written automatically. No tape measure required.  
**Protocol:** Tag sends `MSG_SURVEY_REQ(target=B)` to Anchor A → Anchor A calls `rangeTo(B)` → Anchor A sends `MSG_SURVEY_RESP(src=A, dst=B, dist, rxp)` → Tag outputs `SURVEY,v1,<src>,<dst>,<avg_dist_mm>,<ok>` line.  
**MATLAB:** `matlab/runSurvey.m`. Collects N×(N-1)/2 pairs (100 samples each, ~4 s/pair). Classical MDS. Frame fixed (anchor 1 origin, anchor 2 on +X, anchor 3 in +Y half-plane). Writes `anchors.json`.  
**Accuracy:** ~1–2 cm anchor position accuracy after 100-sample averaging.  
**Limitation:** 3D self-survey requires meaningful height variation across anchors. All-same-height gives degenerate Z axis.  
**Complexity:** Medium (firmware protocol + MATLAB MDS)

---

## Phase 2 — Ranging Quality

*Firmware changes + MATLAB. Attacks measurement errors at their source.*

### 2.1 FP_POWER NLOS detection in firmware ← unique advantage
**What:** Read first-path amplitude registers (FP_AMPL1/2/3, RXPACC) from DW1000 after each receive. Compute NLOS score per Decawave APS006. Send in RANGE_REPORT payload. Parse in MATLAB.  
**Math:**
```
FP_power_dBm = 10·log10((A1² + A2² + A3²) / RXPACC²) − 115.72
NLOS_score = RX_power_dBm − FP_power_dBm
```
`< 3 dB` → LOS. `3–6 dB` → soft NLOS. `> 6 dB` → hard NLOS.  
**Why it matters:** Detects NLOS before the solver runs, based on physics — not just large residuals after the fact. Works even when consistent NLOS bias looks "normal" to MAD gating.  
**Complexity:** Medium

### 2.2 Weighted multilateration
**What:** Derive weights from NLOS score: `σᵢ = σ_base × 10^(nlos_i/20)`, `wᵢ = 1/σᵢ²`. Modify LM in Multilaterator.m to use weighted residuals and Jacobian.  
**Effect:** A 10 dB NLOS anchor gets 1/100 the influence of a clean LOS anchor.  
**Complexity:** Medium

### 2.3 NLOS range bias correction
**What:** Physical bias correction per anchor: `d_corrected = d_measured − k_nlos × max(0, nlos_score − 3.0)`. Tunable slider K_NLOS (0–0.06 m/dB).  
**Removes:** Systematic positive through-wall ranging bias (~20–50 cm per wall).  
**Complexity:** Easy (once 2.1 done)

### 2.4 Per-anchor diagnostics panel
**What:** Second axes in LivePlotter showing per-anchor range residual (bar chart) and NLOS score (colour-coded) updated every sweep.  
**Gives:** Immediate visual feedback on which anchor is causing problems.  
**Complexity:** Easy

### 2.5 GPS-style δ-solve in Multilaterator (4+ anchors required)
**What:** Augment state to [x, y, δ] where δ is a common range bias: `ρᵢ = ‖pos − aᵢ‖ + δ`. Jacobian row i: `[unit_vec_i, 1]`.  
**Effect:** δ absorbs tag delay miscalibration + average multipath bias. Solved δ is a real-time calibration health metric.  
**Warning:** With 3 anchors this is equivalent to TDOA geometry — poorly conditioned near centroid. Only enable with 4+ anchors.  
**Requires:** Phase 4.1  
**Complexity:** Medium

---

## Phase 3 — State Estimation

### 3.1 Constant-acceleration EKF (CV → CA model)
**What:** Upgrade from `[x, y, vx, vy]` to `[x, y, vx, vy, ax, ay]`. Dynamics: `x ← x + vx·dt + ½ax·dt²`, `ax ← ax + w_ax` (random jerk).  
**Effect:** Tracks direction changes and acceleration phases better. Reduces lag on turns.  
**Complexity:** Medium

### 3.2 δ as an EKF state (bias estimation — works with 3 anchors)
**What:** Add δ to EKF state: `[x, y, vx, vy, δ]`. δ evolves as random walk. Tight coupling: raw ranges feed directly into EKF. Measurement: `hᵢ(state) = ‖pos − aᵢ‖ + δ`. Jacobian row i: `[unit_vec_i, 0, 0, 1]`.  
**Advantage over multilaterator δ-solve:** Works with 3 anchors; temporally smoothed; no ill-conditioning.  
**Complexity:** Medium–Hard

### 3.3 Moving Horizon Estimator (MHE)
**What:** Constrained optimisation over sliding window of N past sweeps.

Objective:
```
min  ‖x_{k−N} − x̄_{k−N}‖²_{P⁻¹}         (arrival cost)
   + Σ_t ‖x_t − F·x_{t−1}‖²_{Q⁻¹}        (dynamics)
   + Σ_t Σ_i ρ(‖pos_t − aᵢ‖ − dᵢ_t)     (ranges, Huber loss)

subject to: x_min ≤ pos_t ≤ x_max         (room bounds, hard)
            ‖vel_t‖ ≤ v_max               (speed limit, hard)
```

Huber loss with δ=0.15 m:
```
ρ(e) = e²/2                   if |e| ≤ 0.15
       0.15·(|e| − 0.075)     if |e| > 0.15
```

**Advantages over EKF:** NLOS absorbed by Huber loss; room boundary prevents wild jumps; physically impossible positions excluded.  
**Computation:** fmincon with N=6, dim=4 → 2–5 ms per solve (well within 333 ms sweep period).  
**Complexity:** Hard

### 3.4 Hybrid EKF + MHE
**What:** EKF every sweep (real-time, zero lag). MHE every K=5 sweeps (accuracy). If they diverge >threshold → flag uncertainty.  
**Complexity:** Medium (once 3.3 done)

---

## Phase 4 — 3D + Orientation (XYZ + RPY)

### 4.1 4th anchor + 3D configuration
**What:** Flash board 4 as anchor (ANCHOR_ID=0x04). Mount at different height. Update anchors.json: `"dim": 3`. Add `0x04` to `ANCHORS[]` in Tag.ino.  
**Effect:** Enables Z-coordinate estimation and δ-solve (Phase 2.5).  
**Complexity:** Easy (config only)

### 4.2 BNO085 IMU — firmware implementation
**What:** Implement `SensorImu::begin()` and `SensorImu::read()` using Adafruit BNO08x SPI library.  
**Why SPI:** I2C bus is taken by OLED; BNO085 has known clock-stretching issues on ESP32 I2C.  
**Reports:** ARVR Stabilised Rotation Vector at 10 ms (quaternion, 100 Hz) + Linear Acceleration at 10 ms (gravity-removed, 100 Hz).  
**Note:** HostLink IMU tail already wired. FrameParser.m already parses it.  
**Complexity:** Medium

### 4.3 RPY output from quaternion
```
roll  = atan2(2(qw·qx + qy·qz),  1 − 2(qx² + qy²))
pitch = asin(2(qw·qy − qz·qx))
yaw   = atan2(2(qw·qz + qx·qy),  1 − 2(qy² + qz²))
```
BNO085 magnetometer corrects yaw drift. Log RPY alongside XYZ. Add RPY readout to HUD.  
**Complexity:** Easy (once 4.2 done)

### 4.4 Loose coupling — IMU-aided EKF
**What:** IMU predict at 100 Hz (fills 333 ms gaps between UWB sweeps). UWB measurement update at 3 Hz.  
**State:** `[x, y, z, vx, vy, vz, qw, qx, qy, qz, δ]` (11-state).  
**Effect:** Continuous smooth XYZ output between UWB sweeps. Effectively decouples position accuracy (UWB) from position smoothness (IMU).  
**Complexity:** Hard

### 4.5 Tight coupling — raw ranges + IMU in one filter
**State:** `[x, y, z, vx, vy, vz, qw, qx, qy, qz, δ]`  
**Dynamics:** IMU-driven at 100 Hz (quaternion integration + linear acceleration)  
**Measurement:** Raw UWB ranges (not multilaterated position)  
**Solver:** MHE with Huber loss + room bounds + speed limit + quaternion unit-norm constraint  
**Output:** XYZ + RPY + velocity + bias estimate  
**This is the architecture commercial systems use on dedicated hardware.**  
**Complexity:** Very Hard

---

## Phase 5 — System Hardening

### 5.1 Deep sleep on anchors (from dw1000-ng)
DW1000 deep sleep for 80% of the sweep period between exchanges. Power: ~100 mA continuous → ~20 mA average.  
**Complexity:** Medium

### 5.2 DW1000 driver error handling
40+ `// TODO proper error/warning handling` in the vendored driver. Add recovery for SPI lockup, PLL failure, LDE timeout. Auto-reset if radio goes silent for >500 ms.  
**Complexity:** Medium

### 5.3 Temperature compensation
DW1000 on-chip temperature sensor (OTP-calibrated) already read but never applied. Drift ~0.1 ticks/°C → ~0.5 mm/°C per board. Apply linear correction relative to calibration temperature.  
**Complexity:** Medium

### 5.4 Online self-calibration
Batch optimisation (offline): jointly solve for anchor positions and delays using accumulated tag trajectory + range data. Requires spatial diversity.  
**Complexity:** Hard

---

## Phase 6 — Scale and Future

### 6.1 Multi-tag TDMA superframe
Time-slotted TDMA. Master anchor broadcasts slot assignments via ANNOUNCE frames (already defined in UwbFrame.h). MATLAB maintains separate EKF/MHE per tagId.  
**Complexity:** Hard

### 6.2 Differential timestamp encoding (pizzo00 technique)
Broadcast POLL with 12-byte/device encoding (vs 17-byte absolute timestamps). 6 anchors per frame. For future fast multi-anchor mode.  
**Complexity:** Hard

### 6.3 On-device 2D solve (jremington technique)
Precompute (AᵀA)⁻¹ once at startup (anchors fixed). Position update = matrix-vector multiply. Display on OLED without MATLAB.  
**Complexity:** Medium

---

## Full Item Index

```
Phase 0 — Already Done
  ✓  MATLAB watchdog + UDP auto-reconnect
  ✓  EKF divergence guard + auto-reset
  ✓  Age-gated anchor eviction (MATLAB)
  ✓  RANGE_BIAS_M tuning slider
  ✓  Inner try-catch (bad sweeps skip, session survives)
  ✓  Status label in tuning panel

Phase 1 — Foundation
  1.0  Switch to 64 MHz PRF (MODE_LONGDATA_RANGE_ACCURACY)    Easy   ✓ code done (HW: reflash+recal)
  1.1  Better calibration (200 samples, multi-distance)        Easy   ✓ code done (HW: run+validate)
  1.2  Adaptive polling — skip dead anchors (firmware)         Easy   ✓ code done
  1.3  Faster sweep rate — Option A done (5000 µs); 🔴 Option B (6.8 Mbps) pending
  1.5  Anchor self-survey + auto anchors.json                  Medium ✓ code done

Phase 2 — Ranging Quality
  2.1  FP_POWER NLOS detection (firmware + wire format)        Medium
  2.2  Weighted multilateration (NLOS score → weights)         Medium
  2.3  NLOS range bias correction slider                       Easy
  2.4  Per-anchor diagnostics panel in LivePlotter             Easy
  2.5  GPS-style δ-solve in Multilaterator (4+ anchors)       Medium

Phase 3 — State Estimation
  3.1  Constant-acceleration EKF (CV → CA model)              Medium
  3.2  δ as EKF state (bias estimation, 3 anchors OK)         Medium-Hard
  3.3  MHE (sliding window, Huber loss, room constraints)      Hard
  3.4  Hybrid EKF + MHE                                       Medium

Phase 4 — 3D + RPY
  4.1  4th anchor + 3D anchors.json                           Easy
  4.2  BNO085 SensorImu implementation (SPI)                  Medium
  4.3  RPY output from quaternion                             Easy
  4.4  Loose coupling — IMU-aided EKF (100 Hz predict)        Hard
  4.5  Tight coupling — raw ranges + IMU in MHE               Very Hard

Phase 5 — System Hardening
  5.1  Deep sleep on anchors                                  Medium
  5.2  DW1000 driver error handling                          Medium
  5.3  Temperature compensation                              Medium
  5.4  Online self-calibration                               Hard

Phase 6 — Scale
  6.1  Multi-tag TDMA superframe                             Hard
  6.2  Differential timestamp encoding                       Hard
  6.3  On-device 2D solve (OLED display)                    Medium
```

---

## Expected Accuracy at Each Phase

| After phase | XY accuracy | Z accuracy | RPY | Notes |
|---|---|---|---|---|
| Current | 3–10 cm | — | — | Calibration noise + NLOS + CV EKF |
| Phase 1 | 2–6 cm | — | — | Calibration bias removed, multipath floor corrected |
| Phase 2 | 1–4 cm | — | — | NLOS weighted/rejected before solver |
| Phase 3 | 1–3 cm | — | — | Better filter model, MHE room constraints |
| Phase 4 | 1–3 cm | 2–5 cm | ~3–5° | IMU fills gaps, 3D anchors |
| Phase 5 | Same | Same | Same | Better reliability, power, robustness |

---

## Recommended Starting Sequence

```
1.0 → 1.1 → 1.2 → 1.5 → 1.3 → 2.1 → 2.2 → 2.4 → 3.1 → 4.1 → 4.2 → 4.3 → 4.4
PRF   cal   poll  surv  rate  NLOS  wgt  plot   CA   4th   IMU   RPY   fuse
```
