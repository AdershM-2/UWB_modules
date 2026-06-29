# UWB RTLS — Project Guide

Indoor real-time location system. 4× anchor boards + 1 tag with BNO085 IMU (Decawave DW1000).
Goal: XYZ + RPY indoor positioning at microcontroller level, no cameras.
Current ~3–10 cm 2D; target ≤3 cm XY, ≤5 cm Z, ≤5° RPY. Improvement work is organized
as phases — see **Progress tracking** below.

> **Cross-PC note:** This `CLAUDE.md` and the docs it points to are committed to git, so
> they are readable from any clone on any machine. Claude's own per-machine memory under
> `~/.claude/...` is **path-keyed and NOT in the repo** (`.claude/` is gitignored) — it does
> not travel between PCs. Treat the committed files here as the single source of truth.

## Architecture (the one rule that explains everything)
The **tag streams raw ranges**; **all position solving happens host-side in Python**. The
firmware never computes position (until the optional on-device solve, Phase 6.3). So
adding/moving anchors or changing the estimator is usually a host-side change only — no
reflash. Wire format (`HostLink.h`):
`RTLS,v3,<t_ms>,<tag_id>,<n>,<id,d_mm,rx_dbm,fp_dbm,q>...[,IMU,<status,qw,qx,qy,qz,ax,ay,az,gx,gy,gz>]`
where `fp_dbm` = first-path power (NLOS scoring) and `q` = receive quality. The `IMU,...` tail is
appended only when an IMU sample is present. v1/v2 accepted by Python parser for backward compat.

**Host pipeline: Python is primary. MATLAB is shelved.** Entry point: `python/run_localization.py`.

## Board variants
- **Anchor boards:** Makerfabs "ESP32 UWB Pro with Display" — CS=21 (default `UWB_PIN_SS`). Flash: `sketches/Anchor/`.
- **Tag board:** ESP32-Wrover + standalone DW1000 module + BNO085 IMU — CS=4, OLED on GPIO 32/33, BNO085 on Wire1 GPIO 25/26. Flash: `sketches/TagWrover/`.
- **Alternative tag (Pro board):** Flash `sketches/Tag/` (no IMU, CS=21). Produces v1-compatible packets.

## Repo layout
- `libraries/UwbRtls/src/` — **shared firmware library, single-source.** Edit once, applies
  to every board. `UwbConfig.h` (radio mode, reply delay, pins, addressing), `TwrEngine`
  (double-sided TWR + fpPower/quality reads + CRC-wedge fix), `UwbScheduler` (round-robin TDMA,
  carries fpPower/quality per result), `HostLink.h` (v3 wire format), `UwbFrame` (payload pack/unpack),
  `SensorImu.h` (ImuSample struct with status + gyro), `OledStatus`.
- `libraries/UwbRtls/src/dw1000/` — **vendored Decawave/thotro driver (third-party).** Avoid
  gratuitous edits; touch only for Phase 5.2 error-handling work.
- `sketches/{Anchor,Tag,TagWrover,AnchorWrover,AntennaCalibration}/` — **the ACTIVE flash targets**.
  `TagWrover` = Wrover board with BNO085 IMU. `AnchorWrover` = non-Pro anchor variant. OLED enabled.
- `libraries/UwbRtls/examples/{Anchor,Tag,TagWrover,AnchorWrover,AntennaCalibration}/` — library
  bundled reference copies. Intentionally diverged from `sketches/` (no OLED, different demo constants).
- `python/` — **primary host pipeline.** `run_localization.py` (UDP/serial, FusionEKF, ZUPT, NIS,
  adaptive R, live display, JSONL logging). `run_survey.py` (anchor self-survey → `anchors.json`).
  `rtls/` subpackage: `frame_parser` (v1/v2/v3), `fusion_ekf`, `zupt_detector`, `multilaterator`,
  `calibration_manager`, etc. Entry point: `python/run_localization.py`.
- `matlab/+rtls/` — **SHELVED.** Legacy host pipeline (v1 only, no IMU, no NLOS). Do not develop.
- `matlab/config/anchors.json` — **anchor coordinates** (shared by Python + MATLAB). `dim: 2` or `3`.
- `docs/` — plan (`uwb_rtls_plan.{md,tex,pdf}`), design docs, setup guides.

## Gotchas (learned the hard way)
1. **Dual sketch copies.** Every sketch exists in BOTH `sketches/` (active) and
   `libraries/UwbRtls/examples/` (reference). For any sketch-level change, edit BOTH. Shared
   headers under `libraries/UwbRtls/src/` are single-source — edit once.
2. **Machine-specific values are baked into source and differ per PC** — don't assume they're
   canonical: WiFi SSID/pass + host IP in `sketches/Tag/Tag.ino` and `sketches/TagWrover/TagWrover.ino`,
   serial `COM` ports in `python/run_localization.py`. Adjust per machine; don't "fix" them.
3. **Per-board antenna delay** is per-device and lives in each board's sketch (`ANTENNA_DELAY`).
   It changes when the PRF mode changes — recalibrate after any `UwbConfig.h` radio change.
4. **Phase 1.3B confirmed failed.** 6.8 Mbps (`MODE_SHORTDATA_FAST_ACCURACY`) caused garbage frames
   on all 4 boards (2026-06-17). Reverted. Root cause unknown — do not retry without diagnosis.
5. **fpPower/quality read order:** In `TwrEngine::rangeTo()`, `getFirstPathPower()` and `getReceiveQuality()`
   must be called BEFORE `startRx()` — once RX restarts, those registers are cleared.
6. **CRC-wedge fix:** `onReceiveFailed` and `onReceiveTimeout` no-op handlers MUST be registered in
   `TwrEngine::begin()`. Without them, any CRC/LDE error leaves the DW1000 receiver permanently wedged.
   This was the root cause of 4-anchor disconnections. All boards need reflash to activate.

## Progress tracking
**`docs/uwb_rtls_plan.md` is the live, portable progress tracker** — per-item status markers
there are the source of truth (the `.tex`/`.pdf` are formal snapshots; regenerate from the `.md`).

Done so far (code in working tree):
- **Phase 0** — Python FusionEKF (6D state, IMU-driven prediction, ZUPT, NIS, adaptive R). MATLAB robustness (shelved).
- **Phase 1.0** — `UwbConfig.h`: 64 MHz PRF accuracy mode + reply delay 7000→6000 µs. *HW: reflash+recal.*
- **Phase 1.1** — `AntennaCalibration.ino` (both copies): 200 samples / 14 iters, uint16_t-safe. *HW: run+validate.*
- **Phase 1.2** — `UwbScheduler`: per-anchor `_failStreak`/`_skipSweeps`/`_backoffMult`; 3 fails → skip 5 sweeps, doubling (5→10→20→40). `[SCHED]` serial prints on skip/recovery.
- **Phase 1.3A** — `UWB_REPLY_DELAY_US` 6000 → 5000 µs. *HW: reflash+confirm.*
- **Phase 1.5** — Anchor self-survey: `MSG_SURVEY_REQ/RESP` in `UwbFrame`+`TwrEngine`; tag accepts `SURVEY\n`; `python/run_survey.py` runs MDS, writes `anchors.json`.
- **Phase 2.1 firmware** — `TwrEngine` reads `fpPower`/`quality`; `UwbScheduler` carries them in `RangeResult`; `HostLink` outputs v3 (`fp_dbm`, `quality` per anchor). `SensorImu` adds `status`+gyro. *HW: reflash to activate.*
- **Phase 2.1 Python** — `fusion_ekf.py` nlos_factor adaptive R.
- **Phase 4.1** — 4 anchors in `ANCHORS[]`.
- **Phase 4.2** — `TagWrover.ino`: full BNO085 on Wire1 (GPIO 25/26), 100 Hz, WiFi reinit workaround.
- **Phase 4.3** — RPY from quaternion in `TagWrover.ino`, OLED + serial output.
- **Phase 4.4** — Python `FusionEKF`: IMU-driven prediction, ZUPT, NIS, adaptive R. 6D state.
- **Phase 5.2 partial** — `TwrEngine` CRC-wedge fix (`onReceiveFailed`/`onReceiveTimeout` handlers). *HW: reflash.*
- **Python calibration** — `calibration_manager.py` (Tier-1 per-anchor bias), `multipoint_survey.py` (9-position joint solve).

**Recommended immediate action:** reflash all boards with current library (CRC fix + v3 format), then run calibration.
