# UWB RTLS — Project Guide

Indoor real-time location system. 4× Makerfabs "ESP32 UWB Pro with Display" (Decawave
DW1000). Goal: XYZ + RPY indoor positioning at microcontroller level, no cameras.
Current ~3–10 cm 2D; target ≤3 cm XY, ≤5 cm Z, ≤5° RPY. Improvement work is organized
as phases — see **Progress tracking** below.

> **Cross-PC note:** This `CLAUDE.md` and the docs it points to are committed to git, so
> they are readable from any clone on any machine. Claude's own per-machine memory under
> `~/.claude/...` is **path-keyed and NOT in the repo** (`.claude/` is gitignored) — it does
> not travel between PCs. Treat the committed files here as the single source of truth.

## Architecture (the one rule that explains everything)
The **tag streams raw ranges**; **all position solving happens host-side in MATLAB**. The
firmware never computes position (until the optional on-device solve, Phase 6.3). So
adding/moving anchors or changing the estimator is usually a host-side change only — no
reflash. Wire format (`HostLink.h`):
`RTLS,v1,<t_ms>,<tag_id>,<n>,<id,d_mm,q>...[,IMU,<qw,qx,qy,qz,ax,ay,az>]` where `q` = RX
power in dBm. The `IMU,...` tail is appended only when an IMU sample is present.

## Repo layout
- `libraries/UwbRtls/src/` — **shared firmware library, single-source.** Edit once, applies
  to every board. `UwbConfig.h` (radio mode, reply delay, pins, addressing), `TwrEngine`
  (double-sided TWR), `UwbScheduler` (round-robin TDMA over anchor list — this is what
  removes the 4-anchor ceiling), `HostLink.h` (wire format), `UwbFrame` (payload pack/unpack),
  `SensorImu.h` (BNO085 stub, wired up in Phase 4.2), `OledStatus`.
- `libraries/UwbRtls/src/dw1000/` — **vendored Decawave/thotro driver (third-party).** Avoid
  gratuitous edits; touch only for Phase 5.2 error-handling work.
- `sketches/{Anchor,Tag,AntennaCalibration}/` — **the ACTIVE flash targets** (`.vscode/arduino.json`
  points here). OLED enabled to match the "with Display" boards.
- `libraries/UwbRtls/examples/{Anchor,Tag,AntennaCalibration}/` — the library's bundled
  reference copies. Intentionally diverged from `sketches/` (no OLED, different demo constants).
- `matlab/+rtls/` — host pipeline: `UwbReceiver`, `FrameParser`, `AnchorConfig`,
  `Multilaterator` (LM solve, any N≥dim+1), `PositionEKF` (CV, dim-parameterized), `LivePlotter`.
  Entry point: `matlab/run_localization.m` (UDP/serial, live tuning panel, EKF, CSV logging).
- `matlab/config/anchors.json` — **the one place anchor coordinates live** (host-side). Supports
  `dim: 2` or `3`; the whole MATLAB pipeline already handles 3D.
- `docs/` — plan (`uwb_rtls_plan.{md,tex,pdf}`), `uwb_rtls_design.*`, `uwb_rtls_for_dummies.*`,
  and setup/protocol/calibration/matlab guides.

## Gotchas (learned the hard way)
1. **Dual sketch copies.** Every sketch exists in BOTH `sketches/` (active) and
   `libraries/UwbRtls/examples/` (reference). For any sketch-level change, edit BOTH. Shared
   headers under `libraries/UwbRtls/src/` are single-source — edit once.
2. **Machine-specific values are baked into source and differ per PC** — don't assume they're
   canonical: WiFi SSID/pass + host IP in `sketches/Tag/Tag.ino`, serial `COM` ports in
   `matlab/run_localization.m` and `.vscode/arduino.json`. Adjust per machine; don't "fix" them.
3. **Per-board antenna delay** is per-device and lives in each board's sketch (`ANTENNA_DELAY`).
   It changes when the PRF mode changes — recalibrate after any `UwbConfig.h` radio change.

## Progress tracking
**`docs/uwb_rtls_plan.md` is the live, portable progress tracker** — per-item status markers
there are the source of truth (the `.tex`/`.pdf` are formal snapshots; regenerate from the `.md`).

Done so far (code in working tree):
- **Phase 0** — MATLAB robustness (watchdog/UDP reconnect, EKF divergence guard, age-gated
  eviction, bias slider, tuning panel). See plan.
- **Phase 1.0** — `UwbConfig.h`: 64 MHz PRF accuracy mode + reply delay 7000→6000 µs.
  *Hardware follow-up pending: reflash all 4 boards + recalibrate.*
- **Phase 1.1** — both `AntennaCalibration.ino` copies: 200 samples / 14 iters, uint16_t-safe.
  *Hardware follow-up pending: run calibration + validate at two distances.*
- **Phase 1.2** — `UwbScheduler`: per-anchor `_failStreak`/`_skipSweeps`/`_backoffMult`; 3 fails → skip 5 sweeps, doubling each time (5→10→20→40). Serial `[SCHED]` prints on skip/recovery. Library change only — no reflash needed.
- **Phase 1.3 Option A** — `UwbConfig.h`: `UWB_REPLY_DELAY_US` 6000 → 5000 µs. ~8 Hz → ~10 Hz sweep rate. *Hardware follow-up: reflash all boards, confirm no increase in exchange failures.*
  *Option B (6.8 Mbps fast mode) — PENDING: deferred until Phase 2.1 NLOS detection is in place.*

Recommended next sequence: **1.5 → 2.1 → 2.2 → 2.4 → 3.1 → 4.1 → 4.2 → 4.3 → 4.4.**
