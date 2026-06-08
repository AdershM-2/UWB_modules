# 04 — MATLAB host

The host receives ranges, solves position (overdetermined, outlier-gated),
filters with an EKF, plots live, and logs to CSV. Requires R2020b+ (for
`udpport`/`serialport`). No toolboxes required — the solver is hand-rolled
Levenberg–Marquardt.

## Run
```matlab
cd matlab
run_localization
```
Edit the top of `run_localization.m`:
- `TRANSPORT` = `"serial"` or `"udp"` (match how `Tag.ino` was built).
- `SERIAL_PORT` / `SERIAL_BAUD`, or `UDP_PORT` (must match `HOST_PORT` in `Tag.ino`).

## Adding / moving anchors (no firmware change)
Copy `config/anchors.example.json` to `config/anchors.json` and edit:
```json
{
  "dim": 2,
  "bounds": [-0.5, 4.5, -0.5, 4.5, 0.0, 3.0],
  "anchors": [
    { "id": 1, "x": 0.0, "y": 0.0, "z": 1.5 },
    { "id": 2, "x": 4.0, "y": 0.0, "z": 1.5 },
    { "id": 3, "x": 2.0, "y": 4.0, "z": 1.5 },
    { "id": 4, "x": 0.0, "y": 4.0, "z": 1.5 }
  ]
}
```
- `id` must match `ANCHOR_ID` in the anchor’s firmware **and** be listed in the
  tag’s `ANCHORS[]`.
- Add as many as you like. The solver uses every anchor that reports.

## 2D → 3D
Set `"dim": 3`, give anchors **varied heights** (`z`), and ensure ≥4 well-spread
anchors. The EKF and solver extend automatically; **firmware is unchanged**
(it only ships ranges).

## Pipeline components (`+rtls`)
- `UwbReceiver` — opens `udpport`/`serialport`, returns one parsed sweep per call.
- `FrameParser` — parses the `RTLS,v1,...` line (incl. optional `IMU,…` tail).
- `AnchorConfig` — id→coordinate map + room bounds + `dim`.
- `Multilaterator` — LM solve of `min Σ(‖x−aᵢ‖−dᵢ)²` with robust residual gating.
- `PositionEKF` — constant-velocity filter; `update()` is pluggable so it can
  later take raw ranges (tight coupling) and an IMU prediction step.
- `LivePlotter` — anchors, moving tag, trail, 2-σ covariance ellipse.

## Tuning
- `Multilaterator.gateK` (default 3.0): lower = more aggressive NLOS rejection.
- `Multilaterator.rangeSigma` (0.10 m): your post-calibration range noise.
- `PositionEKF.qAccel`: raise if tracking lags fast motion, lower for smoother.

## Future: BNO085 fusion
When the IMU is added, packets carry an `IMU,…` tail (already parsed into
`s.imu`). The EKF gains an IMU-driven predict step (orientation + linear accel)
between UWB updates — drift correction and a higher output rate.
