# 03 — Antenna-delay calibration

Antenna delay is the **single biggest** DW1000 ranging error source. Skipping it
leaves ~30–50 cm bias; doing it gets you to ~10 cm. Calibrate **every** board.

## Idea
The reported distance is monotonic in antenna delay: **more delay → shorter
reported distance.** We binary-search each board’s antenna delay until its
measured distance to a reference matches a known truth.

## Procedure
1. Flash one board as a normal **Anchor** (`examples/Anchor`) with the default
   delay (16384). This is your **reference**; note its `ANCHOR_ID` (e.g. `0x01`).
2. Place the board **under test** and the reference an accurately measured
   line-of-sight distance apart — **7.0 m** is a good choice. Antennas facing
   each other, clear path, away from large metal surfaces.
3. Flash `examples/AntennaCalibration` to the board under test. Set:
   - `TRUE_DISTANCE_M` = your measured distance,
   - `REF_ANCHOR_ID` = the reference board’s ID,
   - (optionally widen `DELAY_LOW`/`DELAY_HIGH`).
4. Open Serial Monitor @ 115200. It prints each step, e.g.:
   ```
   delay=16450  mean=7.030 m  err=+0.030 m
   delay=16500  mean=6.998 m  err=-0.002 m
   ...
   ==> Calibrated ANTENNA_DELAY = 16493
   ```
5. **Paste** that value into this board’s `Anchor.ino` / `Tag.ino`
   (`ANTENNA_DELAY`). Repeat for every board (swap which board is under test).

## Tips
- Tuned values usually land in **16450–16650**.
- Average plenty of samples per step (`SAMPLES_PER_STEP`, default 40) for a
  stable mean.
- Calibrate at a realistic distance/orientation for your deployment.
- After calibrating, verify with a few static distances (1, 3, 5, 8 m) — you
  should see ±10 cm.

## Optional: persist in flash (NVS)
For convenience you can later store the calibrated delay in ESP32 NVS via the
`Preferences` library and load it at boot, so re-flashing firmware doesn’t lose
the per-board value. Not required — a constant in the sketch is fine.
