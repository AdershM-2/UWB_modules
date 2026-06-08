# UWB RTLS — scalable indoor positioning for Makerfabs ESP32 UWB Pro (DW1000)

A clean, from-scratch ranging + scheduling stack for the **Makerfabs “ESP32 UWB
Pro with Display”** (Decawave **DW1000**) plus a **MATLAB** host that solves and
visualizes position. Built specifically to remove the limitations of the
existing libraries.

## Why this exists

The popular DW1000 stacks (Makerfabs `mf-DW1000`, Thomas Trojer’s
`arduino-DW1000`, jremington’s project) inherit a **~4-anchor ceiling and a
single-tag assumption**. That limit lives in the *high-level protocol layer*
(`DW1000Ranging`/`DW1000Device`/`DW1000Mac`) — its auto-discovery ranging state
machine — **not** in the low-level register driver, which is solid.

So this project:

- **Vendors only the proven low-level DW1000 driver** (`src/dw1000/`, Apache-2.0,
  unmodified) and **drops the limited protocol layer entirely**.
- Implements a **new protocol/scheduling layer**:
  - `TwrEngine` — asymmetric **double-sided two-way ranging** (DS-TWR).
  - `UwbScheduler` — the tag does **round-robin TDMA** over an explicit list of
    anchor addresses. **Adding anchors = adding a list entry.** No 4-anchor wall.
  - `HostLink` — the tag is a **dumb ranging sensor**: it streams raw ranges to a
    host over **UDP or USB serial** (compile-time selectable).
- Does **all position solving on the MATLAB host** (`matlab/+rtls`):
  multilateration (overdetermined, outlier-gated) + a constant-velocity EKF +
  live plotting. **Adding anchors never requires a firmware change.**
- Designs in the future paths: a **BNO085 IMU** hook (`SensorImu`, reserved
  packet field, EKF ready for IMU fusion) and a **multi-tag** superframe.

## Layout

```
libraries/UwbRtls/     Arduino library (the firmware stack)
  src/                 UwbRtls.h + protocol layer + vendored dw1000/ driver
  examples/            Anchor, Tag, AntennaCalibration sketches
matlab/                MATLAB host
  +rtls/               receiver, parser, multilaterator, EKF, plotter, config
  run_localization.m   main entry point
  config/              anchor geometry (anchors.json)
docs/                  hardware setup, protocol, calibration, MATLAB guide
```

## Quick start

1. **Install** the library: copy `libraries/UwbRtls` into your Arduino
   `libraries/` folder (or symlink). See [docs/01_hardware_setup.md](docs/01_hardware_setup.md).
2. **Calibrate** each board’s antenna delay
   (`examples/AntennaCalibration`) — see [docs/03_calibration.md](docs/03_calibration.md).
3. **Flash** 3+ boards as anchors (`examples/Anchor`, unique `ANCHOR_ID` each)
   and one as the tag (`examples/Tag`).
4. **Edit anchor geometry** in `matlab/config/anchors.json`.
5. **Run** `matlab/run_localization.m`. See [docs/04_matlab.md](docs/04_matlab.md).

## Roadmap (designed-in, not yet built)

- **BNO085 IMU** on the tag → tight UWB/IMU fusion in the EKF
  ([SensorImu.h](libraries/UwbRtls/src/SensorImu.h)).
- **Multiple tags** via a TDMA superframe (frame types + `tag_id` already reserved).
- **3D**: flip `dim` to 3 in the anchor config and give anchors varied heights —
  firmware unchanged.

## Attribution

`src/dw1000/` is an unmodified copy of the low-level driver from
[thotro/arduino-dw1000](https://github.com/thotro/arduino-dw1000) (Apache-2.0).
See [libraries/UwbRtls/NOTICE](libraries/UwbRtls/NOTICE).
