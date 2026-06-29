"""
Centralized configuration for the IMU+UWB fusion pipeline.

All noise parameters and pipeline constants live here. Pass a copy of
FUSION_DEFAULTS (possibly overriding individual keys) to FusionEKF,
ZuptDetector, and RtlsPipeline instead of scattering magic numbers.
"""

FUSION_DEFAULTS: dict = {
    # ── IMU-driven prediction ─────────────────────────────────────────────────
    # Accelerometer white-noise standard deviation [m/s²].
    # Drives the Q matrix via the Singer model; increase if position drifts
    # between UWB corrections when stationary.
    'sigma_accel':          0.15,

    # Accelerometer bias random-walk rate [m/s³].
    # Small constant; keeps the bias state from becoming over-confident.
    'sigma_bias_drift':     1e-3,

    # Q inflation factor applied when BNO085 orientation confidence (status) == 0.
    # At confidence 0 the prediction reverts to a constant-velocity model.
    'imu_conf_inflate':     4.0,

    # ── UWB measurement covariance (adaptive R) ───────────────────────────────
    # Fallback position sigma used when multilaterator covariance is unavailable [m].
    'sigma_pos_nominal':    0.10,

    # Expected multilateration RMSE at good geometry [m].
    # R is scaled by (1 + (rms/rms_nominal)²).
    'rms_nominal':          0.05,

    # Expected DOP-proxy (sqrt(trace(cov))) at good geometry [m].
    # R is further scaled when DOP exceeds this value.
    'dop_nominal':          0.10,

    # ── Innovation gate (NIS chi-squared test) ────────────────────────────────
    # Threshold for the Normalised Innovation Squared (NIS).
    # chi²(2 DOF, p=0.95) = 5.991 — measurements with NIS above this are rejected.
    'nis_gate':             5.991,

    # EKF is re-initialised to the raw multilat position after this many
    # consecutive rejected measurements (possible filter divergence).
    'max_consecutive_rej':  5,

    # ── ZUPT (Zero-Velocity Update) ───────────────────────────────────────────
    # Sliding window length for stationary detection [packets at ~10 Hz].
    'zupt_window':          8,

    # Mean acceleration magnitude threshold for stationary detection [m/s²].
    'zupt_accel_mean':      0.10,

    # Variance threshold for stationary detection [(m/s²)²].
    'zupt_accel_var':       0.005,

    # Angular-rate threshold for stationary detection [rad/s] (≈5°/s).
    'zupt_gyro':            0.087,

    # EKF speed below which the velocity gate passes [m/s].
    'zupt_speed':           0.05,

    # Consecutive stationary packets required before ZUPT fires (debounce).
    'zupt_debounce':        3,

    # Noise applied to the zero-velocity pseudo-measurement [m/s].
    'sigma_zupt':           0.01,

    # ── Pipeline constants (mirrors prior hard-coded values) ──────────────────
    # Median filter window per anchor [packets].
    'range_filt_n':         8,

    # Systematic range bias applied uniformly to all anchors [m].
    'range_bias_m':         0.010,

    # Age gate: drop an anchor's measurement if unseen for this long [s].
    'age_gate_sec':         3.0,

    # EMA smoothing factor for final reported position.
    'ema_alpha':            0.15,

    # Last-resort divergence guard: EKF is reset if estimated position
    # jumps more than this distance from the multilat solution [m].
    'ekf_jump_thresh':      2.0,

    # Last-resort divergence guard: EKF is reset if trace(P) exceeds this.
    'ekf_trace_thresh':     25.0,

    # Watchdog: log a warning if no packet arrives within this period [s].
    'watchdog_sec':         10.0,

    # ── Logging ───────────────────────────────────────────────────────────────
    # When True, log the full 6×2 Kalman gain matrix instead of just the diagonal.
    'log_full_kalman_gain': False,
}

# ── Tag color palette ─────────────────────────────────────────────────────────
# Colors assigned in order to each newly discovered tag_id (CSS hex strings).
# The GUI cycles through this list. Add more colours to support more tags.
TAG_PALETTE: list[str] = [
    '#00d4ff',   # cyan   — first tag
    '#e63946',   # red    — second tag
    '#a3e635',   # lime   — third tag
    '#a78bfa',   # violet — fourth tag
    '#ff9d00',   # orange — fifth tag
    '#ff3d5a',   # rose   — sixth tag
]
