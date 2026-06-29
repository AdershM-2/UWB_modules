"""
FusionEKF — loosely-coupled UWB + IMU Extended Kalman Filter.

State vector  x = [px, py, vx, vy, bx, by]   (6×1)
  px, py  — 2D position in world frame [m]
  vx, vy  — 2D velocity in world frame [m/s]
  bx, by  — horizontal accelerometer bias in world frame [m/s²]

State vector design rationale
------------------------------
- Yaw NOT included: BNO085 provides yaw directly at high quality; estimating
  it in the EKF without a dedicated reference measurement would only add
  complexity and degrade accuracy.
- Gyro bias NOT included: SH2_GYROSCOPE_CALIBRATED already has bias removed
  by the BNO085 internal fusion; redundant to re-estimate it here.
- Bias states bx, by ARE included: accelerometer offset accumulates into
  position as ½bt² between UWB corrections; the bias states prevent this.
- Reducing to 4D (no bias): valid for short runs, but position drifts during
  UWB outages and during ZUPT when the tag is stationary with a biased accel.

Process model
-------------
IMU-driven prediction (when orientation is valid, status ≥ 1):
  ẋ = F·x + G·(a_world - b)
where a_world is the BNO085 linear acceleration rotated to world frame,
and b = [bx, by] absorbs residual offset.

The discrete-time state transition (dt = actual elapsed time):
  px' = px + vx·dt + ½·(ax - bx)·dt²
  py' = py + vy·dt + ½·(ay - by)·dt²
  vx' = vx + (ax - bx)·dt
  vy' = vy + (ay - by)·dt
  bx' = bx   (bias random walk)
  by' = by

Constant-velocity fallback (when IMU absent or status == 0):
  px' = px + vx·dt
  py' = py + vy·dt
  vx' = vx
  vy' = vy
  bx' = bx
  by' = by

Process noise Q (Singer model)
  For the IMU-driven case, Q accounts for accelerometer white noise σ_a and
  bias random-walk σ_b. The block structure (position, velocity, bias):

    Q_pp = σ_a²·(dt³/3)·I₂    Q_pv = σ_a²·(dt²/2)·I₂    Q_pb = 0
    Q_vv = σ_a²·dt·I₂          Q_vb = 0
    Q_bb = σ_b²·dt·I₂

  For the CV fallback the same formula applies but σ_a uses the nominal
  constant-velocity model uncertainty (slightly larger to reflect missing IMU).

Measurement model (UWB position correction)
  H = [I₂  0₂₂  0₂₂]   → z = [px, py]
  R = adaptive covariance (inflated by RMS, NLOS, DOP — computed externally)

Measurement model (ZUPT)
  H_zupt = [0₂₂  I₂  0₂₂]   → z = [0, 0]   (zero velocity pseudo-measurement)
  R_zupt = (σ_zupt)²·I₂

Generic update
  Any sensor can be fused by passing z, H, R to update(). The method:
  1. Computes innovation and NIS
  2. Gates by chi²(DOF, p=0.95) — auto-sized to len(z)
  3. Performs standard EKF update if gate passes
  4. Returns an UpdateResult with all diagnostics for logging
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional
import numpy as np
from scipy.stats import chi2


# ── Result type returned by update() ────────────────────────────────────────

@dataclass
class UpdateResult:
    accepted:        bool
    label:           str   = ''
    NIS:             float = 0.0
    gate_threshold:  float = 0.0
    innovation:      Optional[np.ndarray] = None
    S:               Optional[np.ndarray] = None   # innovation covariance
    K:               Optional[np.ndarray] = None   # Kalman gain
    correction_mag:  float = 0.0                   # ||K @ innovation||


# ── FusionEKF ────────────────────────────────────────────────────────────────

class FusionEKF:
    """
    Loosely-coupled 2D UWB + IMU Extended Kalman Filter.
    State: [px, py, vx, vy, bx, by]  (6×1)
    """

    def __init__(self, cfg: dict):
        self._cfg = cfg
        self._x   = np.zeros(6)
        self._P   = np.eye(6) * 1e4   # uninitialised: large uncertainty
        self._initialized = False

        # Chi-squared gate thresholds cached per DOF
        self._chi2_cache: dict[int, float] = {}

        # Diagnostics from the last update call (for logging)
        self._last_result: Optional[UpdateResult] = None

    # ── Initialisation ───────────────────────────────────────────────────────

    def initialize(self, pos: np.ndarray) -> None:
        """Set position to pos, zero velocity and bias, moderate P."""
        self._x = np.array([pos[0], pos[1], 0.0, 0.0, 0.0, 0.0])
        self._P = np.diag([0.25, 0.25, 1.0, 1.0, 0.01, 0.01])
        self._initialized = True

    @property
    def initialized(self) -> bool:
        return self._initialized

    # ── State accessors ──────────────────────────────────────────────────────

    @property
    def position(self) -> np.ndarray:
        return self._x[:2].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self._x[2:4].copy()

    @property
    def accel_bias(self) -> np.ndarray:
        return self._x[4:6].copy()

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self._x[2:4]))

    @property
    def heading_deg(self) -> float:
        import math
        vx, vy = self._x[2], self._x[3]
        if abs(vx) < 1e-6 and abs(vy) < 1e-6:
            return 0.0
        return math.degrees(math.atan2(vy, vx))

    @property
    def P(self) -> np.ndarray:
        return self._P.copy()

    @property
    def P_diag(self) -> list[float]:
        return [round(float(v), 6) for v in np.diag(self._P)]

    @property
    def last_result(self) -> Optional[UpdateResult]:
        return self._last_result

    # ── Predict ──────────────────────────────────────────────────────────────

    def predict(self, dt: float, a_world_xy: np.ndarray | None = None,
                orientation_confidence: int = 3) -> None:
        """
        Propagate state forward by dt seconds.

        a_world_xy: (2,) world-frame horizontal acceleration [m/s²] after bias
                    subtraction will happen inside this method; pass None to
                    use the constant-velocity fallback.
        orientation_confidence: BNO085 status 0–3; inflates Q when < 3.
        """
        if not self._initialized:
            return

        dt = max(dt, 1e-4)

        sa  = self._cfg.get('sigma_accel',       0.15)
        sb  = self._cfg.get('sigma_bias_drift',  1e-3)
        inf = self._cfg.get('imu_conf_inflate',  4.0)

        # Inflate process noise when orientation is unreliable
        if orientation_confidence == 0:
            # Fully fall back to CV — treat as if no IMU
            a_world_xy = None
            q_scale = inf
        else:
            q_scale = 1.0 + (1.0 - orientation_confidence / 3.0) * (inf - 1.0)

        # State transition
        px, py, vx, vy, bx, by = self._x

        if a_world_xy is not None:
            ax = float(a_world_xy[0]) - bx
            ay = float(a_world_xy[1]) - by
            self._x[0] = px + vx*dt + 0.5*ax*dt*dt
            self._x[1] = py + vy*dt + 0.5*ay*dt*dt
            self._x[2] = vx + ax*dt
            self._x[3] = vy + ay*dt
            # bx, by unchanged (random walk — covariance handles it)
        else:
            # Constant-velocity fallback
            self._x[0] = px + vx*dt
            self._x[1] = py + vy*dt
            # vx, vy, bx, by unchanged

        # Transition Jacobian F (6×6)
        F = np.eye(6)
        if a_world_xy is not None:
            F[0, 2] = dt;  F[0, 4] = -0.5*dt*dt
            F[1, 3] = dt;  F[1, 5] = -0.5*dt*dt
            F[2, 4] = -dt
            F[3, 5] = -dt
        else:
            F[0, 2] = dt
            F[1, 3] = dt

        # Process noise Q
        dt2 = dt*dt
        dt3 = dt2*dt
        sa2 = sa*sa * q_scale
        sb2 = sb*sb

        Q = np.zeros((6, 6))
        # Position-position block
        Q[0, 0] = Q[1, 1] = sa2 * dt3 / 3.0
        # Position-velocity cross
        Q[0, 2] = Q[2, 0] = Q[1, 3] = Q[3, 1] = sa2 * dt2 / 2.0
        # Velocity-velocity block
        Q[2, 2] = Q[3, 3] = sa2 * dt
        # Bias random walk
        Q[4, 4] = Q[5, 5] = sb2 * dt

        self._P = F @ self._P @ F.T + Q

    # ── Generic measurement update ────────────────────────────────────────────

    def update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray,
               label: str = '') -> UpdateResult:
        """
        Standard EKF linearised measurement update with NIS innovation gate.

        z : (m,) measurement vector
        H : (m, 6) measurement Jacobian
        R : (m, m) measurement noise covariance
        label : source tag for logging ('UWB', 'ZUPT', ...)

        Returns UpdateResult with full diagnostics regardless of gate outcome.
        The state is only modified when the gate passes (accepted=True).
        """
        if not self._initialized:
            return UpdateResult(accepted=False, label=label)

        m = len(z)
        innovation = z - H @ self._x                   # (m,)
        S          = H @ self._P @ H.T + R             # (m, m)

        # NIS gate
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return UpdateResult(accepted=False, label=label, NIS=float('inf'))

        NIS = float(innovation @ S_inv @ innovation)   # scalar

        # Cache chi² threshold per DOF
        if m not in self._chi2_cache:
            self._chi2_cache[m] = float(chi2.ppf(0.95, df=m))
        gate = self._chi2_cache[m]

        # Check explicit override from config (for UWB 2-DOF case)
        if m == 2:
            gate = float(self._cfg.get('nis_gate', gate))

        result = UpdateResult(
            accepted=NIS <= gate,
            label=label,
            NIS=round(NIS, 4),
            gate_threshold=round(gate, 4),
            innovation=innovation.copy(),
            S=S.copy(),
        )

        if result.accepted:
            K            = self._P @ H.T @ S_inv           # (6, m)
            dx           = K @ innovation                   # (6,)
            self._x     += dx
            self._P      = (np.eye(6) - K @ H) @ self._P
            # Symmetrise to guard against numerical drift
            self._P      = 0.5 * (self._P + self._P.T)
            result.K              = K.copy()
            result.correction_mag = round(float(np.linalg.norm(dx)), 6)

        self._last_result = result
        return result

    # ── Convenience wrappers ─────────────────────────────────────────────────

    def update_position(self, pos: np.ndarray, R: np.ndarray) -> UpdateResult:
        """UWB position correction. H selects [px, py] from state."""
        H = np.zeros((2, 6))
        H[0, 0] = H[1, 1] = 1.0
        return self.update(pos, H, R, label='UWB')

    def update_zupt(self, sigma_v: float | None = None) -> UpdateResult:
        """Zero-velocity pseudo-measurement. H selects [vx, vy] from state."""
        if sigma_v is None:
            sigma_v = float(self._cfg.get('sigma_zupt', 0.01))
        H = np.zeros((2, 6))
        H[0, 2] = H[1, 3] = 1.0
        R = np.eye(2) * sigma_v**2
        return self.update(np.zeros(2), H, R, label='ZUPT')

    # ── Divergence guard ─────────────────────────────────────────────────────

    def check_divergence(self, ref_pos: np.ndarray) -> bool:
        """
        Last-resort divergence guard. Returns True (and re-initialises) if:
          - position differs from ref_pos by more than ekf_jump_thresh, OR
          - trace(P) exceeds ekf_trace_thresh.
        """
        if not self._initialized:
            return False
        jump = float(np.linalg.norm(self._x[:2] - ref_pos))
        if jump > float(self._cfg.get('ekf_jump_thresh', 2.0)):
            self.initialize(ref_pos)
            return True
        if np.trace(self._P) > float(self._cfg.get('ekf_trace_thresh', 25.0)):
            self.initialize(ref_pos)
            return True
        return False
