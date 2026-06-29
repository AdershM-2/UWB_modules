#!/usr/bin/env python3
"""
run_localization.py — UWB RTLS host: receive ranges, solve, filter, serve GUI.

Python replacement for run_localization.m.  Receives RTLS packets from one or
more tags over UDP (or serial), runs per-tag multilateration + EKF + EMA, then
pushes results to a browser GUI via WebSocket.

Each tag that produces a packet automatically gets its own independent pipeline
state (TagState). No tag IDs are hard-coded; the system scales to N tags.

Multi-point survey: the GUI can request distance captures at known grid
positions.  After enough captures (≥4), the optimizer jointly solves for
anchor positions and writes anchors.json.

Usage:
    python run_localization.py                  # UDP on port 4100
    python run_localization.py --serial /dev/ttyUSB0
    python run_localization.py --udp-port 4100 --ws-port 8765
"""

import sys, os, json, time, argparse, asyncio, socket, threading, signal
from pathlib import Path
from datetime import datetime
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

# Add parent so `rtls` is importable when run as a script
sys.path.insert(0, str(Path(__file__).resolve().parent))

from rtls import FrameParser, AnchorConfig, Multilaterator
from rtls.fusion_ekf     import FusionEKF
from rtls.imu_integrator import ImuIntegrator
from rtls.zupt_detector  import ZuptDetector
from rtls.system_health  import SystemHealth
from rtls.fusion_config  import FUSION_DEFAULTS, TAG_PALETTE
from multipoint_survey import solve_anchor_positions, write_anchors_json, SURVEY_GRID
from calibration_manager import CalibrationManager
import numpy as np

try:
    import websockets
    import websockets.server
except ImportError:
    print("Missing dependency: pip install websockets")
    sys.exit(1)

try:
    import serial as pyserial
except ImportError:
    pyserial = None

# ── Defaults ──────────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_DIR = SCRIPT_DIR.parent / 'matlab' / 'config'

DEFAULT_UDP_PORT  = 4100
DEFAULT_WS_PORT   = 8765
DEFAULT_BAUD      = 115200

# Multilaterator constants (not in fusion_config since they are solver-level)
RANGE_SIGMA = 0.20
GATE_K      = 2.5


# ── Per-tag state ─────────────────────────────────────────────────────────────

@dataclass
class TagState:
    """
    All mutable filter state for one discovered tag.

    One TagState is created the first time a packet with a new tag_id arrives.
    The pipeline keeps a dict[int, TagState] and dispatches every incoming
    packet to the correct state.  No tag IDs are pre-configured.
    """
    tag_id: int

    # Filter components (created inside RtlsPipeline._get_or_create_state)
    ekf:              Optional[FusionEKF]    = field(default=None)
    imu_i:            ImuIntegrator          = field(default_factory=ImuIntegrator)
    zupt:             Optional[ZuptDetector] = field(default=None)
    health:           SystemHealth           = field(default_factory=SystemHealth)

    # Per-anchor sliding history (median filter) keyed by anchor int id
    range_hist:       dict                   = field(default_factory=dict)
    # Per-anchor last-seen monotonic time (for age-gate eviction)
    anchor_last_seen: dict                   = field(default_factory=dict)

    # Tracking state
    last_pos:         Optional[np.ndarray]   = field(default=None)
    ema_pos:          Optional[np.ndarray]   = field(default=None)
    prev_t:           Optional[float]        = field(default=None)
    last_imu:         Optional[dict]         = field(default=None)

    # Per-tag counters
    pkt_count:        int                    = 0
    ekf_resets:       int                    = 0


# ── Pipeline class ─────────────────────────────────────────────────────────────

class RtlsPipeline:
    """
    Full signal pipeline, dispatched per tag_id:

        parse → per-tag median filter → IMU predict → ZUPT → multilat
        → adaptive R → FusionEKF update → EMA → result dict

    Adding a new tag requires no code change: the pipeline creates a fresh
    TagState on the first packet from any previously unseen tag_id.
    """

    def __init__(self, cfg: AnchorConfig, fusion_cfg: dict | None = None):
        self._fcfg = dict(FUSION_DEFAULTS)
        if fusion_cfg:
            self._fcfg.update(fusion_cfg)

        self.cfg = cfg
        self.ml  = Multilaterator(cfg.dim)
        self.ml.gate_k      = GATE_K
        self.ml.range_sigma = RANGE_SIGMA

        # Per-tag state — populated on first packet from each tag
        self._tag_states: dict[int, TagState] = {}

        # Pipeline-level tunables shared across all tags (global tuning panel)
        self.range_filt_n = int(self._fcfg['range_filt_n'])
        self.ema_alpha    = float(self._fcfg['ema_alpha'])
        self.range_bias_m = float(self._fcfg['range_bias_m'])

        # Tier-1 per-anchor bias corrections shared across all tags
        # (anchors have one geometric bias regardless of which tag ranges to them)
        self.per_anchor_bias_m: dict[int, float] = {}

        # Global packet counter (all tags combined)
        self.pkt_count = 0

    # ── Tag state management ─────────────────────────────────────────────────

    def _get_or_create_state(self, tag_id: int) -> TagState:
        """Return the TagState for tag_id, creating it on first encounter."""
        if tag_id not in self._tag_states:
            state = TagState(tag_id=tag_id)
            state.ekf  = FusionEKF(self._fcfg)
            state.zupt = ZuptDetector(self._fcfg)
            self._tag_states[tag_id] = state
            print(f"[TAG]  New tag discovered: 0x{tag_id:02X}  (decimal {tag_id})")
        return self._tag_states[tag_id]

    @property
    def tag_ids(self) -> list[int]:
        """All tag IDs seen so far, in discovery order."""
        return list(self._tag_states.keys())

    # ── Live tuning ──────────────────────────────────────────────────────────

    def update_tuning(self, params: dict):
        if 'rangeFiltN' in params:
            self.range_filt_n = max(1, int(params['rangeFiltN']))
        if 'sigmaAccel' in params:
            self._fcfg['sigma_accel'] = float(params['sigmaAccel'])
        if 'ekfQAccel' in params:
            self._fcfg['sigma_accel'] = float(params['ekfQAccel'])
        if 'emaAlpha'   in params:
            self.ema_alpha = float(params['emaAlpha'])
        if 'gateK'      in params:
            self.ml.gate_k = float(params['gateK'])
        if 'rangeBiasM' in params:
            self.range_bias_m = float(params['rangeBiasM'])
        if 'zuptAccelMean' in params:
            self._fcfg['zupt_accel_mean'] = float(params['zuptAccelMean'])
            # Rebuild ZUPT detector for every existing tag (config changed)
            for state in self._tag_states.values():
                state.zupt = ZuptDetector(self._fcfg)

    # ── Calibration ──────────────────────────────────────────────────────────

    def apply_t1_calibration(self, bias_dict: dict[int, float]):
        """
        Apply Tier-1 per-anchor biases.

        Biases are anchor-specific (not tag-specific): they correct for the
        physical antenna geometry and multipath at each anchor.  The same
        correction applies regardless of which tag is ranging.

        Design note: the data structure is dict[anchor_id, bias_m].  If future
        per-tag-per-anchor calibration is needed, expand to
        dict[tag_id, dict[anchor_id, bias_m]] and look up by (tag_id, anchor_id).
        """
        self.per_anchor_bias_m = bias_dict.copy()
        self.range_bias_m = 0.0
        # Clear range histories so stale pre-calibration samples don't persist
        for state in self._tag_states.values():
            state.range_hist.clear()

    # ── Adaptive measurement covariance ──────────────────────────────────────

    def _adaptive_R(self, info, fp_active, q_active) -> np.ndarray:
        """Scale info.cov by RMSE, NLOS gap, and DOP proxy."""
        if np.all(np.isfinite(info.cov[:2, :2])):
            R_base = info.cov[:2, :2].copy()
        else:
            s = self._fcfg['sigma_pos_nominal']
            R_base = np.eye(2) * s * s

        # Factor 1: RMSE inflation
        rms_n = self._fcfg['rms_nominal']
        rms_factor = 1.0 + (float(info.rms) / rms_n) ** 2

        # Factor 2: NLOS score — positive gap means NLOS (rx > fp in dBm magnitude)
        nlos_scores = []
        for fp, rx in zip(fp_active, q_active):
            if fp != 0.0 and rx != 0.0:
                gap = rx - fp
                nlos_scores.append(max(0.0, gap / 6.0))
        nlos_factor = 1.0 + (float(np.mean(nlos_scores)) if nlos_scores else 0.0)

        # Factor 3: DOP proxy — bad geometry inflates R
        dop_n = self._fcfg['dop_nominal']
        dop = float(np.sqrt(np.trace(R_base)))
        dop_factor = 1.0 + max(0.0, (dop - dop_n) / dop_n)

        scale = float(np.clip(rms_factor * nlos_factor * dop_factor, 1.0, 50.0))
        return R_base * scale

    # ── Main process ─────────────────────────────────────────────────────────

    def process(self, line: str) -> dict | None:
        """
        Process one RTLS line.  Dispatches to the correct TagState based on
        pkt.tag_id.  Returns a result dict with tag_id included, or None if the
        packet is invalid or the solver failed.
        """
        pkt = FrameParser.parse(line)
        if not pkt.valid:
            return None

        self.pkt_count += 1
        state = self._get_or_create_state(pkt.tag_id)
        state.pkt_count += 1
        now = time.monotonic()
        age_gate = self._fcfg['age_gate_sec']

        # Age-gate: evict stale anchors for THIS tag only
        stale = [k for k, t in state.anchor_last_seen.items()
                 if now - t > age_gate]
        for k in stale:
            state.range_hist.pop(k, None)
            state.anchor_last_seen.pop(k, None)

        # Map anchor ids → coordinates; keep only known anchors
        A, found = self.cfg.coords_for(pkt.ids)
        d_raw   = np.array(pkt.dist)
        ids_arr = np.array(pkt.ids)
        fp_arr  = np.array(pkt.fp)    if pkt.fp    else np.zeros(len(pkt.ids))
        q_arr   = np.array(pkt.q)

        mask      = np.array(found, dtype=bool)
        A         = A[mask]
        d_raw     = d_raw[mask]
        active    = ids_arr[mask]
        fp_active = fp_arr[mask]
        q_active  = q_arr[mask]

        if len(d_raw) < self.cfg.dim + 1:
            return None

        # ── Per-anchor sliding median filter + T1 bias ────────────────────────
        # Keep a copy of raw ranges before any filtering for the log record.
        d_raw_for_log = d_raw.copy()
        d_filt = d_raw.copy()
        for k, aid in enumerate(active):
            aid_i = int(aid)
            state.anchor_last_seen[aid_i] = now
            if aid_i not in state.range_hist:
                state.range_hist[aid_i] = deque(maxlen=self.range_filt_n)
            buf = state.range_hist[aid_i]
            corrected = d_raw[k] - self.per_anchor_bias_m.get(aid_i, 0.0)
            buf.append(corrected)
            while len(buf) > self.range_filt_n:
                buf.popleft()
            d_filt[k] = float(np.median(list(buf)))

        d_filt = np.maximum(0.05, d_filt - self.range_bias_m)

        # dt from actual firmware timestamps
        if state.prev_t is None:
            dt = 0.1
        else:
            dt = max((pkt.t_ms - state.prev_t) / 1000.0, 1e-3)
        state.prev_t = pkt.t_ms

        # ── IMU: transform to world frame ─────────────────────────────────────
        a_world_xy   = None
        orient_conf  = 0
        imu_gyro     = []
        a_world_log  = [0.0, 0.0]

        state.health.imu_available = pkt.imu is not None
        if pkt.imu is not None:
            orient_conf = pkt.imu.status
            imu_gyro    = list(pkt.imu.gyro)
            state.health.gyro_available          = bool(pkt.imu.gyro)
            state.health.orientation_confidence  = orient_conf
            state.health.orientation_valid       = orient_conf >= 1

            a_world_xy = state.imu_i.transform_accel(pkt.imu)
            if a_world_xy is not None:
                a_world_log = [round(float(a_world_xy[0]), 4),
                               round(float(a_world_xy[1]), 4)]

            roll, pitch, yaw = state.imu_i.roll_pitch_yaw_deg(pkt.imu)
            state.last_imu = {
                'roll':  round(roll,  2),
                'pitch': round(pitch, 2),
                'yaw':   round(yaw,   2),
                'ax':    round(pkt.imu.acc[0], 3),
                'ay':    round(pkt.imu.acc[1], 3),
                'az':    round(pkt.imu.acc[2], 3),
            }

        # ── EKF predict ───────────────────────────────────────────────────────
        if state.ekf.initialized:
            state.ekf.predict(dt, a_world_xy, orient_conf)
            state.health.prediction_mode = 'IMU' if a_world_xy is not None else 'CV'

        # ── ZUPT ─────────────────────────────────────────────────────────────
        v_ekf = state.ekf.velocity if state.ekf.initialized else np.zeros(2)
        a_for_zupt = a_world_xy
        if a_for_zupt is None and pkt.imu is not None:
            a_for_zupt = np.array(pkt.imu.acc[:2])
        is_stationary = state.zupt.update(a_for_zupt, imu_gyro, v_ekf)
        state.health.zupt_active = is_stationary
        if is_stationary and state.ekf.initialized:
            state.ekf.update_zupt()

        # ── Multilateration ───────────────────────────────────────────────────
        init_guess = state.ekf.position if state.ekf.initialized else state.last_pos
        pos, info = self.ml.solve(A, d_filt, init_guess)

        if not info.ok or not np.all(np.isfinite(pos)):
            return None
        state.last_pos = pos.copy()

        n_used     = int(np.sum(info.used))
        n_rej_ml   = len(active) - n_used
        state.health.n_anchors_used     = n_used
        state.health.n_anchors_rejected = n_rej_ml
        state.health.uwb_available      = True

        # ── EKF initialise (first valid fix) + UWB correction ────────────────
        if not state.ekf.initialized:
            state.ekf.initialize(pos)

        R_eff        = self._adaptive_R(info, fp_active, q_active)
        update_res   = state.ekf.update_position(pos, R_eff)

        state.health.correction_accepted = update_res.accepted
        state.health.correction_rejected = not update_res.accepted
        if update_res.accepted:
            state.health.n_uwb_accepted += 1
            state.health.nis_last = update_res.NIS
        else:
            state.health.n_uwb_rejected += 1

        # ── Divergence guard (last resort) ────────────────────────────────────
        if state.ekf.check_divergence(pos):
            state.ekf_resets += 1
            state.ema_pos = None
            state.zupt.reset()

        # ── EMA on EKF position ───────────────────────────────────────────────
        fpos = state.ekf.position
        if state.ema_pos is None:
            state.ema_pos = fpos.copy()
        else:
            state.ema_pos = (self.ema_alpha * fpos
                             + (1 - self.ema_alpha) * state.ema_pos)

        # ── System health composite confidence ────────────────────────────────
        state.health.ekf_initialized = True
        state.health.compute_confidence()

        # ── Build result dict ─────────────────────────────────────────────────
        # Filtered distances (keyed as d<anchor_id>, used by GUI range bars)
        dists_filt = {f'd{int(aid)}': round(float(d_filt[k]), 4)
                      for k, aid in enumerate(active)}
        # Raw distances before filtering (for logging/debugging)
        dists_raw  = {f'r{int(aid)}': round(float(d_raw_for_log[k]), 4)
                      for k, aid in enumerate(active)}

        innovation  = (update_res.innovation.tolist()
                       if update_res.innovation is not None else [0.0, 0.0])
        K_diag      = ([round(float(update_res.K[i, i % 2]), 6)
                        for i in range(6)]
                       if update_res.K is not None else [0.0] * 6)

        result: dict = {
            # ── Tag identity ────────────────────────────────────────────────
            'tag_id':    pkt.tag_id,
            'pkt_n':     state.pkt_count,        # per-tag packet counter

            # ── Core position (GUI-compatible keys) ─────────────────────────
            't_ms':  pkt.t_ms,
            'x':     round(float(pos[0]), 4),    # multilateration output
            'y':     round(float(pos[1]), 4),
            'ex':    round(float(state.ema_pos[0]), 4),  # EMA-smoothed EKF pos
            'ey':    round(float(state.ema_pos[1]), 4),
            'rmse':  round(float(info.rms), 4),
            'nUsed': n_used,
            'ekfR':  state.ekf_resets,

            # ── Anchor measurements ─────────────────────────────────────────
            'anchor_ids':  [int(a) for a in active],  # which anchors were used
            **dists_filt,                              # d1, d2, … (filtered)
            **dists_raw,                               # r1, r2, … (raw pre-filter)

            # ── EKF state ───────────────────────────────────────────────────
            'ekf_pos':    [round(float(fpos[0]), 4), round(float(fpos[1]), 4)],
            'ekf_P_diag': state.ekf.P_diag,
            'vx':    round(float(state.ekf.velocity[0]), 4),
            'vy':    round(float(state.ekf.velocity[1]), 4),
            'speed': round(state.ekf.speed, 4),
            'heading': round(state.ekf.heading_deg, 2),
            'bx':    round(float(state.ekf.accel_bias[0]), 6),
            'by':    round(float(state.ekf.accel_bias[1]), 6),

            # ── IMU ─────────────────────────────────────────────────────────
            **(({'imu': state.last_imu}) if state.last_imu is not None else {}),
            'imu_gyro':        [round(v, 4) for v in imu_gyro],
            'imu_orient_conf': orient_conf,
            'imu_ax_world':    a_world_log[0],
            'imu_ay_world':    a_world_log[1],

            # ── Filter diagnostics ──────────────────────────────────────────
            'zupt':            is_stationary,
            'pred_mode':       state.health.prediction_mode,
            'innovation':      [round(v, 5) for v in innovation],
            'nis':             update_res.NIS,
            'meas_accepted':   update_res.accepted,
            'correction_mag':  update_res.correction_mag,

            # ── System health ────────────────────────────────────────────────
            'health':          state.health.to_dict(),
        }

        if self._fcfg.get('log_full_kalman_gain') and update_res.K is not None:
            result['K_full'] = update_res.K.tolist()
        else:
            result['K_diag'] = K_diag

        return result


# ── WebSocket server + UDP/serial reader ──────────────────────────────────────
clients: set = set()
pipeline: RtlsPipeline = None   # type: ignore
log_fh = None
cfg_global: AnchorConfig = None  # type: ignore
serial_port = None
calib_mgr: CalibrationManager = None  # type: ignore

# ── Multi-point survey state ──────────────────────────────────────────────────
_survey_captures: list = []
_survey_capture_active: bool = False
_survey_capture_buf: list = []
_survey_capture_target: int = 50
_survey_capture_point_idx: int = -1


async def ws_handler(websocket):
    clients.add(websocket)
    print(f"[WS]  Browser connected  ({len(clients)} total)")
    try:
        anchors_msg = json.dumps({
            'type': 'config',
            'dim':  cfg_global.dim,
            'bounds': cfg_global.bounds,
            'anchors': [
                {'id': cfg_global.ids[i],
                 'x': float(cfg_global.coords[i, 0]),
                 'y': float(cfg_global.coords[i, 1])}
                for i in range(len(cfg_global.ids))
            ],
            'calibration': calib_mgr.status_msg() if calib_mgr else {},
            # Tag palette for the GUI — colour assignments are data-driven
            'tag_palette': TAG_PALETTE,
        })
        await websocket.send(anchors_msg)
    except Exception:
        pass

    try:
        async for msg in websocket:
            try:
                data = json.loads(msg)
                if data.get('type') == 'tune':
                    pipeline.update_tuning(data)
                elif data.get('type') == 'survey_capture':
                    await _start_survey_capture(data)
                elif data.get('type') == 'survey_compute':
                    await _do_survey_compute()
                elif data.get('type') == 'survey_clear':
                    _survey_captures.clear()
                    await broadcast(json.dumps({'type': 'survey_cleared'}))
                    print("[SURVEY] Cleared all captures")
                elif data.get('type') == 'CALIB_T1_START':
                    await _handle_t1_start(data)
                elif data.get('type') == 'CALIB_T1_STOP':
                    await _handle_t1_stop()
                elif data.get('type') == 'CALIB_T1_SAVE':
                    await _handle_t1_save()
            except Exception as e:
                print(f"[WS] Error handling message: {e}")
    finally:
        clients.discard(websocket)
        print(f"[WS]  Browser disconnected  ({len(clients)} total)")


async def broadcast(payload: str):
    if clients:
        await asyncio.gather(
            *[c.send(payload) for c in list(clients)],
            return_exceptions=True
        )


async def _start_survey_capture(data: dict):
    global _survey_capture_active, _survey_capture_buf
    global _survey_capture_target, _survey_capture_point_idx

    if _survey_capture_active:
        print("[SURVEY] Capture already in progress")
        return

    point_idx = data.get('point_idx', -1)
    n_samples = data.get('n_samples', 50)

    _survey_capture_point_idx = point_idx
    _survey_capture_target = max(10, n_samples)
    _survey_capture_buf = []
    _survey_capture_active = True

    grid_pt = SURVEY_GRID[point_idx] if 0 <= point_idx < len(SURVEY_GRID) else None
    label = f"({grid_pt[0]}, {grid_pt[1]})" if grid_pt else f"idx={point_idx}"
    print(f"[SURVEY] Capture started at {label}  (collecting {n_samples} sweeps)")
    await broadcast(json.dumps({
        'type': 'survey_capture_started',
        'point_idx': point_idx,
        'n_samples': n_samples,
    }))


def _process_survey_sweep(result: dict):
    """Called for each RTLS fix during an active capture."""
    global _survey_capture_active

    if not _survey_capture_active:
        return None

    dists = {}
    for key, val in result.items():
        if key.startswith('d') and key[1:].isdigit():
            aid = int(key[1:])
            dists[aid] = val

    if dists:
        _survey_capture_buf.append(dists)

    n = len(_survey_capture_buf)
    progress_msg = {
        'type': 'survey_capture_progress',
        'point_idx': _survey_capture_point_idx,
        'n': n,
        'target': _survey_capture_target,
    }

    if n >= _survey_capture_target:
        _survey_capture_active = False
        all_aids = set()
        for d in _survey_capture_buf:
            all_aids.update(d.keys())

        avg_dists = {}
        for aid in sorted(all_aids):
            vals = [d[aid] for d in _survey_capture_buf if aid in d]
            if vals:
                avg_dists[aid] = float(np.median(vals))

        capture = {'dists': avg_dists, 'point_idx': _survey_capture_point_idx}
        _survey_captures.append(capture)

        grid_pt = SURVEY_GRID[_survey_capture_point_idx] \
            if 0 <= _survey_capture_point_idx < len(SURVEY_GRID) else None
        label = f"({grid_pt[0]}, {grid_pt[1]})" if grid_pt else "?"
        dists_str = '  '.join(f'A{k}={v:.3f}m' for k, v in avg_dists.items())
        print(f"[SURVEY] Capture done at {label}: {dists_str}")

        done_msg = {
            'type': 'survey_capture_done',
            'point_idx': _survey_capture_point_idx,
            'avg_dists': {str(k): round(v, 4) for k, v in avg_dists.items()},
            'n_total_captures': len(_survey_captures),
        }
        return done_msg

    return progress_msg


async def _do_survey_compute():
    global cfg_global, pipeline

    if len(_survey_captures) < 4:
        msg = f"Need ≥4 captures, have {len(_survey_captures)}"
        print(f"[SURVEY] {msg}")
        await broadcast(json.dumps({'type': 'survey_error', 'msg': msg}))
        return

    anchor_ids = sorted(cfg_global.ids) if cfg_global else [1, 2, 3]
    dim = cfg_global.dim if cfg_global else 2
    z_height = 0.0
    if cfg_global and cfg_global.coords.shape[1] >= 3:
        z_height = float(cfg_global.coords[0, 2])

    print(f"[SURVEY] Computing positions from {len(_survey_captures)} captures...")
    await broadcast(json.dumps({'type': 'survey_computing'}))

    try:
        positions, info = solve_anchor_positions(
            _survey_captures, anchor_ids, dim=dim
        )
        write_anchors_json(
            positions, anchor_ids, dim=dim, z_height=z_height,
            path=CONFIG_DIR / 'anchors.json'
        )
    except Exception as e:
        msg = f"Optimizer failed: {e}"
        print(f"[SURVEY] {msg}")
        await broadcast(json.dumps({'type': 'survey_error', 'msg': msg}))
        return

    anchors_result = []
    for i, aid in enumerate(anchor_ids):
        anchors_result.append({
            'id': aid,
            'x': round(float(positions[i, 0]), 4),
            'y': round(float(positions[i, 1]), 4),
        })

    result_msg = json.dumps({
        'type': 'survey_result',
        'anchors': anchors_result,
        'residual_rms': round(info['residual_rms'], 4),
        'n_captures': info['n_captures'],
        'success': info['success'],
    })
    print(f"[SURVEY] Done — residual RMS={info['residual_rms']:.4f} m")
    for a in anchors_result:
        print(f"  A{a['id']}: ({a['x']:+.3f}, {a['y']:+.3f})")
    await broadcast(result_msg)

    # Hot-reload config and pipeline (per-tag state is discarded intentionally —
    # anchor positions changed, so all EKFs must reinitialise from scratch)
    anchors_path = CONFIG_DIR / 'anchors.json'
    cfg_global = AnchorConfig.from_json(str(anchors_path))
    pipeline = RtlsPipeline(cfg_global)
    if calib_mgr:
        calib_mgr.on_anchors_updated(cfg_global)
    print(f"[SURVEY] Pipeline reloaded with new anchor positions")

    config_msg = json.dumps({
        'type': 'config',
        'dim':  cfg_global.dim,
        'bounds': cfg_global.bounds,
        'anchors': [
            {'id': cfg_global.ids[i],
             'x': float(cfg_global.coords[i, 0]),
             'y': float(cfg_global.coords[i, 1])}
            for i in range(len(cfg_global.ids))
        ],
        'calibration': calib_mgr.status_msg() if calib_mgr else {},
        'tag_palette': TAG_PALETTE,
    })
    await broadcast(config_msg)


async def _handle_t1_start(data: dict):
    if calib_mgr is None:
        return
    n = int(data.get('n_samples', 500))
    msg = calib_mgr.start_collection(n)
    await broadcast(json.dumps(msg))


async def _handle_t1_stop():
    if calib_mgr is None:
        return
    msg = calib_mgr.stop_collection()
    await broadcast(json.dumps(msg))
    await broadcast(json.dumps({'type': 'CALIB_T1_STATUS', **calib_mgr.status_msg()}))


async def _handle_t1_save():
    global pipeline
    if calib_mgr is None:
        return
    msg = calib_mgr.apply_and_save()
    if msg.get('type') == 'CALIB_T1_SAVED':
        pipeline.apply_t1_calibration(calib_mgr.per_anchor_bias_m)
        print("[CAL] T1 applied to pipeline")
    await broadcast(json.dumps(msg))


async def udp_reader(port: int):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', port))
    sock.setblocking(True)
    loop = asyncio.get_event_loop()
    print(f"[UDP] Listening on port {port}...")

    while True:
        try:
            data, addr = await loop.run_in_executor(None, lambda: sock.recvfrom(512))
            line = data.decode('utf-8', errors='replace').strip()
            result = pipeline.process(line)
            if result:
                payload = json.dumps(result)
                await broadcast(payload)
                if log_fh:
                    log_fh.write(payload + '\n')
                    log_fh.flush()
                print(f"[FIX] tag=0x{result['tag_id']:02X}"
                      f"  ({result['ex']:+.3f}, {result['ey']:+.3f})"
                      f"  rms={result['rmse']:.3f}"
                      f"  pkt#{result['pkt_n']}")

                if _survey_capture_active:
                    survey_msg = _process_survey_sweep(result)
                    if survey_msg:
                        await broadcast(json.dumps(survey_msg))

            if calib_mgr and calib_mgr.state == CalibrationManager.COLLECTING:
                for cal_msg in calib_mgr.feed_line(line):
                    await broadcast(json.dumps(cal_msg))
        except Exception as e:
            print(f"[UDP] Error: {e}")
            await asyncio.sleep(0.05)


async def serial_reader(port: str, baud: int):
    global serial_port
    if pyserial is None:
        print("ERROR: pip install pyserial")
        return
    ser = pyserial.Serial(port, baud, timeout=1.0)
    serial_port = ser
    loop = asyncio.get_event_loop()
    print(f"[SER] Listening on {port} @ {baud} baud...")

    while True:
        try:
            raw = await loop.run_in_executor(None, ser.readline)
            line = raw.decode('utf-8', errors='replace').strip()
            if not line:
                continue

            result = pipeline.process(line)
            if result:
                payload = json.dumps(result)
                await broadcast(payload)
                if log_fh:
                    log_fh.write(payload + '\n')
                    log_fh.flush()
                print(f"[FIX] tag=0x{result['tag_id']:02X}"
                      f"  ({result['ex']:+.3f}, {result['ey']:+.3f})"
                      f"  rms={result['rmse']:.3f}"
                      f"  pkt#{result['pkt_n']}")

                if _survey_capture_active:
                    survey_msg = _process_survey_sweep(result)
                    if survey_msg:
                        await broadcast(json.dumps(survey_msg))

            if calib_mgr and calib_mgr.state == CalibrationManager.COLLECTING:
                for cal_msg in calib_mgr.feed_line(line):
                    await broadcast(json.dumps(cal_msg))
        except Exception as e:
            print(f"[SER] Error: {e}")
            await asyncio.sleep(0.05)


async def main(args):
    global pipeline, log_fh, cfg_global, calib_mgr

    anchors_path = CONFIG_DIR / 'anchors.json'
    if anchors_path.exists():
        cfg_global = AnchorConfig.from_json(str(anchors_path))
        print(f"[CFG] Loaded {len(cfg_global.ids)} anchors from {anchors_path}")
    else:
        cfg_global = AnchorConfig()
        print("[CFG] Using built-in 4-anchor example (no anchors.json found)")

    pipeline = RtlsPipeline(cfg_global)

    calib_mgr = CalibrationManager(cfg_global, CONFIG_DIR / 'calibration.json')
    if calib_mgr.try_load():
        pipeline.apply_t1_calibration(calib_mgr.per_anchor_bias_m)
        print("[CAL] T1 calibration applied to pipeline")

    log_name = SCRIPT_DIR / f"rtls_log_{datetime.now():%Y%m%d_%H%M%S}.jsonl"
    log_fh = open(log_name, 'w')
    print(f"[LOG] {log_name}")

    print(f"[WS]  WebSocket server on ws://localhost:{args.ws_port}")
    print(f"      Open gui.html in your browser\n")

    async with websockets.serve(ws_handler, '0.0.0.0', args.ws_port):
        if args.serial:
            await serial_reader(args.serial, args.baud)
        else:
            await udp_reader(args.udp_port)


def parse_args():
    p = argparse.ArgumentParser(description='UWB RTLS Python host')
    p.add_argument('--udp-port', type=int, default=DEFAULT_UDP_PORT)
    p.add_argument('--ws-port',  type=int, default=DEFAULT_WS_PORT)
    p.add_argument('--serial',   type=str, default=None,
                   help='Serial port for RTLS data (e.g. /dev/ttyUSB0)')
    p.add_argument('--baud',     type=int, default=DEFAULT_BAUD)
    return p.parse_args()


if __name__ == '__main__':
    args = parse_args()
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if log_fh:
            log_fh.close()
