"""
calibration_manager.py — Tier 1 per-anchor bias calibration for UWB RTLS.

Place the tag at a known point (auto-computed as circumcenter of the anchor
triangle), collect N raw RTLS sweeps, compute per-anchor mean bias, save to
calibration.json v2.

States:  IDLE -> COLLECTING -> REVIEW -> IDLE (after save or discard)
"""

import json, hashlib
import numpy as np
from pathlib import Path
from datetime import datetime
from collections import defaultdict

# Importable from run_localization.py context where sys.path already includes
# the python/ directory containing the rtls package.
from rtls import FrameParser, AnchorConfig


def _circumcenter_2d(pts: np.ndarray):
    """Return circumcenter of the first 3 rows of pts (Nx2). Returns None if degenerate."""
    if len(pts) < 3:
        return None
    (ax, ay), (bx, by), (cx, cy) = pts[0], pts[1], pts[2]
    D = 2 * (ax*(by - cy) + bx*(cy - ay) + cx*(ay - by))
    if abs(D) < 1e-9:
        return None
    ux = ((ax**2 + ay**2)*(by - cy) + (bx**2 + by**2)*(cy - ay) + (cx**2 + cy**2)*(ay - by)) / D
    uy = ((ax**2 + ay**2)*(cx - bx) + (bx**2 + by**2)*(ax - cx) + (cx**2 + cy**2)*(bx - ax)) / D
    return np.array([ux, uy])


class CalibrationManager:
    """Per-anchor constant bias calibration (Tier 1)."""

    IDLE       = 'IDLE'
    COLLECTING = 'COLLECTING'
    REVIEW     = 'REVIEW'

    def __init__(self, cfg: AnchorConfig, cal_path: Path):
        self.cfg      = cfg
        self.cal_path = cal_path
        self.state    = self.IDLE

        self._cal_point: np.ndarray | None = None
        self._true_dists_m: dict[int, float] = {}

        self._n_target   = 500
        self._samples: dict[int, list[float]] = defaultdict(list)
        self._last_progress_n = 0

        # Computed or loaded biases: {aid: {bias_mm, bias_std_mm, n_samples}}
        self._biases: dict[int, dict] = {}

        # Applied biases in metres (subtracted from d_raw in pipeline)
        self.per_anchor_bias_m: dict[int, float] = {}
        self.t1_loaded = False

        self._compute_cal_point()

    # ── Calibration point ─────────────────────────────────────────────────────

    def _compute_cal_point(self):
        coords2d = self.cfg.coords[:, :2]
        n = len(coords2d)
        if n == 3:
            cc = _circumcenter_2d(coords2d)
            if cc is not None:
                self._cal_point = cc
        elif n >= 4:
            # Centroid is equidistant from all corners for any regular polygon
            self._cal_point = np.mean(coords2d, axis=0)
        if self._cal_point is not None:
            self._recompute_true_dists()

    def _recompute_true_dists(self):
        if self._cal_point is None:
            return
        coords2d = self.cfg.coords[:, :2]
        for i, aid in enumerate(self.cfg.ids):
            d = float(np.linalg.norm(coords2d[i] - self._cal_point))
            self._true_dists_m[int(aid)] = d

    def set_calibration_point(self, point_m: list):
        """Override the auto-computed circumcenter."""
        self._cal_point = np.array(point_m[:2], dtype=float)
        self._recompute_true_dists()

    @property
    def cal_point_m(self):
        if self._cal_point is None:
            return None
        # `or 0.0` eliminates -0.0 from floating-point circumcenter arithmetic
        return [(round(float(v), 4) or 0.0) for v in self._cal_point]

    @property
    def true_dists_m(self) -> dict:
        return self._true_dists_m

    # ── Startup load ──────────────────────────────────────────────────────────

    def try_load(self) -> bool:
        """Load calibration.json v2 on startup. Returns True if T1 biases were loaded."""
        if not self.cal_path.exists():
            return False
        try:
            data = json.loads(self.cal_path.read_text())
        except Exception as e:
            print(f"[CAL] Failed to parse calibration.json: {e}")
            return False

        if data.get('version') != 2:
            print(f"[CAL] calibration.json is v{data.get('version')}, not v2 — ignoring")
            return False

        t1 = data.get('tier1', {})
        if not t1 or 'anchors' not in t1:
            return False

        biases = {}
        for aid_str, vals in t1['anchors'].items():
            try:
                biases[int(aid_str)] = vals
            except ValueError:
                pass

        if not biases:
            return False

        self._biases = biases
        self.per_anchor_bias_m = {aid: v['bias_mm'] / 1000.0 for aid, v in biases.items()}
        self.t1_loaded = True

        summary = ', '.join(f"A{k}={v['bias_mm']:+.1f}mm" for k, v in sorted(biases.items()))
        print(f"[CAL] Loaded T1: {summary}")
        return True

    # ── Collection state machine ───────────────────────────────────────────────

    def start_collection(self, n_samples: int = 500) -> dict:
        if self.state == self.COLLECTING:
            return {'type': 'CALIB_T1_ERROR', 'msg': 'Already collecting'}
        self._n_target = max(50, n_samples)
        self._samples = defaultdict(list)
        self._last_progress_n = 0
        self.state = self.COLLECTING
        print(f"[CAL] T1 collecting {self._n_target} samples at {self.cal_point_m}")
        return {
            'type': 'CALIB_T1_STARTED',
            'n_target': self._n_target,
            'cal_point_m': self.cal_point_m,
            'true_dists_m': {str(k): round(v, 4) for k, v in self._true_dists_m.items()},
        }

    def stop_collection(self) -> dict:
        self.state = self.IDLE
        self._samples = defaultdict(list)
        self._biases = {}  # clear partial/review biases; per_anchor_bias_m (active) unchanged
        print("[CAL] T1 collection discarded")
        return {'type': 'CALIB_T1_STOPPED'}

    def feed_line(self, line: str) -> list[dict]:
        """Parse a raw RTLS line and accumulate per-anchor samples.

        Returns a list of WS messages to broadcast (empty if not collecting).
        Progress is throttled to every 10 samples to avoid flooding the browser.
        """
        if self.state != self.COLLECTING:
            return []

        pkt = FrameParser.parse(line)
        if not pkt.valid:
            return []

        for i, aid in enumerate(pkt.ids):
            aid_i = int(aid)
            if aid_i in self._true_dists_m:
                self._samples[aid_i].append(float(pkt.dist[i]))

        n_min = min((len(v) for v in self._samples.values()), default=0)

        # Finished?
        if n_min >= self._n_target:
            return self._finish_collection()

        # Throttled progress update (every 10 samples or on the last one before done)
        if n_min - self._last_progress_n < 10:
            return []
        self._last_progress_n = n_min

        return [self._build_progress_msg(n_min)]

    def _build_progress_msg(self, n_min: int) -> dict:
        anchors = {}
        for aid, samps in self._samples.items():
            if not samps:
                continue
            mean_m = float(np.mean(samps))
            std_m  = float(np.std(samps)) if len(samps) > 1 else 0.0
            live_bias_mm = (mean_m - self._true_dists_m.get(aid, mean_m)) * 1000.0
            anchors[str(aid)] = {
                'n':       len(samps),
                'bias_mm': round(live_bias_mm, 1),
                'std_mm':  round(std_m * 1000.0, 1),
            }
        return {
            'type':     'CALIB_T1_PROGRESS',
            'n':        n_min,
            'n_target': self._n_target,
            'anchors':  anchors,
        }

    def _finish_collection(self) -> list[dict]:
        self.state = self.REVIEW
        biases = {}
        for aid, samps in self._samples.items():
            if not samps or aid not in self._true_dists_m:
                continue
            mean_m   = float(np.mean(samps))
            std_m    = float(np.std(samps))
            bias_mm  = (mean_m - self._true_dists_m[aid]) * 1000.0
            biases[aid] = {
                'bias_mm':     round(bias_mm, 2),
                'bias_std_mm': round(std_m * 1000.0, 2),
                'n_samples':   len(samps),
            }
        self._biases = biases
        summary = ', '.join(
            f"A{k}={v['bias_mm']:+.1f}±{v['bias_std_mm']:.1f}mm"
            for k, v in sorted(biases.items())
        )
        print(f"[CAL] T1 done: {summary}")
        return [{
            'type':        'CALIB_T1_DONE',
            'anchors':     {str(k): v for k, v in biases.items()},
            'cal_point_m': self.cal_point_m,
        }]

    # ── Apply & save ──────────────────────────────────────────────────────────

    def apply_and_save(self) -> dict:
        """Commit the reviewed biases to per_anchor_bias_m and write calibration.json v2."""
        if self.state != self.REVIEW or not self._biases:
            return {'type': 'CALIB_T1_ERROR', 'msg': 'No calibration data to save'}

        self.per_anchor_bias_m = {aid: v['bias_mm'] / 1000.0 for aid, v in self._biases.items()}
        self.t1_loaded = True

        try:
            self._write_json()
            self.state = self.IDLE
            print(f"[CAL] T1 saved → {self.cal_path}")
            return {
                'type':    'CALIB_T1_SAVED',
                'anchors': {str(k): v for k, v in self._biases.items()},
                'path':    str(self.cal_path),
            }
        except Exception as e:
            return {'type': 'CALIB_T1_ERROR', 'msg': f'Save failed: {e}'}

    def _write_json(self):
        # Traceability hash of the anchor config
        anchors_path = self.cal_path.parent / 'anchors.json'
        anchor_hash = ''
        if anchors_path.exists():
            anchor_hash = hashlib.sha256(anchors_path.read_bytes()).hexdigest()[:16]

        # Preserve existing tier2 block if present
        existing_t2 = None
        if self.cal_path.exists():
            try:
                existing = json.loads(self.cal_path.read_text())
                if existing.get('version') == 2:
                    existing_t2 = existing.get('tier2')
            except Exception:
                pass

        n_samples = next(iter(self._biases.values()), {}).get('n_samples', 0)
        doc: dict = {
            'version':             2,
            'schema':              'uwb_rtls_calibration_v2',
            'created_at':          datetime.now().isoformat(timespec='seconds'),
            'anchor_config_hash':  anchor_hash,
            'tier1': {
                'method':               'single_point_mean_bias',
                'calibration_point_m':  self.cal_point_m,
                'calibrated_at':        datetime.now().isoformat(timespec='seconds'),
                'n_samples':            n_samples,
                'anchors':              {str(k): v for k, v in sorted(self._biases.items())},
            },
        }
        if existing_t2 is not None:
            doc['tier2'] = existing_t2

        self.cal_path.write_text(json.dumps(doc, indent=2))

    def on_anchors_updated(self, cfg: AnchorConfig):
        """Call after a survey changes anchor positions.

        Recomputes circumcenter for the new geometry and invalidates any loaded
        T1 calibration (biases were computed for the old positions).
        """
        self.cfg = cfg
        self._compute_cal_point()
        self.t1_loaded = False
        self.per_anchor_bias_m = {}
        self._biases = {}
        self.state = self.IDLE
        self._samples = defaultdict(list)
        print("[CAL] Anchor positions updated — T1 calibration invalidated, recalibrate")

    # ── Status helpers ────────────────────────────────────────────────────────

    def status_msg(self) -> dict:
        """Compact status dict included in the WS 'config' message on browser connect."""
        return {
            't1_loaded':    self.t1_loaded,
            't1_state':     self.state,
            'cal_point_m':  self.cal_point_m,
            'biases':       {str(k): v for k, v in sorted(self._biases.items())} if self._biases else {},
        }
