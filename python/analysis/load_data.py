"""
load_data.py — Data loading and cleaning utilities for UWB bias analysis.

All three CSV files are in v1 format (fp_power_dbm and quality empty).
"""

import csv
import math
import numpy as np
from pathlib import Path

DATA_DIR = Path(__file__).resolve().parent.parent

FILES = {
    'A2_run1':  ('bias_anchor2_20260619_124326.csv', 2,
                 'Anchor 2, calibrated previous evening, run next day'),
    'A1_fresh': ('bias_anchor1_20260619_134320.csv', 1,
                 'Anchor 1, freshly calibrated at 3.0 m, immediate sweep'),
    'A2_recal': ('bias_anchor2_20260619_143553.csv', 2,
                 'Anchor 2, recalibrated at 3.0 m, sweep with battery interruption'),
}

DISTANCES_CM = [5, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600]
IQR_K = 1.5          # Tukey IQR multiplier for outlier rejection
SENTINEL_RX = -1000  # rx_power values below this are hardware sentinel / invalid


def _parse_float(s, sentinel=None):
    try:
        v = float(s)
        return v
    except (ValueError, TypeError):
        return sentinel


def load_file(key):
    """
    Load and clean one dataset.

    Returns a dict keyed by distance_cm, each value is a dict:
      {
        'sessions': {session_id: {'d_mm': np.array, 'rx': np.array,
                                   'bias_mm': np.array, 't_ms': np.array,
                                   'wall_time': list}},
        'label': str,
        'anchor_id': int,
        'desc': str,
      }
    """
    fname, anchor_id, desc = FILES[key]
    fpath = DATA_DIR / fname

    raw = []
    with open(fpath, newline='', encoding='utf-8') as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            d_mm     = _parse_float(row.get('d_mm'))
            rx       = _parse_float(row.get('rx_power_dbm'))
            true_m   = _parse_float(row.get('true_distance_m'))
            dist_cm  = _parse_float(row.get('distance_cm'))
            session  = _parse_float(row.get('session'))
            t_ms     = _parse_float(row.get('t_ms'))
            wall     = row.get('wall_time_iso', '')

            # Skip bad rows
            if any(v is None for v in [d_mm, rx, true_m, dist_cm, session]):
                continue
            if rx < SENTINEL_RX:
                continue
            if not math.isfinite(d_mm) or not math.isfinite(rx):
                continue

            raw.append({
                'd_mm': d_mm,
                'rx': rx,
                'true_m': true_m,
                'dist_cm': int(round(dist_cm)),
                'session': int(round(session)),
                't_ms': t_ms if t_ms is not None else float('nan'),
                'wall': wall,
            })

    # Organise by distance → session
    by_dist = {}
    for r in raw:
        dc = r['dist_cm']
        s  = r['session']
        if dc not in by_dist:
            by_dist[dc] = {}
        if s not in by_dist[dc]:
            by_dist[dc][s] = {'d_mm': [], 'rx': [], 't_ms': [], 'wall': []}
        by_dist[dc][s]['d_mm'].append(r['d_mm'])
        by_dist[dc][s]['rx'].append(r['rx'])
        by_dist[dc][s]['t_ms'].append(r['t_ms'])
        by_dist[dc][s]['wall'].append(r['wall'])

    # Convert to numpy, add true_mm and bias_mm, pre-compute IQR-filtered stats
    result = {'label': key, 'anchor_id': anchor_id, 'desc': desc,
              'distances': {}}

    for dc, sess_dict in by_dist.items():
        true_mm = dc * 10.0   # cm → mm
        result['distances'][dc] = {
            'true_mm': true_mm,
            'sessions': {},
        }
        for s, arrays in sess_dict.items():
            d  = np.array(arrays['d_mm'], dtype=float)
            rx = np.array(arrays['rx'],   dtype=float)
            tm = np.array(arrays['t_ms'], dtype=float)

            # IQR filtering
            q1, q3 = np.percentile(d, 25), np.percentile(d, 75)
            iqr = q3 - q1
            lo, hi = q1 - IQR_K * iqr, q3 + IQR_K * iqr
            mask = (d >= lo) & (d <= hi)
            n_out = int((~mask).sum())

            result['distances'][dc]['sessions'][s] = {
                'd_raw': d,
                'rx_raw': rx,
                't_ms': tm,
                'wall': arrays['wall'],
                'd': d[mask],
                'rx': rx[mask],
                'bias_mm': d[mask] - true_mm,
                'mask': mask,
                'n_out': n_out,
                'n_raw': len(d),
                'n_filt': int(mask.sum()),
                'outlier_frac': n_out / max(len(d), 1),
            }

    return result


def pooled_stats(dataset, dist_cm):
    """
    Pool all sessions for a given distance and return robust statistics.
    """
    dist_data = dataset['distances'].get(dist_cm)
    if dist_data is None:
        return None
    true_mm = dist_data['true_mm']
    all_d, all_rx, all_bias = [], [], []
    for s_data in dist_data['sessions'].values():
        all_d.extend(s_data['d'].tolist())
        all_rx.extend(s_data['rx'].tolist())
        all_bias.extend(s_data['bias_mm'].tolist())
    d  = np.array(all_d)
    rx = np.array(all_rx)
    b  = np.array(all_bias)
    if len(d) == 0:
        return None
    return {
        'true_mm': true_mm,
        'n': len(d),
        'mean_d': float(d.mean()),
        'mean_bias': float(b.mean()),
        'std': float(d.std()),
        'median_bias': float(np.median(b)),
        'p05': float(np.percentile(b, 5)),
        'p10': float(np.percentile(b, 10)),
        'p25': float(np.percentile(b, 25)),
        'p75': float(np.percentile(b, 75)),
        'p90': float(np.percentile(b, 90)),
        'p95': float(np.percentile(b, 95)),
        'iqr': float(np.percentile(d, 75) - np.percentile(d, 25)),
        'mean_rx': float(rx.mean()),
        'std_rx': float(rx.std()),
        'd_arr': d,
        'rx_arr': rx,
        'b_arr': b,
    }


def load_all():
    datasets = {}
    for key in FILES:
        datasets[key] = load_file(key)
    return datasets
