#!/usr/bin/env python3
"""
analyze.py — Research-grade UWB bias characterisation analysis.

Generates all figures for the LaTeX report.

Usage:
    python analysis/analyze.py
"""

import warnings, sys, math
from pathlib import Path
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from scipy import stats as spstats
from scipy.interpolate import CubicSpline
from scipy.optimize import curve_fit

# ── Paths ──────────────────────────────────────────────────────────────────────
ANALYSIS_DIR = Path(__file__).resolve().parent
DATA_DIR     = ANALYSIS_DIR.parent
FIG_DIR      = ANALYSIS_DIR / 'figures'
FIG_DIR.mkdir(exist_ok=True)

FILES = {
    'A2_run1':  DATA_DIR / 'bias_anchor2_20260619_124326.csv',
    'A1_fresh': DATA_DIR / 'bias_anchor1_20260619_134320.csv',
    'A2_recal': DATA_DIR / 'bias_anchor2_20260619_143553.csv',
}

LABELS = {
    'A2_run1':  'Anchor 2 (old cal.)',
    'A1_fresh': 'Anchor 1 (fresh cal.)',
    'A2_recal': 'Anchor 2 (recal. 3 m)',
}

COLORS = {
    'A2_run1':  '#d62728',   # red
    'A1_fresh': '#1f77b4',   # blue
    'A2_recal': '#2ca02c',   # green
}

MARKERS = {
    'A2_run1':  'D',
    'A1_fresh': 's',
    'A2_recal': 'o',
}

DISTS = [5, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600]

# ── Publication style ─────────────────────────────────────────────────────────
plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 10,
    'axes.titlesize': 11,
    'axes.labelsize': 10,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'legend.fontsize': 9,
    'figure.dpi': 150,
    'axes.spines.top': False,
    'axes.spines.right': False,
    'axes.grid': True,
    'grid.alpha': 0.35,
    'grid.linestyle': '--',
})


# ── Data loading ──────────────────────────────────────────────────────────────
def load(key):
    df = pd.read_csv(FILES[key], on_bad_lines='skip')
    for c in ['d_mm','rx_power_dbm','true_distance_m','distance_cm','session','t_ms']:
        df[c] = pd.to_numeric(df[c], errors='coerce')
    df = df.dropna(subset=['d_mm','rx_power_dbm','true_distance_m','distance_cm','session'])
    df = df[df['rx_power_dbm'] > -1000].copy()
    df['true_mm'] = df['true_distance_m'] * 1000.0
    df['bias_mm'] = df['d_mm'] - df['true_mm']
    return df

def iqr_filter(arr, k=1.5):
    q1, q3 = np.percentile(arr, [25, 75])
    iqr = q3 - q1
    m = (arr >= q1 - k*iqr) & (arr <= q3 + k*iqr)
    return arr[m], m

def dist_stats(df, dc, k=1.5):
    sub = df[df['distance_cm'] == dc]
    if len(sub) == 0:
        return None
    true_mm = float(sub['true_mm'].iloc[0])
    d_filt, m = iqr_filter(sub['d_mm'].values, k)
    b_filt = d_filt - true_mm
    rx_filt, _ = iqr_filter(sub['rx_power_dbm'].values, k)
    n_neg = int((sub['d_mm'] < 0).sum())
    return dict(
        true_mm=true_mm, n_raw=len(sub), n=len(d_filt),
        mean=float(d_filt.mean()), bias=float(b_filt.mean()),
        std=float(d_filt.std()), median_bias=float(np.median(b_filt)),
        p10=float(np.percentile(b_filt, 10)), p25=float(np.percentile(b_filt, 25)),
        p75=float(np.percentile(b_filt, 75)), p90=float(np.percentile(b_filt, 90)),
        iqr_b=float(np.percentile(b_filt,75)-np.percentile(b_filt,25)),
        rx_mean=float(rx_filt.mean()), rx_std=float(rx_filt.std()),
        outlier_pct=float(100*(1-len(d_filt)/len(sub))),
        n_neg=n_neg,
        d_arr=d_filt, b_arr=b_filt, rx_arr=rx_filt,
    )

print("Loading datasets...")
dfs = {k: load(k) for k in FILES}

# Pre-compute stats for all datasets × distances
stats = {}
for key, df in dfs.items():
    stats[key] = {}
    for dc in DISTS:
        s = dist_stats(df, dc)
        if s:
            stats[key][dc] = s

print("Data loaded. Computing figures...")


# ── Helper: save figure ───────────────────────────────────────────────────────
def savefig(fig, name, **kw):
    p = FIG_DIR / name
    fig.savefig(p, bbox_inches='tight', dpi=150, **kw)
    plt.close(fig)
    print(f"  Saved {name}")


# ════════════════════════════════════════════════════════════════════════════════
# FIGURE 1 — Bias vs Distance, all three datasets (main result)
# ════════════════════════════════════════════════════════════════════════════════
def fig_bias_vs_distance():
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.5, 8),
                                    gridspec_kw={'height_ratios': [3, 2]})

    for key in ['A2_run1', 'A1_fresh', 'A2_recal']:
        dc_list, bias_list, err_lo, err_hi = [], [], [], []
        for dc in DISTS:
            s = stats[key].get(dc)
            if s is None: continue
            dc_list.append(dc)
            bias_list.append(s['bias'])
            err_lo.append(s['bias'] - s['p10'])
            err_hi.append(s['p90'] - s['bias'])

        dc_arr = np.array(dc_list)
        b_arr  = np.array(bias_list)
        ax1.errorbar(dc_arr/10.0, b_arr,
                     yerr=[err_lo, err_hi],
                     color=COLORS[key], marker=MARKERS[key], ms=5,
                     lw=1.4, capsize=3, label=LABELS[key], zorder=3)

    ax1.axhline(0, color='k', lw=0.8, ls='--', alpha=0.6)
    ax1.axhspan(-10, 10, alpha=0.08, color='green', label='±10 mm band')
    ax1.set_xlabel('True distance (m)')
    ax1.set_ylabel('Ranging bias (mm)\n[measured − true]')
    ax1.set_title('Ranging Bias vs. True Distance — All Datasets\n'
                  'Error bars = P10–P90 range after IQR outlier rejection')
    ax1.legend(framealpha=0.9)
    ax1.set_xlim(-0.05, 6.3)

    # Lower panel: A1_fresh and A2_recal only (zoomed)
    for key in ['A1_fresh', 'A2_recal']:
        dc_list, bias_list = [], []
        std_list = []
        for dc in DISTS:
            s = stats[key].get(dc)
            if s is None: continue
            dc_list.append(dc)
            bias_list.append(s['bias'])
            std_list.append(s['std'])
        dc_arr = np.array(dc_list)
        b_arr  = np.array(bias_list)
        st_arr = np.array(std_list)
        ax2.fill_between(dc_arr/10.0, b_arr - st_arr, b_arr + st_arr,
                         alpha=0.15, color=COLORS[key])
        ax2.plot(dc_arr/10.0, b_arr, color=COLORS[key], marker=MARKERS[key],
                 ms=5, lw=1.4, label=LABELS[key])

    ax2.axhline(0, color='k', lw=0.8, ls='--', alpha=0.6)
    ax2.axhspan(-10, 10, alpha=0.08, color='green')
    ax2.set_xlabel('True distance (m)')
    ax2.set_ylabel('Ranging bias (mm)')
    ax2.set_title('Zoomed: Properly Calibrated Anchors Only\n'
                  'Shaded band = ±1σ measurement noise')
    ax2.legend(framealpha=0.9)
    ax2.set_xlim(-0.05, 6.3)
    ax2.set_ylim(-160, 200)

    fig.tight_layout()
    savefig(fig, 'fig01_bias_vs_distance.pdf')

fig_bias_vs_distance()


# ════════════════════════════════════════════════════════════════════════════════
# FIGURE 2 — A2: Old calibration vs New recalibration
# ════════════════════════════════════════════════════════════════════════════════
def fig_calibration_comparison():
    fig, axes = plt.subplots(2, 1, figsize=(7.5, 8))
    ax1, ax2 = axes

    # Panel 1: Raw bias values
    for key in ['A2_run1', 'A2_recal']:
        dc_list, bias_list, std_list = [], [], []
        for dc in DISTS:
            s = stats[key].get(dc)
            if s is None: continue
            dc_list.append(dc)
            bias_list.append(s['bias'])
            std_list.append(s['std'])
        dc_arr = np.array(dc_list) / 10.0
        b_arr  = np.array(bias_list)
        ax1.fill_between(dc_arr, b_arr-std_list, b_arr+std_list,
                         alpha=0.12, color=COLORS[key])
        ax1.plot(dc_arr, b_arr, color=COLORS[key], marker=MARKERS[key],
                 ms=5, lw=1.4, label=LABELS[key])

    ax1.axhline(0, color='k', lw=0.8, ls='--', alpha=0.6)
    ax1.set_title('Anchor 2: Effect of Calibration on Ranging Bias')
    ax1.set_xlabel('True distance (m)')
    ax1.set_ylabel('Ranging bias (mm)')
    ax1.legend()

    # Panel 2: Delta (A2_run1 - A2_recal)
    dc_common, delta_list = [], []
    for dc in DISTS:
        s1 = stats['A2_run1'].get(dc)
        s2 = stats['A2_recal'].get(dc)
        if s1 and s2:
            dc_common.append(dc/10.0)
            delta_list.append(s1['bias'] - s2['bias'])

    ax2.bar(dc_common, delta_list, width=0.04,
            color='#9467bd', alpha=0.75, label='A2_run1 − A2_recal')
    ax2.axhline(np.mean(delta_list), color='k', ls='--', lw=1,
                label=f'Mean offset = {np.mean(delta_list):.0f} mm')
    ax2.set_title('Bias Difference: Old Calibration − New Calibration\n'
                  f'(consistent ~{np.mean(delta_list):.0f} mm shift = calibration error)')
    ax2.set_xlabel('True distance (m)')
    ax2.set_ylabel('Bias difference (mm)')
    ax2.legend()

    fig.tight_layout()
    savefig(fig, 'fig02_calibration_comparison.pdf')

fig_calibration_comparison()


# ════════════════════════════════════════════════════════════════════════════════
# FIGURE 3 — Anchor-to-anchor comparison (A1 vs A2_recal)
# ════════════════════════════════════════════════════════════════════════════════
def fig_anchor_comparison():
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.5, 8))

    for key in ['A1_fresh', 'A2_recal']:
        dc_list, bias_list, std_list = [], [], []
        for dc in DISTS:
            s = stats[key].get(dc)
            if s is None: continue
            dc_list.append(dc)
            bias_list.append(s['bias'])
            std_list.append(s['std'])
        dc_arr = np.array(dc_list) / 10.0
        b_arr  = np.array(bias_list)
        ax1.fill_between(dc_arr, np.array(b_arr)-std_list, np.array(b_arr)+std_list,
                         alpha=0.12, color=COLORS[key])
        ax1.plot(dc_arr, b_arr, color=COLORS[key], marker=MARKERS[key],
                 ms=5, lw=1.4, label=LABELS[key])

    ax1.axhline(0, color='k', lw=0.8, ls='--', alpha=0.6)
    ax1.axhspan(-10, 10, alpha=0.08, color='green')
    ax1.set_title('Anchor-to-Anchor Bias Comparison\n'
                  'Both anchors freshly calibrated at 3.0 m; different rooms / positions')
    ax1.set_xlabel('True distance (m)')
    ax1.set_ylabel('Ranging bias (mm)')
    ax1.legend()

    # Difference
    dc_common, delta_list = [], []
    for dc in DISTS:
        s1 = stats['A1_fresh'].get(dc)
        s2 = stats['A2_recal'].get(dc)
        if s1 and s2:
            dc_common.append(dc/10.0)
            delta_list.append(s1['bias'] - s2['bias'])

    delta_arr = np.array(delta_list)
    colors_d = ['#d62728' if v < 0 else '#2ca02c' for v in delta_list]
    ax2.bar(dc_common, delta_list, width=0.04, color=colors_d, alpha=0.75)
    ax2.axhline(0, color='k', lw=0.8, ls='--', alpha=0.6)
    ax2.axhline(np.mean(delta_list), color='purple', ls='--', lw=1,
                label=f'Mean = {np.mean(delta_list):.0f} mm')
    ax2.set_title(f'Anchor 1 − Anchor 2 Bias Difference\n'
                  f'RMS difference = {np.sqrt(np.mean(delta_arr**2)):.0f} mm  '
                  f'Range = [{delta_arr.min():.0f}, {delta_arr.max():.0f}] mm')
    ax2.set_xlabel('True distance (m)')
    ax2.set_ylabel('Bias difference (mm)')
    ax2.legend()

    fig.tight_layout()
    savefig(fig, 'fig03_anchor_comparison.pdf')

fig_anchor_comparison()


# ════════════════════════════════════════════════════════════════════════════════
# FIGURE 4 — Session repeatability (all three datasets)
# ════════════════════════════════════════════════════════════════════════════════
def fig_repeatability():
    fig, axes = plt.subplots(1, 3, figsize=(12, 4.5), sharey=False)

    for ax, key in zip(axes, ['A2_run1', 'A1_fresh', 'A2_recal']):
        df = dfs[key]
        dc_list, delta_list = [], []
        for dc in DISTS:
            sessions = sorted(df[df['distance_cm']==dc]['session'].unique())
            if len(sessions) < 2:
                continue
            true_mm = dc * 10.0
            biases = []
            for s in sessions:
                sub = df[(df['distance_cm']==dc) & (df['session']==s)]
                d_f, _ = iqr_filter(sub['d_mm'].values)
                biases.append(float(d_f.mean() - true_mm))
            dc_list.append(dc/10.0)
            delta_list.append(biases[-1] - biases[0])

        delta_arr = np.array(delta_list)
        bar_colors = ['#d62728' if v < 0 else '#2ca02c' for v in delta_list]
        ax.bar(dc_list, delta_list, width=0.04, color=bar_colors, alpha=0.75)
        ax.axhline(0, color='k', lw=0.8, ls='--', alpha=0.6)
        ax.set_title(f'{LABELS[key]}\nRMS Δ = {np.sqrt(np.mean(delta_arr**2)):.1f} mm')
        ax.set_xlabel('True distance (m)')
        ax.set_ylabel('Session 2 − Session 1 bias (mm)')
        ax.set_ylim(-40, 40)

    fig.suptitle('Session Repeatability: Bias Difference Between Sessions at Same Distance',
                 fontsize=11, y=1.02)
    fig.tight_layout()
    savefig(fig, 'fig04_repeatability.pdf')

fig_repeatability()


# ════════════════════════════════════════════════════════════════════════════════
# FIGURE 5 — Noise (STD) vs Distance
# ════════════════════════════════════════════════════════════════════════════════
def fig_noise_vs_distance():
    fig, ax = plt.subplots(figsize=(7, 4.5))

    for key in ['A2_run1', 'A1_fresh', 'A2_recal']:
        dc_list, std_list = [], []
        for dc in DISTS:
            s = stats[key].get(dc)
            if s is None: continue
            dc_list.append(dc/10.0)
            std_list.append(s['std'])
        ax.plot(dc_list, std_list, color=COLORS[key], marker=MARKERS[key],
                ms=5, lw=1.4, label=LABELS[key])

    # Fit power-law σ(d) = a·d^b to A2_recal
    dc_a2 = np.array([dc/10.0 for dc in DISTS if stats['A2_recal'].get(dc)])
    st_a2 = np.array([stats['A2_recal'][dc]['std'] for dc in DISTS if stats['A2_recal'].get(dc)])
    try:
        popt, _ = curve_fit(lambda d, a, b: a * d**b, dc_a2, st_a2, p0=[10, 0.3])
        d_fit = np.linspace(0.05, 6.1, 200)
        ax.plot(d_fit, popt[0]*d_fit**popt[1], 'k:', lw=1.2, alpha=0.7,
                label=f'Power law fit: {popt[0]:.1f}·d^{{{popt[1]:.2f}}}')
    except Exception:
        pass

    ax.set_xlabel('True distance (m)')
    ax.set_ylabel('Ranging noise σ (mm, IQR-filtered std)')
    ax.set_title('Measurement Noise vs. Distance\nAll datasets show noise growth at long range')
    ax.legend()
    ax.set_xlim(-0.05, 6.3)
    ax.set_ylim(0, 60)
    savefig(fig, 'fig05_noise_vs_distance.pdf')

fig_noise_vs_distance()


# ════════════════════════════════════════════════════════════════════════════════
# FIGURE 6 — RX power vs Distance (path loss)
# ════════════════════════════════════════════════════════════════════════════════
def fig_rx_power():
    fig, ax = plt.subplots(figsize=(7, 4.5))

    for key in ['A2_run1', 'A1_fresh', 'A2_recal']:
        dc_list, rx_list = [], []
        for dc in DISTS:
            s = stats[key].get(dc)
            if s is None: continue
            dc_list.append(dc/10.0)
            rx_list.append(s['rx_mean'])
        ax.plot(dc_list, rx_list, color=COLORS[key], marker=MARKERS[key],
                ms=5, lw=1.4, label=LABELS[key])

    # Theoretical free-space path loss: P(d) = P0 - 20·log10(d/d0)
    dc_a2 = np.array([dc/10.0 for dc in DISTS if stats['A2_recal'].get(dc)])
    rx_a2 = np.array([stats['A2_recal'][dc]['rx_mean'] for dc in DISTS if stats['A2_recal'].get(dc)])
    p0 = rx_a2[2]  # reference at 100cm
    d0 = 1.0
    d_th = np.linspace(0.1, 6.5, 200)
    P_fs = p0 - 20*np.log10(d_th/d0)
    ax.plot(d_th, P_fs, 'k--', lw=1, alpha=0.5, label='Free-space (20 dB/decade)')

    ax.set_xlabel('True distance (m)')
    ax.set_ylabel('Mean RX power (dBm)')
    ax.set_title('Received Power vs. Distance\nCompared to free-space path loss model')
    ax.legend()
    ax.set_xlim(-0.05, 6.3)
    savefig(fig, 'fig06_rx_power.pdf')

fig_rx_power()


# ════════════════════════════════════════════════════════════════════════════════
# FIGURE 7 — Bias vs RX power (power-dependent correction analysis)
# ════════════════════════════════════════════════════════════════════════════════
def fig_bias_vs_rxpower():
    fig, axes = plt.subplots(1, 3, figsize=(12, 4.5))

    for ax, key in zip(axes, ['A2_run1', 'A1_fresh', 'A2_recal']):
        bias_list, rx_list = [], []
        for dc in DISTS:
            s = stats[key].get(dc)
            if s is None: continue
            bias_list.append(s['bias'])
            rx_list.append(s['rx_mean'])

        b  = np.array(bias_list)
        rx = np.array(rx_list)
        r, pval = spstats.pearsonr(b, rx)

        ax.scatter(rx, b, color=COLORS[key], s=50, zorder=3)
        for dc, bi, ri in zip(DISTS, bias_list, rx_list):
            ax.annotate(f'{dc}', (ri, bi), textcoords='offset points',
                        xytext=(4, 2), fontsize=7, alpha=0.7)

        # Regression line
        m, c = np.polyfit(rx, b, 1)
        x_fit = np.linspace(rx.min()-1, rx.max()+1, 100)
        ax.plot(x_fit, m*x_fit + c, '--', color=COLORS[key], lw=1, alpha=0.7)

        ax.set_title(f'{LABELS[key]}\nPearson r = {r:.3f}  p = {pval:.3f}')
        ax.set_xlabel('Mean RX power (dBm)')
        ax.set_ylabel('Ranging bias (mm)')

    fig.suptitle('Bias vs. Received Power — Power-based Correction Analysis\n'
                 'Labels = true distance (cm)', fontsize=11)
    fig.tight_layout()
    savefig(fig, 'fig07_bias_vs_rxpower.pdf')

fig_bias_vs_rxpower()


# ════════════════════════════════════════════════════════════════════════════════
# FIGURE 8 — Box plots per distance (A1_fresh & A2_recal combined)
# ════════════════════════════════════════════════════════════════════════════════
def fig_boxplots():
    fig, axes = plt.subplots(2, 1, figsize=(10, 8))

    for ax, key in zip(axes, ['A1_fresh', 'A2_recal']):
        df = dfs[key]
        box_data, positions = [], []
        for dc in DISTS:
            sub = df[df['distance_cm'] == dc]
            if len(sub) == 0: continue
            d_f, _ = iqr_filter(sub['d_mm'].values)
            true_mm = dc * 10.0
            box_data.append(d_f - true_mm)
            positions.append(dc/10.0)

        bp = ax.boxplot(box_data, positions=positions, widths=0.04,
                        patch_artist=True, showfliers=True,
                        flierprops=dict(marker='.', ms=2, alpha=0.3, color=COLORS[key]),
                        medianprops=dict(color='k', lw=1.5),
                        boxprops=dict(facecolor=COLORS[key], alpha=0.4))

        ax.axhline(0, color='k', lw=0.8, ls='--', alpha=0.6)
        ax.axhspan(-10, 10, alpha=0.08, color='green')
        ax.set_title(f'{LABELS[key]} — Bias Distribution at Each Distance\n'
                     f'Box = IQR, whiskers = 1.5×IQR, fliers shown')
        ax.set_xlabel('True distance (m)')
        ax.set_ylabel('Bias (mm)')
        ax.set_xlim(0.0, 6.4)
        ax.set_xticks([d/10.0 for d in DISTS])
        ax.set_xticklabels([f'{d/100:.2f}' for d in DISTS], rotation=45)

    fig.tight_layout()
    savefig(fig, 'fig08_boxplots.pdf')

fig_boxplots()


# ════════════════════════════════════════════════════════════════════════════════
# FIGURE 9 — Battery interruption diagnostic (A2_recal at 400 cm)
# ════════════════════════════════════════════════════════════════════════════════
def fig_battery():
    df3 = dfs['A2_recal']
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))

    # Panel 1: Time series at 400 cm
    ax1 = axes[0]
    sub400 = df3[df3['distance_cm'] == 400].copy()
    true_mm = 4000.0
    session_colors = {1: '#1f77b4', 2: '#ff7f0e'}

    for s in sorted(sub400['session'].unique()):
        ss = sub400[sub400['session'] == s].reset_index(drop=True)
        d_f, m = iqr_filter(ss['d_mm'].values)
        idx_filt = np.where(m)[0]
        bias = d_f - true_mm
        ax1.scatter(idx_filt, d_f - true_mm, s=4, alpha=0.5,
                    color=session_colors[int(s)], label=f'Session {int(s)}')
        ax1.axhline(bias.mean(), color=session_colors[int(s)], lw=1.5,
                    ls='--', label=f's{int(s)} mean = {bias.mean():.1f} mm')

    ax1.axhline(0, color='k', lw=0.7, ls=':')
    ax1.set_title('Battery Interruption Test: 400 cm\n'
                  f'(battery replaced between sessions 1 and 2)')
    ax1.set_xlabel('Sample index within session')
    ax1.set_ylabel('Bias (mm)')
    ax1.legend(fontsize=8)

    # Panel 2: Compare all distances for A2_recal — session delta
    ax2 = axes[1]
    dc_list, delta_list, s1_bias, s2_bias = [], [], [], []
    for dc in DISTS:
        sessions = sorted(df3[df3['distance_cm']==dc]['session'].unique())
        if len(sessions) < 2: continue
        true_mm = dc * 10.0
        biases = []
        for s in sessions:
            sub = df3[(df3['distance_cm']==dc) & (df3['session']==s)]
            d_f, _ = iqr_filter(sub['d_mm'].values)
            biases.append(float(d_f.mean() - true_mm))
        dc_list.append(dc/10.0)
        s1_bias.append(biases[0])
        s2_bias.append(biases[1])
        delta_list.append(biases[1]-biases[0])

    delta_arr = np.array(delta_list)
    colors_d  = ['#d62728' if v < 0 else '#2ca02c' for v in delta_list]
    ax2.bar(dc_list, delta_list, width=0.04, color=colors_d, alpha=0.75)
    ax2.axhline(0, color='k', lw=0.8, ls='--', alpha=0.6)
    ax2.axhspan(-5, 5, alpha=0.1, color='green', label='±5 mm tolerance')
    # Highlight 400 cm
    idx400 = dc_list.index(4.0) if 4.0 in dc_list else None
    if idx400 is not None:
        ax2.bar([4.0], [delta_list[idx400]], width=0.04, color='purple',
                alpha=0.9, label=f'400 cm (battery): Δ={delta_list[idx400]:+.1f} mm')
    ax2.set_xlabel('True distance (m)')
    ax2.set_ylabel('Session 2 − Session 1 (mm)')
    ax2.set_title(f'A2_recal: All Session Deltas\nRMS = {np.sqrt(np.mean(delta_arr**2)):.1f} mm')
    ax2.legend(fontsize=8)

    fig.tight_layout()
    savefig(fig, 'fig09_battery_interruption.pdf')

fig_battery()


# ════════════════════════════════════════════════════════════════════════════════
# FIGURE 10 — Near-field anomaly and long-range behaviour
# ════════════════════════════════════════════════════════════════════════════════
def fig_nearfield_longrange():
    fig, axes = plt.subplots(1, 3, figsize=(12, 4.5))

    # Panel 1: Histogram at 5 cm for all datasets
    ax = axes[0]
    for key in ['A1_fresh', 'A2_recal', 'A2_run1']:
        sub = dfs[key][dfs[key]['distance_cm'] == 5]
        if len(sub) == 0: continue
        d = sub['d_mm'].values
        true_mm = 50.0
        b = d - true_mm
        ax.hist(b, bins=60, alpha=0.45, color=COLORS[key], label=LABELS[key],
                density=True, range=(-500, 800))
    ax.axvline(0, color='k', lw=1, ls='--')
    ax.set_title('Near-field (5 cm)\nDistribution of bias')
    ax.set_xlabel('Bias (mm)')
    ax.set_ylabel('Probability density')
    ax.legend(fontsize=8)

    # Panel 2: Outlier fraction vs distance
    ax = axes[1]
    for key in ['A2_run1', 'A1_fresh', 'A2_recal']:
        dc_list, out_list = [], []
        for dc in DISTS:
            s = stats[key].get(dc)
            if s is None: continue
            dc_list.append(dc/10.0)
            out_list.append(s['outlier_pct'])
        ax.plot(dc_list, out_list, color=COLORS[key], marker=MARKERS[key],
                ms=5, lw=1.2, label=LABELS[key])
    ax.set_xlabel('True distance (m)')
    ax.set_ylabel('Outlier fraction (%)')
    ax.set_title('Outlier Rate vs. Distance\n(IQR 1.5× criterion)')
    ax.legend(fontsize=8)

    # Panel 3: Histogram at 550 & 600 cm (A2_recal — long range)
    ax = axes[2]
    for dc, ls in [(550, '-'), (600, '--')]:
        sub = dfs['A2_recal'][dfs['A2_recal']['distance_cm'] == dc]
        if len(sub) == 0: continue
        b = sub['d_mm'].values - dc*10.0
        ax.hist(b, bins=50, alpha=0.45, density=True, ls=ls,
                color=COLORS['A2_recal'],
                label=f'{dc} cm (n={len(sub)})',
                edgecolor=COLORS['A2_recal'])
    ax.axvline(0, color='k', lw=1, ls='--')
    ax.set_title('Long-range Distributions\nA2 recalibrated (550 & 600 cm)')
    ax.set_xlabel('Bias (mm)')
    ax.set_ylabel('Probability density')
    ax.legend(fontsize=8)

    fig.tight_layout()
    savefig(fig, 'fig10_nearfield_longrange.pdf')

fig_nearfield_longrange()


# ════════════════════════════════════════════════════════════════════════════════
# FIGURE 11 — Correction model evaluation (A2_recal as reference)
# ════════════════════════════════════════════════════════════════════════════════
def fig_correction_models():
    # Use A2_recal, exclude 5cm (near-field) and down-weight 600cm
    dc_all  = np.array([dc for dc in DISTS if stats['A2_recal'].get(dc)])
    b_all   = np.array([stats['A2_recal'][dc]['bias'] for dc in dc_all])
    std_all = np.array([stats['A2_recal'][dc]['std']  for dc in dc_all])
    d_m     = dc_all / 100.0   # metres

    # Leave-one-out cross-validation
    def loocv_rmse(model_fn, x, y):
        residuals = []
        for i in range(len(x)):
            xt = np.delete(x, i)
            yt = np.delete(y, i)
            pred = model_fn(xt, yt, x[i])
            residuals.append(y[i] - pred)
        return float(np.sqrt(np.mean(np.array(residuals)**2)))

    def poly1_fn(xt, yt, x_new):
        c = np.polyfit(xt, yt, 1)
        return np.polyval(c, x_new)
    def poly2_fn(xt, yt, x_new):
        c = np.polyfit(xt, yt, 2)
        return np.polyval(c, x_new)
    def poly3_fn(xt, yt, x_new):
        c = np.polyfit(xt, yt, 3)
        return np.polyval(c, x_new)
    def const_fn(xt, yt, x_new):
        return float(np.mean(yt))
    def spline_fn(xt, yt, x_new):
        if len(xt) < 3: return float(np.mean(yt))
        cs = CubicSpline(xt, yt)
        return float(cs(x_new))

    loocv = {
        'No correction (0 mm)': None,
        'Constant offset': loocv_rmse(const_fn, d_m, b_all),
        'Linear (deg 1)':   loocv_rmse(poly1_fn, d_m, b_all),
        'Polynomial deg 2': loocv_rmse(poly2_fn, d_m, b_all),
        'Polynomial deg 3': loocv_rmse(poly3_fn, d_m, b_all),
        'Cubic spline':     loocv_rmse(spline_fn, d_m, b_all),
    }
    loocv['No correction (0 mm)'] = float(np.sqrt(np.mean(b_all**2)))

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    ax1, ax2 = axes

    # Panel 1: model fits
    d_fit = np.linspace(0.0, 6.1, 300)
    ax1.scatter(d_m, b_all, color='k', s=30, zorder=5, label='Measured bias')
    ax1.axhline(0, color='k', lw=0.7, ls=':', alpha=0.5)

    c_const = np.mean(b_all)
    ax1.axhline(c_const, color='gray', lw=1.2, ls='--',
                label=f'Constant = {c_const:.0f} mm')

    for deg, color, ls in [(1,'#ff7f0e','-.'), (2,'#1f77b4','-'), (3,'#d62728','--')]:
        c = np.polyfit(d_m, b_all, deg)
        ax1.plot(d_fit, np.polyval(c, d_fit), color=color, lw=1.2, ls=ls,
                 label=f'Polynomial deg {deg}')

    cs = CubicSpline(d_m, b_all)
    ax1.plot(d_fit, cs(d_fit), color='#9467bd', lw=1.2, ls=(0,(3,1,1,1)),
             label='Cubic spline')

    ax1.set_xlabel('True distance (m)')
    ax1.set_ylabel('Bias to be corrected (mm)')
    ax1.set_title('Correction Model Fits — A2 Recalibrated')
    ax1.legend(fontsize=8)
    ax1.set_xlim(-0.1, 6.3)

    # Panel 2: LOOCV RMSE bar chart
    models = list(loocv.keys())
    rmses  = [loocv[m] for m in models]
    bar_colors = ['#aec7e8']*len(models)
    bar_colors[0] = '#d62728'
    bars = ax2.barh(models, rmses, color=bar_colors, alpha=0.8)
    for bar, v in zip(bars, rmses):
        ax2.text(v + 0.5, bar.get_y() + bar.get_height()/2,
                 f'{v:.1f} mm', va='center', fontsize=9)
    ax2.set_xlabel('Leave-one-out CV RMSE (mm)')
    ax2.set_title('Cross-Validation RMSE by Correction Model\n'
                  '(A2 recalibrated, all distances included)')
    ax2.axvline(loocv['No correction (0 mm)'], color='r', ls='--', lw=1,
                alpha=0.7, label='No correction baseline')
    ax2.legend(fontsize=8)

    fig.tight_layout()
    savefig(fig, 'fig11_correction_models.pdf')

    return loocv

loocv_results = fig_correction_models()


# ════════════════════════════════════════════════════════════════════════════════
# FIGURE 12 — Within-session temporal stability (sample time series)
# ════════════════════════════════════════════════════════════════════════════════
def fig_temporal_stability():
    fig, axes = plt.subplots(2, 3, figsize=(12, 7))
    selected = [(100, 1), (300, 1), (500, 1)]

    for col, (dc, sess) in enumerate(selected):
        for row, (key, df) in enumerate(zip(['A1_fresh', 'A2_recal'], [dfs['A1_fresh'], dfs['A2_recal']])):
            ax = axes[row][col]
            sub = df[(df['distance_cm']==dc) & (df['session']==sess)].reset_index(drop=True)
            if len(sub) == 0:
                ax.set_visible(False); continue
            true_mm = dc * 10.0
            b = sub['d_mm'].values - true_mm
            n = len(b)
            # Running mean (window=50)
            window = min(50, n//4)
            run_mean = np.convolve(b, np.ones(window)/window, 'valid')

            ax.scatter(np.arange(n), b, s=2, alpha=0.25, color=COLORS[key])
            ax.plot(np.arange(window-1, n), run_mean, color=COLORS[key], lw=1.5,
                    label=f'{window}-pt running mean')
            ax.axhline(b.mean(), color='k', lw=1, ls='--',
                       label=f'Mean = {b.mean():.0f} mm')
            ax.axhline(0, color='gray', lw=0.7, ls=':')
            ax.set_title(f'{LABELS[key]}\n{dc} cm, session {sess}')
            ax.set_xlabel('Sample index')
            ax.set_ylabel('Bias (mm)' if col == 0 else '')
            if col == 0 and row == 0:
                ax.legend(fontsize=7)

    fig.suptitle('Within-Session Temporal Stability\n'
                 '(No drift detectable → bias is stationary over 500 samples)',
                 fontsize=11)
    fig.tight_layout()
    savefig(fig, 'fig12_temporal_stability.pdf')

fig_temporal_stability()


# ════════════════════════════════════════════════════════════════════════════════
# FIGURE 13 — Summary heatmap: bias across dataset × distance
# ════════════════════════════════════════════════════════════════════════════════
def fig_heatmap():
    fig, ax = plt.subplots(figsize=(10, 3.5))

    datasets = ['A2_run1', 'A1_fresh', 'A2_recal']
    data_matrix = np.zeros((len(datasets), len(DISTS)))
    data_matrix[:] = np.nan

    for ri, key in enumerate(datasets):
        for ci, dc in enumerate(DISTS):
            s = stats[key].get(dc)
            if s:
                data_matrix[ri, ci] = s['bias']

    # Two separate colour scales: A2_run1 is on a different scale
    im = ax.imshow(data_matrix, aspect='auto', cmap='RdYlGn_r',
                   vmin=-150, vmax=150)
    plt.colorbar(im, ax=ax, label='Bias (mm)', shrink=0.8)

    ax.set_xticks(range(len(DISTS)))
    ax.set_xticklabels([f'{d/100:.2f}' for d in DISTS], rotation=45)
    ax.set_yticks(range(len(datasets)))
    ax.set_yticklabels([LABELS[k] for k in datasets])
    ax.set_xlabel('True distance (m)')
    ax.set_title('Bias Heatmap (mm): All Datasets × Distances\n'
                 '(Green = near zero; Red = large positive/negative)')

    for ri in range(len(datasets)):
        for ci in range(len(DISTS)):
            v = data_matrix[ri, ci]
            if not np.isnan(v):
                ax.text(ci, ri, f'{v:.0f}', ha='center', va='center',
                        fontsize=7.5, color='k' if abs(v) < 100 else 'w')

    fig.tight_layout()
    savefig(fig, 'fig13_bias_heatmap.pdf')

fig_heatmap()


# ════════════════════════════════════════════════════════════════════════════════
# FIGURE 14 — Correction model comparison across A1 and A2
# ════════════════════════════════════════════════════════════════════════════════
def fig_cross_anchor_correction():
    """
    Train a polynomial-2 correction on A2_recal; evaluate residuals on A1_fresh.
    This tests whether a correction learned on one anchor generalizes to another.
    """
    dc_a2 = np.array([dc for dc in DISTS if stats['A2_recal'].get(dc)])
    b_a2  = np.array([stats['A2_recal'][dc]['bias'] for dc in dc_a2])
    d_a2  = dc_a2 / 100.0

    dc_a1 = np.array([dc for dc in DISTS if stats['A1_fresh'].get(dc)])
    b_a1  = np.array([stats['A1_fresh'][dc]['bias'] for dc in dc_a1])
    d_a1  = dc_a1 / 100.0

    # Fit polynomial deg 2 on A2_recal
    c2 = np.polyfit(d_a2, b_a2, 2)

    fig, axes = plt.subplots(1, 2, figsize=(11, 5))

    ax1, ax2 = axes
    d_fit = np.linspace(0, 6.2, 300)

    ax1.scatter(d_a2, b_a2, color=COLORS['A2_recal'], s=40, zorder=4,
                label='A2_recal (training)')
    ax1.scatter(d_a1, b_a1, color=COLORS['A1_fresh'], s=40, zorder=4,
                marker='s', label='A1_fresh (test)')
    ax1.plot(d_fit, np.polyval(c2, d_fit), 'k--', lw=1.2,
             label=f'Poly-2 fit on A2: {c2[0]:.1f}x²{c2[1]:+.1f}x{c2[2]:+.1f}')
    ax1.axhline(0, color='gray', ls=':', lw=0.8)
    ax1.set_xlabel('True distance (m)')
    ax1.set_ylabel('Bias (mm)')
    ax1.set_title('Cross-Anchor Generalization Test\nPoly-2 trained on A2_recal')
    ax1.legend(fontsize=8)

    # Residuals after applying A2 correction to A1_fresh
    corr_on_a1 = b_a1 - np.polyval(c2, d_a1)
    ax2.bar(d_a1, corr_on_a1, width=0.04, color=COLORS['A1_fresh'],
            alpha=0.75, label='Corrected A1_fresh residual')
    corr_on_a2 = b_a2 - np.polyval(c2, d_a2)
    ax2.bar(d_a2+0.05, corr_on_a2, width=0.04, color=COLORS['A2_recal'],
            alpha=0.75, label='Corrected A2_recal residual (training)')
    ax2.axhline(0, color='k', lw=0.8, ls='--')
    rms_a1 = float(np.sqrt(np.mean(corr_on_a1**2)))
    rms_a2 = float(np.sqrt(np.mean(corr_on_a2**2)))
    ax2.set_title(f'Residuals after Poly-2 Correction\n'
                  f'A2 (training) RMSE = {rms_a2:.0f} mm  '
                  f'A1 (test) RMSE = {rms_a1:.0f} mm')
    ax2.set_xlabel('True distance (m)')
    ax2.set_ylabel('Residual bias (mm)')
    ax2.legend(fontsize=8)

    fig.tight_layout()
    savefig(fig, 'fig14_cross_anchor_correction.pdf')
    return rms_a1, rms_a2

rms_a1_poly2, rms_a2_poly2 = fig_cross_anchor_correction()


# ════════════════════════════════════════════════════════════════════════════════
# Print numerical summary for LaTeX report
# ════════════════════════════════════════════════════════════════════════════════
print("\n" + "="*70)
print("  NUMERICAL SUMMARY FOR LATEX REPORT")
print("="*70)

for key, df in dfs.items():
    biases, stds = [], []
    for dc in DISTS:
        s = stats[key].get(dc)
        if s:
            biases.append(s['bias'])
            stds.append(s['std'])
    b = np.array(biases)
    print(f"\n{key}:")
    print(f"  Bias range: [{b.min():.1f}, {b.max():.1f}] mm")
    print(f"  Bias swing: {b.max()-b.min():.1f} mm")
    print(f"  Mean bias:  {b.mean():.1f} mm")
    print(f"  STD range: [{min(stds):.1f}, {max(stds):.1f}] mm")

print(f"\nA2_run1 - A2_recal mean offset: "
      f"{np.mean([stats['A2_run1'][dc]['bias']-stats['A2_recal'][dc]['bias'] for dc in DISTS if stats['A2_run1'].get(dc) and stats['A2_recal'].get(dc)]):.0f} mm")

print(f"\nBattery check (A2_recal, 400cm):")
df3 = dfs['A2_recal']
for s in sorted(df3[df3['distance_cm']==400]['session'].unique()):
    ss = df3[(df3['distance_cm']==400)&(df3['session']==s)]
    d_f,_ = iqr_filter(ss['d_mm'].values)
    print(f"  Session {int(s)}: bias={d_f.mean()-4000:.1f}mm  std={d_f.std():.1f}mm")

print(f"\nCross-validation RMSE (A2_recal):")
for model, rmse in loocv_results.items():
    print(f"  {model}: {rmse:.1f} mm")

print(f"\nCross-anchor poly-2 generalization:")
print(f"  A2_recal (training) RMSE = {rms_a2_poly2:.0f} mm")
print(f"  A1_fresh (test)     RMSE = {rms_a1_poly2:.0f} mm")

# Power-bias correlations
print("\nPower-bias Pearson r:")
for key, df in dfs.items():
    bl, rl = [], []
    for dc in DISTS:
        s = stats[key].get(dc)
        if s: bl.append(s['bias']); rl.append(s['rx_mean'])
    r, p = spstats.pearsonr(np.array(bl), np.array(rl))
    print(f"  {key}: r = {r:.3f}  (p = {p:.3f})")

print("\nAll figures saved to:", FIG_DIR)
print("Done.")
