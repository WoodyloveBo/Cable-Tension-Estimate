#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import os, sys

# ===================== 사용자 설정 =====================
BAGFILE        = '20g_T2_timestamp.bag'
PREFIXES       = ['cf1', 'cf2']
WINDOWS        = [(17.0, 23.0), (27.0, 33.0)]   # Ascending, Descending
SYNC_THRESHOLD = 0.05
SMOOTH_N       = 1 # Moving Average Filter, if) 1이면 꺼진거임.
OUTDIR         = 'fig_out'
USE_LATEX      = False

YLIMS = {
    'x': (-8.0, 8.0),
    'y': (-8.0, 8.0),
    'z': None,
}

COLOR_CF = {
    'cf1': '#D55E00',
    'cf2': '#0072B2',
}

PRED_LINEWIDTH = 2.4
MEAS_LINEWIDTH = 2.4

plt.rcParams.update({
    "figure.dpi": 120,
    "savefig.dpi": 300,
    "font.size": 14,
    "axes.linewidth": 1.0,
    "grid.alpha": 0.35,
    "lines.linewidth": 1.6,
    "mathtext.default": "regular",
})
if USE_LATEX:
    plt.rcParams.update({"text.usetex": True, "font.family": "serif"})


# ===================== 유틸 =====================
def moving_average(x, n):
    if n <= 1: return x
    if n > len(x): return x
    return np.convolve(x, np.ones(n)/n, mode='same')


def load_tension_components(bag, prefix):
    topic = f'/tension_gf/{prefix}/vector'
    rows = []
    for _, msg, _ in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_sec()
        rows.append((ts, msg.vector.x, msg.vector.y, msg.vector.z))
    if not rows:
        return None
    rows.sort()
    return rows


def load_log_components(bag, prefix):
    topic = f'/{prefix}/log1'
    rows = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        vals = getattr(msg, 'values', None)
        if vals is None or len(vals) < 4:
            continue
        rows.append((t.to_sec(), vals[1], vals[2], vals[3]))
    if not rows:
        return None
    rows.sort()
    return rows


def find_nearest_idx(t, arr):
    import bisect
    i = bisect.bisect_left(arr, t)
    if i == 0: return 0
    if i >= len(arr): return len(arr)-1
    return i if abs(arr[i]-t) < abs(arr[i-1]-t) else i-1


def rmse(a, b):
    if len(a) == 0: return np.nan
    return float(np.sqrt(np.mean((np.asarray(a)-np.asarray(b))**2)))


# ===================== 동기화 =====================
def sync_pairs(prefix, bag):
    meas = load_tension_components(bag, prefix)
    pred = load_log_components(bag, prefix)
    if meas is None or pred is None:
        print(f"[{prefix}] 토픽 없음")
        return []

    m_ts = np.array([r[0] for r in meas])
    mx   = np.array([r[1] for r in meas])
    my   = np.array([r[2] for r in meas])
    mz   = np.array([r[3] for r in meas])

    p_ts = np.array([r[0] for r in pred])
    px   = np.array([r[1] for r in pred])
    py   = np.array([r[2] for r in pred])
    pz   = np.array([r[3] for r in pred])

    pairs = []
    for i in range(len(m_ts)):
        j = find_nearest_idx(m_ts[i], p_ts)
        if abs(p_ts[j] - m_ts[i]) <= SYNC_THRESHOLD:
            pairs.append((m_ts[i], px[j], py[j], pz[j],
                          mx[i], my[i], mz[i]))
    return pairs


# ===================== window 선택 =====================
def select_window(pairs, t0, t1):
    return [p for p in pairs if t0 <= p[0] <= t1]


# ===================== 최종 플롯 함수 (Title 제거 버전) =====================
def plot_window_side_by_side(windows, pairs_by_cf):

    fig, axes = plt.subplots(3, 2, figsize=(7.5, 6), sharey='row')
    z_all = []

    for col, (t0, t1) in enumerate(windows):
        window_length = t1 - t0

        for prefix, pairs in pairs_by_cf.items():
            seg = select_window(pairs, t0, t1)
            if not seg:
                continue

            ts_abs = np.array([p[0] for p in seg])
            ts = ts_abs - t0

            px = np.array([p[1] for p in seg])
            py = np.array([p[2] for p in seg])
            pz = np.array([p[3] for p in seg])
            mx = np.array([p[4] for p in seg])
            my = np.array([p[5] for p in seg])
            mz = np.array([p[6] for p in seg])

            color = COLOR_CF[prefix]

            axes[0, col].plot(ts, px, color=color, alpha=1.0, linewidth=PRED_LINEWIDTH)
            axes[1, col].plot(ts, py, color=color, alpha=1.0, linewidth=PRED_LINEWIDTH)
            axes[2, col].plot(ts, pz, color=color, alpha=1.0, linewidth=PRED_LINEWIDTH)

            axes[0, col].plot(ts, mx, color=color, alpha=0.6, linewidth=MEAS_LINEWIDTH, linestyle='--')
            axes[1, col].plot(ts, my, color=color, alpha=0.6, linewidth=MEAS_LINEWIDTH, linestyle='--')
            axes[2, col].plot(ts, mz, color=color, alpha=0.6, linewidth=MEAS_LINEWIDTH, linestyle='--')

            z_all.extend(pz.tolist())
            z_all.extend(mz.tolist())

        # x-range
        for r in range(3):
            axes[r, col].set_xlim(0, window_length)

    # === y-limit 설정 ===
    axes[0, 0].set_ylim(*YLIMS['x'])
    axes[1, 0].set_ylim(*YLIMS['y'])

    if z_all:
        z_mean = np.mean(z_all)
        for col in range(2):
            axes[2, col].set_ylim(z_mean - 7, z_mean + 7)

    # === grid & ticks ===
    for r in range(3):
        for c in range(2):
            axes[r, c].grid(True)
            axes[r, c].minorticks_on()

            # ❗ 위 두 행(x, y)에서는 x축 눈금 숨기기
            if r < 2:
                axes[r, c].tick_params(labelbottom=False)

    plt.tight_layout()
    os.makedirs(OUTDIR, exist_ok=True)
    base = f"{os.path.splitext(os.path.basename(BAGFILE))[0]}_ascending_descending_notitle"

    for ext in (".pdf", ".svg", ".png"):
        plt.savefig(os.path.join(OUTDIR, base + ext), bbox_inches='tight')

    plt.show()
    print(f"[✔] Saved: {base}")

# ===================== 메인 =====================
def main():
    try:
        bag = rosbag.Bag(BAGFILE, 'r')
    except Exception as e:
        print(f"Error opening {BAGFILE}: {e}")
        sys.exit(1)

    pairs_by_cf = {pf: sync_pairs(pf, bag) for pf in PREFIXES}

    # RMSE 출력
    for (w0, w1) in WINDOWS:
        print(f"\n=== Window {w0:.2f}–{w1:.2f}s RMSE (gf) ===")
        for pf, pairs in pairs_by_cf.items():
            seg = select_window(pairs, w0, w1)
            if not seg:
                print(f"  {pf}: (no data)")
                continue

            px = np.array([p[1] for p in seg])
            py = np.array([p[2] for p in seg])
            pz = np.array([p[3] for p in seg])
            mx = np.array([p[4] for p in seg])
            my = np.array([p[5] for p in seg])
            mz = np.array([p[6] for p in seg])

            pred_norm = np.sqrt(px**2 + py**2 + pz**2)
            meas_norm = np.sqrt(mx**2 + my**2 + mz**2)

            print(f"  {pf}: RMSE_x={rmse(px,mx):.3f}, RMSE_y={rmse(py,my):.3f}, RMSE_z={rmse(pz,mz):.3f}, RMSE_norm={rmse(pred_norm, meas_norm):.3f}")

    # 플롯 실행
    plot_window_side_by_side(WINDOWS, pairs_by_cf)

    bag.close()


if __name__ == '__main__':
    main()
