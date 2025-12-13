#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import os, sys

# ===================== 사용자 설정 =====================
BAGFILE        = '26g_T1_timestamp.bag'
PREFIXES       = ['cf1', 'cf2']
WINDOWS        = [(17.0, 23.0),(27.0, 33.0)]
SYNC_THRESHOLD = 0.05
SMOOTH_N       = 1
OUTDIR         = 'fig_out'
USE_LATEX      = False

# ▶ 축별 Y-범위 설정
#   z는 여기 값 대신 "평균 ± 7"을 사용할 거라 None으로 둬도 되고, 그냥 두어도 됨.
YLIMS = {
    'x': (-8.0, 8.0),
    'y': (-5.0, 5.0),
    'z': None,   # z축은 아래에서 평균±7로 자동 설정
}

COLOR_CF = {
    'cf1': '#D55E00',
    'cf2': '#0072B2',
}

PRED_LINEWIDTH = 2.2
MEAS_LINEWIDTH = 2.2

# matplotlib 스타일
plt.rcParams.update({
    "figure.dpi": 120,
    "savefig.dpi": 300,
    "font.size": 11,
    "axes.linewidth": 1.0,
    "grid.alpha": 0.35,
    "lines.linewidth": 1.6,
    "mathtext.default": "regular",
})
if USE_LATEX:
    plt.rcParams.update({"text.usetex": True, "font.family": "serif"})

# ===================== 유틸 =====================
def moving_average(x, n):
    if n is None or n <= 1: 
        return x
    n = int(n)
    if n <= 1 or n > len(x): 
        return x
    k = np.ones(n, dtype=float) / n
    return np.convolve(x, k, mode='same')

def load_tension_components(bag, prefix):
    topic = f'/tension_gf/{prefix}/vector'
    rows = []
    for _, msg, _ in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_sec()
        rows.append((ts, float(msg.vector.x), float(msg.vector.y), float(msg.vector.z)))
    if not rows: 
        return None
    rows.sort(key=lambda r: r[0])
    return rows   # 절대시간 + xyz 반환

def load_log_components(bag, prefix):
    topic = f'/{prefix}/log1'
    rows = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        vals = getattr(msg, 'values', None)
        if vals is None or len(vals) < 4:
            continue
        ts = t.to_sec()
        rows.append((ts, float(vals[1]), float(vals[2]), float(vals[3])))
    if not rows:
        return None
    rows.sort(key=lambda r: r[0])
    return rows  # 절대시간 + xyz 반환

def find_nearest_idx(t, arr):
    import bisect
    i = bisect.bisect_left(arr, t)
    if i == 0:
        return 0
    if i == len(arr):
        return len(arr)-1
    return i if abs(arr[i] - t) < abs(arr[i-1] - t) else i-1

def rmse(a, b):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    if a.size == 0:
        return np.nan
    return float(np.sqrt(np.mean((a - b)**2)))

# ===================== 절대시간 기반 정확한 동기화 =====================
def sync_pairs(prefix, bag):
    meas = load_tension_components(bag, prefix)
    pred = load_log_components(bag, prefix)

    if meas is None or pred is None:
        print(f"[{prefix}] 토픽 없음 → tension/meas 또는 onboard/log")
        return []

    # meas (절대시간)
    m_ts = np.array([r[0] for r in meas], dtype=float)
    mx  = np.array([r[1] for r in meas], dtype=float)
    my  = np.array([r[2] for r in meas], dtype=float)
    mz  = np.array([r[3] for r in meas], dtype=float)

    # pred (절대시간)
    p_ts = np.array([r[0] for r in pred], dtype=float)
    px   = np.array([r[1] for r in pred], dtype=float)
    py   = np.array([r[2] for r in pred], dtype=float)
    pz   = np.array([r[3] for r in pred], dtype=float)

    pairs = []
    for i in range(len(m_ts)):
        j = find_nearest_idx(m_ts[i], p_ts)
        if abs(p_ts[j] - m_ts[i]) <= SYNC_THRESHOLD:
            # pairs의 시간축 = meas의 절대시간
            pairs.append((m_ts[i], px[j], py[j], pz[j], mx[i], my[i], mz[i]))

    return pairs

# ===================== 윈도우 선택 (절대시간 기준) =====================
def select_window(pairs, t0, t1):
    # p[0] = 절대시간
    return [p for p in pairs if t0 <= p[0] <= t1]

# ===================== 플롯 =====================
def plot_window_no_labels(window, pairs_by_cf):
    t0, t1 = window

    fig, axes = plt.subplots(3, 1, figsize=(7.2, 6.5), sharex=True)

    for ax in axes:
        ax.grid(True, which='major')
        ax.minorticks_on()
        ax.grid(True, which='minor', alpha=0.15)

    # z축 평균 계산용 리스트
    z_all = []

    for prefix, pairs in pairs_by_cf.items():
        pairs_w = select_window(pairs, t0, t1)
        if not pairs_w:
            continue

        ts = np.array([p[0] for p in pairs_w], dtype=float)
        px = np.array([p[1] for p in pairs_w], dtype=float)
        py = np.array([p[2] for p in pairs_w], dtype=float)
        pz = np.array([p[3] for p in pairs_w], dtype=float)
        mx = np.array([p[4] for p in pairs_w], dtype=float)
        my = np.array([p[5] for p in pairs_w], dtype=float)
        mz = np.array([p[6] for p in pairs_w], dtype=float)

        if SMOOTH_N > 1:
            px = moving_average(px, SMOOTH_N)
            py = moving_average(py, SMOOTH_N)
            pz = moving_average(pz, SMOOTH_N)
            mx = moving_average(mx, SMOOTH_N)
            my = moving_average(my, SMOOTH_N)
            mz = moving_average(mz, SMOOTH_N)

        color = COLOR_CF.get(prefix, None)

        # ✅ 예측값 (log) – 굵기 PRED_LINEWIDTH
        axes[0].plot(ts, px, color=color, alpha=1.0,  linewidth=PRED_LINEWIDTH)
        axes[1].plot(ts, py, color=color, alpha=1.0,  linewidth=PRED_LINEWIDTH)
        axes[2].plot(ts, pz, color=color, alpha=1.0,  linewidth=PRED_LINEWIDTH)

        # ✅ 측정값 (tension_gf) – 굵기 MEAS_LINEWIDTH
        axes[0].plot(ts, mx, color=color, alpha=0.35, linewidth=MEAS_LINEWIDTH, linestyle='--')
        axes[1].plot(ts, my, color=color, alpha=0.35, linewidth=MEAS_LINEWIDTH, linestyle='--')
        axes[2].plot(ts, mz, color=color, alpha=0.35, linewidth=MEAS_LINEWIDTH, linestyle='--')

        # z 데이터 모두 수집 (pred + meas)
        z_all.extend(pz.tolist())
        z_all.extend(mz.tolist())

    # X축 window 적용
    for ax in axes:
        ax.set_xlim(t0, t1)

    # Y축 제한: x, y는 YLIMS 사용
    if YLIMS.get('x') is not None:
        axes[0].set_ylim(*YLIMS['x'])
    if YLIMS.get('y') is not None:
        axes[1].set_ylim(*YLIMS['y'])

    # z축: 평균 ± 7
    if z_all:
        z_all = np.array(z_all, dtype=float)
        z_mean = float(np.mean(z_all))
        axes[2].set_ylim(z_mean - 7.0, z_mean + 7.0)

    # 제목/라벨 제거
    for ax in axes:
        ax.set_title('')
        ax.set_xlabel('')
        ax.set_ylabel('')

    plt.tight_layout()
    os.makedirs(OUTDIR, exist_ok=True)
    base = f"{os.path.splitext(os.path.basename(BAGFILE))[0]}_{t0:.2f}-{t1:.2f}s_comp_nolabel"
    for ext in (".pdf", ".svg", ".png"):
        plt.savefig(os.path.join(OUTDIR, base + ext), bbox_inches='tight')
    plt.show()
    print(f"[✔] Saved: {base}.[pdf|svg|png]")

# ===================== 메인 =====================
def main():
    try:
        bag = rosbag.Bag(BAGFILE, 'r')
    except Exception as e:
        # 오타 수정: BBOXFILE → BAGFILE
        print(f"Error opening '{BAGFILE}': {e}")
        sys.exit(1)

    pairs_by_cf = {pf: sync_pairs(pf, bag) for pf in PREFIXES}

    for (w0, w1) in WINDOWS:
        print(f"\n=== Window {w0:.2f}–{w1:.2f}s RMSE (gf) ===")

        for pf, pairs in pairs_by_cf.items():
            seg = select_window(pairs, w0, w1)
            if not seg:
                print(f"  {pf}: (no data)")
                continue

            px = np.array([p[1] for p in seg], dtype=float)
            py = np.array([p[2] for p in seg], dtype=float)
            pz = np.array([p[3] for p in seg], dtype=float)
            mx = np.array([p[4] for p in seg], dtype=float)
            my = np.array([p[5] for p in seg], dtype=float)
            mz = np.array([p[6] for p in seg], dtype=float)

            pred_norm = np.sqrt(px**2 + py**2 + pz**2)
            meas_norm = np.sqrt(mx**2 + my**2 + mz**2)

            print(
                f"  {pf}: RMSE_x={rmse(px,mx):.3f}, "
                f"RMSE_y={rmse(py,my):.3f}, "
                f"RMSE_z={rmse(pz,mz):.3f}, "
                f"RMSE_norm={rmse(pred_norm, meas_norm):.3f}"
            )

        plot_window_no_labels((w0, w1), pairs_by_cf)

    bag.close()

if __name__ == '__main__':
    main()
