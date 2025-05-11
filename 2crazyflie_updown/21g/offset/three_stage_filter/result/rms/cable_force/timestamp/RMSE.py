#!/usr/bin/env python3
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys

# ——— 설정 ———
BAGFILE        = '20g_T3_F_s.bag'  # 처리할 bag 파일 경로
WINDOWS        = [(25.0, 35.0)]
SYNC_THRESHOLD = 0.05  # 동기화 최대 허용 오차 (초)


def load_vector3_topic(bag, topic, comp):
    """
    Vector3Stamped 토픽을 읽어 (header.stamp, vector.<comp>) 리스트 반환
    """
    out = []
    for _, msg, _ in bag.read_messages(topics=[topic]):
        ts  = msg.header.stamp.to_sec()
        val = getattr(msg.vector, comp)
        out.append((ts, val))
    return out


def load_float32_topic(bag, topic):
    """
    Float32 토픽을 읽어 (rosbag timestamp, data) 리스트 반환
    """
    out = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        ts  = t.to_sec()
        val = msg.data
        out.append((ts, val))
    return out


def find_nearest(t, data):
    """data: [(time, val), …] 중 t에 가장 가까운 항목 반환"""
    if not data:
        return None
    idx = int(np.argmin([abs(ti - t) for ti, _ in data]))
    return data[idx]


def compute_stats_segment(pairs, start, end):
    """
    pairs: [(rel_time, x_val, z_val), …]
    start <= rel_time < end 구간에서
      - rmse: √mean((x−z)²)
      - min_err: 최소 |x−z|
      - max_err: 최대 |x−z|
    """
    errs = [abs(x - z) for rel, x, z in pairs if start <= rel < end]
    if not errs:
        return None, None, None
    errs = np.array(errs)
    rmse    = np.sqrt(np.mean(errs**2))
    min_err = float(errs.min())
    max_err = float(errs.max())
    return rmse, min_err, max_err


def process_fs(prefix, bag):
    """
    prefix에 대해 F_s와 measured F_s(RMS) 동기화, 플롯 및 통계 계산
    """
    fs_topic    = f'/{prefix}/F_s'
    meas_topic  = f'/{prefix}/distance_scalar_force_rms'

    fs_raw   = load_vector3_topic(bag, fs_topic, 'x')
    meas_raw = load_float32_topic(bag, meas_topic)
    if not fs_raw or not meas_raw:
        print(f"[{prefix}] 필요한 토픽 메시지를 찾을 수 없습니다: {fs_topic}, {meas_topic}")
        return None

    # 기준 시각 t0 (첫 F_s timestamp)
    t0 = min(t for t, _ in fs_raw)

    # 동기화
    synced = []
    for t_fs, fs in fs_raw:
        rel = t_fs - t0
        item = find_nearest(t_fs, meas_raw)
        if item:
            t_meas, meas = item
            if abs(t_fs - t_meas) <= SYNC_THRESHOLD:
                synced.append((rel, fs, meas))
    if not synced:
        print(f"[{prefix}] 동기화된 샘플이 없습니다.")
        return None

    # 플롯
    rels = [s[0] for s in synced]
    fs_vals   = [s[1] for s in synced]
    meas_vals = [s[2] for s in synced]

    # cf4 파랑, cf1 빨강 구분
    color = 'blue' if prefix == 'cf4' else 'red'
    plt.plot(rels, fs_vals,   label=f'{prefix} Predict F_s',   color=color, linestyle='-')
    plt.plot(rels, meas_vals, label=f'{prefix} Measure F_s',   color=color, linestyle='--')

    # 섹션 강조
    for start, end in WINDOWS:
        plt.axvspan(start, end, alpha=0.2, color='gray')

    # 통계 계산
    stats = {}
    for start, end in WINDOWS:
        stats[(start, end)] = compute_stats_segment(synced, start, end)
    return stats


def main():
    try:
        bag = rosbag.Bag(BAGFILE, 'r')
    except Exception as e:
        print(f"Error opening '{BAGFILE}': {e}")
        sys.exit(1)

    plt.figure()
    stats_cf4 = process_fs('cf4', bag)
    stats_cf1 = process_fs('cf1', bag)

    plt.xlabel('Time (s)')
    plt.ylabel('Cable Force (g)')
    plt.title('Predict vs Measured F_s for cf4 and cf1')
    plt.legend()
    plt.tight_layout()
    plt.show()

    # 통계 출력
    print("구간별 통계 (RMSE, Min Error, Max Error):")
    for prefix, stats in [('cf4', stats_cf4), ('cf1', stats_cf1)]:
        if stats is None:
            continue
        print(f"[{prefix}]")
        for (start, end), vals in stats.items():
            rmse, mn, mx = vals
            if rmse is None:
                print(f"  {start:.1f}–{end:.1f}s: 데이터 없음")
            else:
                print(f"  {start:.1f}–{end:.1f}s: RMSE={rmse:.4f}, Min={mn:.4f}, Max={mx:.4f}")

    bag.close()

if __name__ == '__main__':
    main()
