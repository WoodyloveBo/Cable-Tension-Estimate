#!/usr/bin/env python3
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys

# ——— 설정 ———
BAGFILE        = '14g_T4_timestamp.bag'
WINDOWS        = [(12.5, 15.0), (17.0, 19.5), (21.0, 23.5), (25.0, 27.5)]
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
    rmse    = np.sqrt(np.mean([e**2 for e in errs]))
    min_err = float(np.min(errs))
    max_err = float(np.max(errs))
    return rmse, min_err, max_err

def main():
    # 1) bag 열기
    try:
        bag = rosbag.Bag(BAGFILE, 'r')
    except Exception as e:
        print(f"Error opening '{BAGFILE}': {e}", file=sys.stderr)
        sys.exit(1)

    # 2) 두 토픽 읽기
    fs_raw   = load_vector3_topic(bag, '/cf4/F_s', 'x')
    scal_raw = load_float32_topic(bag, '/cf4/distance_scalar_force_rms')
    bag.close()

    if not fs_raw or not scal_raw:
        print("Error: 필요한 토픽 메시지를 찾을 수 없습니다.", file=sys.stderr)
        sys.exit(1)

    # 3) t0: bag 시작 시점 (F_s의 최소 timestamp)
    t0 = min(t for t, _ in fs_raw)

    # 4) 전체 구간 동기화 (제로 포함)
    synced = []
    for t_fs, fs in fs_raw:
        rel = t_fs - t0
        t_sc, sc = find_nearest(t_fs, scal_raw)
        if abs(t_fs - t_sc) <= SYNC_THRESHOLD:
            synced.append((rel, fs, sc))

    if not synced:
        print(f"No synchronized samples (threshold={SYNC_THRESHOLD}s)")
        sys.exit(1)

    # 5) 플롯
    times_fs, vals_fs = zip(*[(rel, fs) for rel, fs, _ in synced])
    times_sc, vals_sc = zip(*[(rel, sc) for rel, _, sc in synced])

    plt.figure()
    plt.plot(times_fs, vals_fs, label='Predict F_s')
    plt.plot(times_sc, vals_sc, label='Measurement F_s')

    for start, end in WINDOWS:
        plt.axvspan(start, end, alpha=0.2, color='gray')

    plt.xlabel('Time bag (s)')
    plt.ylabel('Cable Force(g)')
    plt.title('Predict F_s vs Measure F_s with Highlighted Section_14g')
    plt.legend()
    plt.tight_layout()

    # 6) 구간별 통계 계산 및 출력
    print("구간별 통계 (RMSE, Min Error, Max Error):")
    for start, end in WINDOWS:
        rmse, min_err, max_err = compute_stats_segment(synced, start, end)
        if rmse is None:
            print(f"  {start:.1f}–{end:.1f}s: 데이터 없음")
        else:
            print(f"  {start:.1f}–{end:.1f}s: RMSE={rmse:.4f}, Min={min_err:.4f}, Max={max_err:.4f}")

    # 7) 그래프 표시
    plt.show()

if __name__ == '__main__':
    main()
