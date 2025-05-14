#!/usr/bin/env python3
import rosbag
import numpy as np
import matplotlib.pyplot as plt

# ——— 설정 ———
BAGFILE        = '10g_T4_timestamp.bag'
WINDOWS        = [(11.0, 13.5), (15.0, 17.5), (19.0, 21.5), (23.0, 25.5)]
SYNC_THRESHOLD = 0.1  # 동기화 최대 허용 오차 (초)

def load_topic(bag, topic, comp):
    """
    rosbag 메시지의 header.stamp 을 기준으로,
    component('x' 또는 'z') 값을 (time, value) 리스트로 반환
    """
    out = []
    for _, msg, _ in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_sec()
        val = getattr(msg.vector, comp)
        out.append((ts, val))
    return out

def find_nearest(t, data):
    """data: [(time, val), …] 중 t에 가장 가까운 항목 반환"""
    idx = int(np.argmin([abs(t - ti) for ti, _ in data]))
    return data[idx]

def compute_stats_segment(pairs, start, end):
    """
    pairs: [(rel_time, tx, tz), …]
    start <= rel_time < end 구간에서
      - rmse: √mean((tx−tz)²)
      - min_err: 최소 |tx−tz|
      - max_err: 최대 |tx−tz|
    """
    errs = [abs(tx - tz) for rel, tx, tz in pairs if start <= rel < end]
    if not errs:
        return None, None, None
    rmse    = np.sqrt(np.mean([e**2 for e in errs]))
    min_err = float(np.min(errs))
    max_err = float(np.max(errs))
    return rmse, min_err, max_err

def main():
    # 1) bag 열기 및 토픽 로드
    bag = rosbag.Bag(BAGFILE, 'r')
    thrust_raw = load_topic(bag, '/cf4/thrust',      'x')
    total_raw  = load_topic(bag, '/cf4/total_force', 'z')
    bag.close()

    # 2) t0: bag 파일 시작 시점 (thrust header.stamp 최소값)
    t0 = min(t for t, _ in thrust_raw)

    # 3) 데이터 동기화 (제로 포함)
    synced = []
    for t_tx, tx in thrust_raw:
        rel = t_tx - t0
        t_tz, tz = find_nearest(t_tx, total_raw)
        if abs(t_tx - t_tz) <= SYNC_THRESHOLD:
            synced.append((rel, tx, tz))

    # 4) 플롯 준비
    times_t, vals_t = zip(*[(rel, tx) for rel, tx, _ in synced])
    times_f, vals_f = zip(*[(rel, tz) for rel, _, tz in synced])

    plt.figure()
    plt.plot(times_t, vals_t, label='Thrust')
    plt.plot(times_f, vals_f, label='Total_force')

    for start, end in WINDOWS:
        plt.axvspan(start, end, alpha=0.2, color='gray')

    plt.xlabel('Time bag (s)')
    plt.ylabel('Thrust(g)')
    plt.title('Thrust vs Total_force with Highlighted Section_10g')
    plt.legend()
    plt.tight_layout()

    # 5) 구간별 통계 계산 및 출력
    print("구간별 통계 (RMSE, Min Error, Max Error):")
    for start, end in WINDOWS:
        rmse, min_err, max_err = compute_stats_segment(synced, start, end)
        if rmse is None:
            print(f"  {start:.1f}–{end:.1f}s: 데이터 없음")
        else:
            print(f"  {start:.1f}–{end:.1f}s: RMSE={rmse:.4f}, Min={min_err:.4f}, Max={max_err:.4f}")

    # 6) 그래프 표시
    plt.show()

if __name__ == '__main__':
    main()

