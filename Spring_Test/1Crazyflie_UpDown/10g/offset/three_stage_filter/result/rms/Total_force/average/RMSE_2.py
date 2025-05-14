#!/usr/bin/env python3
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys

# ——— 설정 ———
BAGFILE        = '10g_T4_timestamp.bag'
WINDOWS        = [(11.0, 13.5), (15.0, 17.5), (19.0, 21.5), (23.0, 25.5)]
SYNC_THRESHOLD = 0.1  # 동기화 최대 허용 오차 (초)

def load_topic(bag, topic, comp):
    """
    rosbag 메시지 내부 header.stamp 기준으로
    component('x' 또는 'z') 값을 (time, value) 리스트로 반환
    """
    out = []
    for _, msg, _ in bag.read_messages(topics=[topic]):
        ts  = msg.header.stamp.to_sec()
        val = getattr(msg.vector, comp)
        out.append((ts, val))
    return out

def find_nearest(t, data):
    """
    data: [(time, val), …] 중 t에 가장 가까운 항목 반환
    """
    idx = int(np.argmin([abs(t - ti) for ti, _ in data]))
    return data[idx]

def compute_stats_segment(pairs, start, end):
    """
    pairs: [(rel_time, tx, tz), …]
    start <= rel_time < end 구간에서
      - rmse: √mean((tx−tz)²)
      - (min_err, t_min): 최소 절대오차 값과 시각
      - (max_err, t_max): 최대 절대오차 값과 시각
    """
    # (rel_time, abs_error) 리스트
    err_list = [
        (rel, abs(tx - tz))
        for rel, tx, tz in pairs
        if start <= rel < end
    ]
    if not err_list:
        return None, None, None, None, None

    # RMSE 계산
    sq = [e**2 for _, e in err_list]
    rmse = np.sqrt(np.mean(sq))

    # 최소·최대 오차 및 시각
    rels, errs = zip(*err_list)
    min_idx = int(np.argmin(errs))
    max_idx = int(np.argmax(errs))
    t_min, min_err = rels[min_idx], errs[min_idx]
    t_max, max_err = rels[max_idx], errs[max_idx]

    return rmse, min_err, t_min, max_err, t_max

def main():
    # 1) bag 열기 및 토픽 로드
    try:
        bag = rosbag.Bag(BAGFILE, 'r')
    except Exception as e:
        print(f"Error opening '{BAGFILE}': {e}", file=sys.stderr)
        sys.exit(1)

    thrust_raw = load_topic(bag, '/cf4/thrust',      'x')
    total_raw  = load_topic(bag, '/cf4/total_force', 'z')
    bag.close()

    if not thrust_raw or not total_raw:
        print("Error: 필요한 토픽 메시지를 찾을 수 없습니다.", file=sys.stderr)
        sys.exit(1)

    # 2) t0: bag 파일 시작 시점 (thrust 헤더 중 최소 timestamp)
    t0 = min(t for t, _ in thrust_raw)

    # 3) 전체 구간 동기화 (제로 포함)
    synced = []
    for t_tx, tx in thrust_raw:
        rel = t_tx - t0
        t_tz, tz = find_nearest(t_tx, total_raw)
        if abs(t_tx - t_tz) <= SYNC_THRESHOLD:
            synced.append((rel, tx, tz))

    if not synced:
        print(f"No synchronized samples (threshold={SYNC_THRESHOLD}s)")
        sys.exit(1)

    # 4) 플롯
    times_t, vals_t = zip(*[(rel, tx) for rel, tx, tz in synced])
    times_f, vals_f = zip(*[(rel, tz) for rel, tx, tz in synced])

    plt.figure()
    plt.plot(times_t, vals_t, label='thrust.x (including zeros)')
    plt.plot(times_f, vals_f, label='total_force.z')

    # 윈도우 영역 음영 처리
    for start, end in WINDOWS:
        plt.axvspan(start, end, alpha=0.2, color='gray')

    plt.xlabel('Time since bag start (s)')
    plt.ylabel('Value')
    plt.title('thrust.x vs total_force.z with Highlighted Windows')
    plt.legend()
    plt.tight_layout()

    # 5) 구간별 통계 계산 및 출력
    print("구간별 통계 (평균 RMSE, 최소 오류@시각, 최대 오류@시각):")
    for start, end in WINDOWS:
        rmse, min_err, t_min, max_err, t_max = compute_stats_segment(synced, start, end)
        if rmse is None:
            print(f"  {start:.1f}–{end:.1f}s: 데이터 없음")
        else:
            print(
                f"  {start:.1f}–{end:.1f}s → "
                f"RMSE={rmse:.4f}, "
                f"MinErr={min_err:.4f}@{t_min:.2f}s, "
                f"MaxErr={max_err:.4f}@{t_max:.2f}s"
            )

    # 6) 그래프 표시
    plt.show()

if __name__ == '__main__':
    main()
