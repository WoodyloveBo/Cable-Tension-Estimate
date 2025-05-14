#!/usr/bin/env python3
import rosbag
import numpy as np
import sys

# ——— 설정 ———
BAGFILE        = '14g_T4_timestamp.bag'
WINDOWS        = [(12.5, 15.0), (17.0, 19.5), (21.0, 23.5), (25.0, 27.5)]
SYNC_THRESHOLD = 0.05  # 동기화 최대 허용 오차 (초)

def load_topic(bag, topic, comp):
    """rosbag에서 topic 메시지를 (time, value) 리스트로 읽어들임"""
    out = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        out.append((t.to_sec(), getattr(msg.vector, comp)))
    return out

def find_nearest(t, data):
    """data: [(time, val), …] 중 t에 가장 가까운 항목 반환"""
    idx = int(np.argmin([abs(t - ti) for ti, _ in data]))
    return data[idx]

def compute_stats_segment(pairs, start, end):
    """
    pairs: [(rel_time, tx, tz), …]
    start<=rel_time<end 구간에서
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
    try:
        bag = rosbag.Bag(BAGFILE, 'r')
    except Exception as e:
        print(f"Error opening '{BAGFILE}': {e}", file=sys.stderr)
        sys.exit(1)

    thrust = load_topic(bag, '/cf4/thrust',      'x')
    total  = load_topic(bag, '/cf4/total_force', 'z')
    bag.close()

    if not thrust or not total:
        print("Error: 필요한 토픽 메시지를 찾을 수 없습니다.", file=sys.stderr)
        sys.exit(1)

    # 2) 그래프상의 0s: thrust.x가 0→양수로 바뀌는 첫 시점
    t0 = next(t for t, tx in thrust if tx > 0.0)

    # 3) 상대시간으로 동기화된 리스트 생성
    synced = []
    for t_tx, tx in thrust:
        rel = t_tx - t0
        if rel < 0:
            continue
        t_tz, tz = find_nearest(t_tx, total)
        if abs(t_tx - t_tz) <= SYNC_THRESHOLD:
            synced.append((rel, tx, tz))

    print(f"동기화된 샘플: {len(synced)}개 (threshold={SYNC_THRESHOLD}s)\n")

    # 4) 구간별 통계 출력
    for start, end in WINDOWS:
        rmse, min_err, max_err = compute_stats_segment(synced, start, end)
        if rmse is None:
            print(f"구간 {start:.1f}–{end:.1f}초: 데이터 없음")
        else:
            print(
                f"구간 {start:.1f}–{end:.1f}초 → "
                f"평균 RMSE={rmse:.4f}, "
                f"최소 오류={min_err:.4f}, "
                f"최대 오류={max_err:.4f}"
            )

if __name__ == '__main__':
    main()
