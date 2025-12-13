#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rosbag
import numpy as np
import sys

# ===================== 사용자 설정 =====================
BAGFILE           = "alpha_index9_ftB.bag"

# 상대 구간(초). 아래 값은 "대상 토픽들의 실제 시작시각 중 최솟값"을 0초로 보고 적용됩니다.
WINDOW_REL        = (17.0, 23.0)

# 대상 토픽
FTB_TOPIC_FMT     = "/{cf}/ft_b"   # Vector3Stamped, header.stamp 사용
LOG1_TOPIC_FMT    = "/{cf}/log1"   # values[0], rosbag t 사용
CFS               = ["cf1", "cf2"]

# ===================== 유틸 =====================
def topic_time_range_ftb_header(bag, cf):
    """ /{cf}/ft_b 의 header.stamp 범위 반환 (min_ts, max_ts) 또는 (None, None) """
    topic = FTB_TOPIC_FMT.format(cf=cf)
    mn, mx = None, None
    for _, msg, _ in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_sec()
        mn = ts if mn is None else min(mn, ts)
        mx = ts if mx is None else max(mx, ts)
    return mn, mx

def topic_time_range_log1_bag(bag, cf):
    """ /{cf}/log1 의 bag timestamp 범위 반환 (min_ts, max_ts) 또는 (None, None) """
    topic = LOG1_TOPIC_FMT.format(cf=cf)
    mn, mx = None, None
    for _, msg, t in bag.read_messages(topics=[topic]):
        ts = t.to_sec()
        mn = ts if mn is None else min(mn, ts)
        mx = ts if mx is None else max(mx, ts)
    return mn, mx

def load_ft_b_z(bag, cf, t0, t1):
    """ header.stamp in [t0, t1] 범위의 z 성분 수집 """
    topic = FTB_TOPIC_FMT.format(cf=cf)
    vals = []
    for _, msg, _ in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_sec()
        if t0 <= ts <= t1:
            vals.append(float(msg.vector.z))
    return np.array(vals, dtype=float)

def load_log1_val0(bag, cf, t0, t1):
    """ bag timestamp in [t0, t1] 범위의 values[0] 수집 """
    topic = LOG1_TOPIC_FMT.format(cf=cf)
    vals = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        ts = t.to_sec()
        if t0 <= ts <= t1:
            v = getattr(msg, "values", None)
            if v is not None and len(v) >= 1:
                vals.append(float(v[0]))
    return np.array(vals, dtype=float)

def avg_str(arr):
    if arr.size == 0:
        return "N/A (0 samples)"
    return f"{float(np.mean(arr)):.6f}  ({arr.size} samples)"

# ===================== 메인 =====================
def main():
    try:
        bag = rosbag.Bag(BAGFILE, 'r')
    except Exception as e:
        print(f"[!] bag 오픈 실패: {e}")
        sys.exit(1)

    print(f"Bag: {BAGFILE}")

    # 1) 토픽별 실제 가용 시간 범위 스캔
    ranges = {}
    starts = []

    for cf in CFS:
        mn_h, mx_h = topic_time_range_ftb_header(bag, cf)
        ranges[f"/{cf}/ft_b(header)"] = (mn_h, mx_h)
        if mn_h is not None:
            starts.append(mn_h)

        mn_l, mx_l = topic_time_range_log1_bag(bag, cf)
        ranges[f"/{cf}/log1(bagtime)"] = (mn_l, mx_l)
        if mn_l is not None:
            starts.append(mn_l)

    print("Available time ranges:")
    for k, (mn, mx) in ranges.items():
        if mn is None:
            print(f"  {k:22s}: (no data)")
        else:
            print(f"  {k:22s}: [{mn:.6f}, {mx:.6f}]")

    if not starts:
        print("[!] 대상 토픽들에서 데이터를 찾지 못했습니다.")
        bag.close()
        sys.exit(1)

    # 2) 상대시간 기준 = 대상 4개 스트림 중 '가장 이른 시작시각'
    base = min(starts)
    a_rel, b_rel = WINDOW_REL
    a_abs = base + a_rel
    b_abs = base + b_rel

    print("-" * 60)
    print(f"Relative window: [{a_rel:.3f}, {b_rel:.3f}] s  ->  Absolute: [{a_abs:.6f}, {b_abs:.6f}] s")
    print("-" * 60)

    # 3) 평균 계산
    for cf in CFS:
        ft_z = load_ft_b_z(bag, cf, a_abs, b_abs)
        lg0  = load_log1_val0(bag, cf, a_abs, b_abs)
        print(f"[{cf}] /{cf}/ft_b.z   mean = {avg_str(ft_z)}")
        print(f"[{cf}] /{cf}/log1[0]  mean = {avg_str(lg0)}")
        print("-" * 60)

    bag.close()

if __name__ == "__main__":
    main()
