#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import argparse
import glob
import csv
import numpy as np
import rosbag
import genpy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float32


def _pose_to_xyz(msg: PoseStamped):
    return np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)

def _pointcloud_single_point_to_xyz(msg: PointCloud):
    if not msg.points:
        raise ValueError("PointCloud has no points")
    p = msg.points[0]
    return np.array([p.x, p.y, p.z], dtype=float)

def _load_series(bag_path, topic, extractor):
    times, xyz = [], []
    with rosbag.Bag(bag_path, 'r') as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            try:
                pos = extractor(msg)
            except Exception:
                continue
            times.append(t.to_sec())
            xyz.append(pos)
    if not times:
        return [], np.zeros((0, 3))
    order = np.argsort(times)
    times_sorted = [times[i] for i in order]
    xyz_sorted = np.array([xyz[i] for i in order], dtype=float)
    return times_sorted, xyz_sorted

def _interp_pos(t_query, t_list, xyz_list):
    import bisect
    i = bisect.bisect_left(t_list, t_query)
    if i <= 0:
        return xyz_list[0]
    if i >= len(t_list):
        return xyz_list[-1]
    t0, t1 = t_list[i-1], t_list[i]
    p0, p1 = xyz_list[i-1], xyz_list[i]
    if t1 == t0:
        return p0
    w = (t_query - t0) / (t1 - t0)
    return (1.0 - w) * p0 + w * p1

def _dist(a, b):
    return float(np.linalg.norm(a - b))

def process_one_file(in_bag, out_bag, write_csv=True):
    # 타임시리즈 로드
    cf2_t, cf2_xyz = _load_series(in_bag, "/natnet_ros/cf2/pose", _pose_to_xyz)
    cf3_t, cf3_xyz = _load_series(in_bag, "/natnet_ros/cf3/pose", _pose_to_xyz)
    pt_t,  pt_xyz  = _load_series(in_bag, "/point_index9", _pointcloud_single_point_to_xyz)

    if len(cf2_t) == 0 or len(cf3_t) == 0 or len(pt_t) == 0:
        raise RuntimeError("필수 토픽이 비었습니다. (cf2, cf3, point_index9 중 하나 이상)")

    # CSV 준비
    csv_path = os.path.splitext(out_bag)[0] + "_distance.csv"
    csv_fh = open(csv_path, "w", newline="") if write_csv else None
    csv_writer = None
    if csv_fh:
        csv_writer = csv.writer(csv_fh)
        csv_writer.writerow(["stamp_sec", "dist_cf2_m", "dist_cf3_m"])

    with rosbag.Bag(in_bag, 'r') as inbag, rosbag.Bag(out_bag, 'w') as outbag:
        # 원본 전체 복사 + point_index9 시점에 거리 토픽 추가
        for topic, msg, t in inbag.read_messages():
            outbag.write(topic, msg, t)

            if topic == "/point_index9":
                try:
                    p_point = _pointcloud_single_point_to_xyz(msg)
                except Exception:
                    continue

                ti = t.to_sec()
                p_cf2 = _interp_pos(ti, cf2_t, cf2_xyz)
                p_cf3 = _interp_pos(ti, cf3_t, cf3_xyz)

                d1 = _dist(p_cf2, p_point)
                d3 = _dist(p_cf3, p_point)

                # bag에 기록 (std_msgs/Float32)
                m1 = Float32(); m1.data = d1
                m3 = Float32(); m3.data = d3
                stamp = t  # 원본과 동일 타임스탬프

                outbag.write("/distance/cf2", m1, stamp)
                outbag.write("/distance/cf3", m3, stamp)

                # CSV 기록
                if csv_writer:
                    csv_writer.writerow([f"{ti:.9f}", f"{d1:.9f}", f"{d3:.9f}"])

    if csv_fh:
        csv_fh.close()
        print(f"    CSV saved: {csv_path}")

def main():
    ap = argparse.ArgumentParser(description="Copy all topics and ADD distances (cf2↔point_index9, cf3↔point_index9).")
    ap.add_argument("-p", "--pattern", default="20g_T?_index9.bag",
                    help="입력 bag 글롭 패턴 (기본: %(default)s)")
    ap.add_argument("--suffix", default="_with_distance",
                    help="출력 파일 접미사(확장자 앞, 기본: %(default)s)")
    ap.add_argument("--no-csv", action="store_true",
                    help="CSV 저장 비활성화")
    args = ap.parse_args()

    files = sorted(glob.glob(args.pattern))
    if not files:
        print(f"[!] No files matched: {args.pattern}")
        return

    print(f"[*] files: {len(files)}")
    for f in files:
        base, ext = os.path.splitext(f)
        out_path = f"{base}{args.suffix}{ext}"
        print(f"  > {os.path.basename(f)} -> {os.path.basename(out_path)}")
        process_one_file(f, out_path, write_csv=(not args.no_csv))

    print("[✓] Done.")

if __name__ == "__main__":
    main()
