#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import argparse
import glob
import numpy as np
import rosbag

from geometry_msgs.msg import Vector3Stamped, PoseStamped
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float32


# =======================
# 🔧 Parameters (edit here)
# =======================
K = 0.7686        # [N/m]   slope from regression
B = 0.0448        # [N]     intercept (preload/bias)
L0_cf1 = 0.205    # [m]     natural length (cf1 <-> point)
L0_cf2 = 0.205    # [m]     natural length (cf2 <-> point)
CABLE_CLAMP = True  # True: compression -> 0 (tension-only cable)

GF_PER_NEWTON = 1.0 / 0.00980665  # N → gf


# =======================
# Helpers
# =======================
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
            times.append(t.to_sec()); xyz.append(pos)
    if not times:
        return [], np.zeros((0, 3))
    order = np.argsort(times)
    times_sorted = [times[i] for i in order]
    xyz_sorted = np.array([xyz[i] for i in order], dtype=float)
    return times_sorted, xyz_sorted

def _interp_pos(t_query, t_list, xyz_list):
    import bisect
    i = bisect.bisect_left(t_list, t_query)
    if i <= 0: return xyz_list[0]
    if i >= len(t_list): return xyz_list[-1]
    t0, t1 = t_list[i-1], t_list[i]
    p0, p1 = xyz_list[i-1], xyz_list[i]
    if t1 == t0: return p0
    w = (t_query - t0) / (t1 - t0)
    return (1.0 - w) * p0 + w * p1

def _force_vec_from_regression(p_drone, p_point, L0):
    """
    F_N = k*(r - L0) + b  (optionally clamp x = max(0, r-L0))
    F_vec_N = F_N * (d / r)
    """
    d = p_drone - p_point
    r = float(np.linalg.norm(d))
    if r < 1e-12:
        return np.zeros(3), 0.0
    u = d / r
    x = r - L0
    if CABLE_CLAMP:
        x = max(0.0, x)  # tension-only
    F_N = K * x + B
    if CABLE_CLAMP and F_N < 0.0:
        F_N = 0.0
    F_vec_N = F_N * u
    return F_vec_N, F_N


# =======================
# Main processing
# =======================
def process_one_file(in_bag, out_bag):
    # 보간용 시계열 로드
    cf1_t, cf1_xyz = _load_series(in_bag, "/natnet_ros/cf1/pose", _pose_to_xyz)
    cf2_t, cf2_xyz = _load_series(in_bag, "/natnet_ros/cf2/pose", _pose_to_xyz)
    pt_t,  pt_xyz  = _load_series(in_bag, "/point_index9", _pointcloud_single_point_to_xyz)

    if len(cf1_t) == 0 or len(cf2_t) == 0 or len(pt_t) == 0:
        raise RuntimeError("필수 토픽이 비었습니다. (cf1, cf2, point_index9 중 하나 이상)")

    with rosbag.Bag(in_bag, 'r') as inbag, rosbag.Bag(out_bag, 'w') as outbag:
        for topic, msg, t in inbag.read_messages():
            # 1) 원본 그대로 복사
            outbag.write(topic, msg, t)

            # 2) /point_index9 시점마다 장력(gf) 토픽 추가
            if topic == "/point_index9":
                try:
                    p_point = _pointcloud_single_point_to_xyz(msg)
                except Exception:
                    continue

                ti = t.to_sec()
                p_cf1 = _interp_pos(ti, cf1_t, cf1_xyz)
                p_cf2 = _interp_pos(ti, cf2_t, cf2_xyz)

                # cf1
                F1_vec_N, F1_mag_N = _force_vec_from_regression(p_cf1, p_point, L0_cf1)
                F1_vec_gf = F1_vec_N * GF_PER_NEWTON
                F1_mag_gf = float(F1_mag_N * GF_PER_NEWTON)
                v1 = Vector3Stamped(); v1.header.stamp = t; v1.header.frame_id = "world"
                v1.vector.x, v1.vector.y, v1.vector.z = F1_vec_gf.tolist()
                m1 = Float32(); m1.data = F1_mag_gf
                outbag.write("/tension_gf/cf1/vector", v1, t)
                outbag.write("/tension_gf/cf1/magnitude", m1, t)

                # cf2
                F3_vec_N, F3_mag_N = _force_vec_from_regression(p_cf2, p_point, L0_cf2)
                F3_vec_gf = F3_vec_N * GF_PER_NEWTON
                F3_mag_gf = float(F3_mag_N * GF_PER_NEWTON)
                v3 = Vector3Stamped(); v3.header.stamp = t; v3.header.frame_id = "world"
                v3.vector.x, v3.vector.y, v3.vector.z = F3_vec_gf.tolist()
                m3 = Float32(); m3.data = F3_mag_gf
                outbag.write("/tension_gf/cf2/vector", v3, t)
                outbag.write("/tension_gf/cf2/magnitude", m3, t)


def main():
    ap = argparse.ArgumentParser(description="Copy all topics and ADD tension_gf using F = k*(r - L0) + b.")
    ap.add_argument("-p", "--pattern", default="24g_T?_index9.bag",
                    help="입력 bag 글롭 패턴 (기본: %(default)s)")
    ap.add_argument("--suffix", default="_tension",
                    help="출력 파일 접미사(확장자 앞, 기본: %(default)s)")
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
        process_one_file(f, out_path)

    print("[✓] Done.")


if __name__ == "__main__":
    main()
