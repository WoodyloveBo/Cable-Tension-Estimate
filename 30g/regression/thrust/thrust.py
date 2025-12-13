#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rosbag
import numpy as np
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from sensor_msgs.msg import PointCloud
import sys

# ===================== 사용자 설정 =====================
IN_BAG          = 'alpha_index9.bag'
OUT_BAG         = 'alpha_index9_ftB.bag'
CF_LIST         = ['cf1', 'cf2']

POINT_TOPIC     = '/point_index9'             # PointCloud (points[0] 사용)
POSE_TOPIC_FMT  = '/natnet_ros/{cf}/pose'     # PoseStamped (World, NatNet)
LOG1_TOPIC_FMT  = '/{cf}/log1'                # values[1:3] = m*a^W (gf)  ← 기존 정의 유지

# —— 스프링 파라미터 (참조코드 방식 동일) ——
K_N_PER_M       = 0.7686                      # [N/m]
B_N             = 0.0448                      # [N]
L0_CF           = {'cf1': 0.205, 'cf2': 0.205}  # [m]
CABLE_CLAMP     = True                        # True면 x<0 → x=0 (텐션만)

# —— 중력항 (월드 프레임, gf) ——
MG_W_GF         = np.array([0.0, 0.0, 40.0])  # [gf]

# —— 동기화 ——
SYNC_THRESHOLD  = 0.03                        # [s] 최인접 매칭 허용오차

# —— 단위변환 ——
NEWTON_PER_GF   = 0.00980665
GF_PER_NEWTON   = 1.0 / NEWTON_PER_GF

# (필요 시) NatNet 좌표가 mm라면 아래 스케일만 1e-3로 바꿔주세요.
POS_SCALE_TO_M  = 1.0                         # 좌표 단위 → meter (m일 때 1.0, mm일 때 1e-3)

# ===================== 수학 유틸 =====================
def quat_to_R_body_to_world(qx, qy, qz, qw):
    """ PoseStamped.orientation → R_BW (Body→World) """
    n2 = qx*qx + qy*qy + qz*qz + qw*qw
    if n2 <= 1e-20:
        return np.eye(3)
    s = 2.0 / n2
    x, y, z, w = qx, qy, qz, qw
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    R = np.array([
        [1 - s*(yy+zz),     s*(xy - wz),     s*(xz + wy)],
        [    s*(xy + wz), 1 - s*(xx+zz),     s*(yz - wx)],
        [    s*(xz - wy),     s*(yz + wx), 1 - s*(xx+yy)]
    ], dtype=float)
    return R

def nearest_index(t, arr_sorted):
    """ 정렬된 시간배열에서 t와 가장 가까운 인덱스 """
    import bisect
    i = bisect.bisect_left(arr_sorted, t)
    if i == 0: return 0
    if i == len(arr_sorted): return len(arr_sorted)-1
    return i if abs(arr_sorted[i]-t) < abs(arr_sorted[i-1]-t) else i-1

# ===================== 로더 =====================
def load_point_series(bag):
    """ /point_index9 → (ts_float[], ts_ros[], posW_raw[]) """
    ts_f, ts_ros, pos = [], [], []
    for _, msg, t in bag.read_messages(topics=[POINT_TOPIC]):
        if not msg.points:
            continue
        p = msg.points[0]
        ts_f.append(t.to_sec())
        ts_ros.append(t)
        pos.append([p.x, p.y, p.z])
    return np.array(ts_f), ts_ros, np.array(pos, dtype=float)

def load_pose_series(bag, cf):
    """ /natnet_ros/{cf}/pose → (ts[], posW_raw[], R_BW_list[]) """
    topic = POSE_TOPIC_FMT.format(cf=cf)
    ts, pos, Rbw = [], [], []
    for _, msg, t in bag.read_messages(topics=[topic]):
        ts.append(t.to_sec())
        pos.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = msg.pose.orientation
        R_BW = quat_to_R_body_to_world(q.x, q.y, q.z, q.w)
        Rbw.append(R_BW)
    return np.array(ts), np.array(pos, dtype=float), np.array(Rbw)

def load_maw_series(bag, cf):
    """
    /{cf}/log1 → values[1],values[2],values[3] = m*a^W (gf)
    반환: (ts[], mawW_gf[])
    """
    topic = LOG1_TOPIC_FMT.format(cf=cf)
    ts, vec = [], []
    for _, msg, t in bag.read_messages(topics=[topic]):
        vals = getattr(msg, 'values', None)
        if vals is None or len(vals) < 4:
            continue
        ts.append(t.to_sec())
        vec.append([float(vals[1]), float(vals[2]), float(vals[3])])  # gf
    return np.array(ts), np.array(vec, dtype=float)

# ===================== 스프링력 (참조코드 방식 동일) =====================
def spring_force_world_gf(p_cf_W_m, p_pt_W_m, L0_m):
    """
    참조코드:
      F_N = K*(r - L0) + B (x<0일 때 x=0 옵션)
      F_vec_N = F_N * (d/r),  이후 gf로 변환
    여기서는:
      - 좌표는 meter 사용 (POS_SCALE_TO_M 적용)
      - 최종은 gf 벡터 반환
    """
    d = (p_cf_W_m - p_pt_W_m)
    r = float(np.linalg.norm(d))
    if r < 1e-12:
        return np.zeros(3)
    u = d / r
    x = r - L0_m
    if CABLE_CLAMP:
        x = max(0.0, x)
    F_N  = K_N_PER_M * x + B_N               # [N]
    if CABLE_CLAMP and F_N < 0.0:
        F_N = 0.0
    F_gf = F_N * GF_PER_NEWTON               # [gf]
    return F_gf * u                          # [gf] (World)

# ===================== 계산 및 기록 =====================
def compute_and_write(inbag, outbag):
    # 기준 타임라인: point_index9 (원본 타임스탬프 보존)
    pt_ts_f, pt_ts_ros, pt_posW_raw = load_point_series(inbag)
    if pt_ts_f.size == 0:
        print("[!] No /point_index9 data found. Abort.")
        return

    # 원본 전부 복사
    for topic, msg, t in inbag.read_messages():
        outbag.write(topic, msg, t)

    # 선계산 시계열 적재
    pose_data, maw_data = {}, {}
    for cf in CF_LIST:
        pose_ts, pose_posW_raw, pose_RBW = load_pose_series(inbag, cf)
        maw_ts,  mawW_gf                 = load_maw_series(inbag, cf)
        pose_data[cf] = (pose_ts, pose_posW_raw, pose_RBW)
        maw_data[cf]  = (maw_ts,  mawW_gf)

    # 포인트 타임라인 순회하며 cf1, cf2 각각 계산
    for cf in CF_LIST:
        pose_ts, pose_posW_raw, pose_RBW = pose_data[cf]
        maw_ts,  mawW_gf                 = maw_data[cf]

        if pose_ts.size == 0 or maw_ts.size == 0:
            print(f"[{cf}] Missing pose/log1. Skip.")
            continue

        L0 = L0_CF.get(cf, 0.205)

        for k in range(len(pt_ts_f)):
            t_ref_f   = pt_ts_f[k]
            t_ref_ros = pt_ts_ros[k]

            # pose / maW 최근접 매칭
            jp = nearest_index(t_ref_f, pose_ts)
            jm = nearest_index(t_ref_f, maw_ts)
            if abs(pose_ts[jp] - t_ref_f) > SYNC_THRESHOLD:  # 시간 불일치 시 스킵
                continue
            if abs(maw_ts[jm]  - t_ref_f) > SYNC_THRESHOLD:
                continue

            # ---- 데이터 구성 ----
            p_cf_W_raw  = pose_posW_raw[jp]
            R_BW        = pose_RBW[jp]          # Body→World
            R_WB        = R_BW.T                # World→Body
            p_pt_W_raw  = pt_posW_raw[k]
            maW_gf      = mawW_gf[jm]          # [gf] (월드 프레임)

            # 좌표를 meter로
            p_cf_W_m = p_cf_W_raw * POS_SCALE_TO_M
            p_pt_W_m = p_pt_W_raw * POS_SCALE_TO_M

            # (1) m a^B : 월드→바디
            maB_gf = R_WB.dot(maW_gf)

            # (2) m g^B
            mgB_gf = R_WB.dot(MG_W_GF)

            # (3) f_s^W (gf) — 참조코드 방식 (K in N/m, B in N → N→gf 변환)
            fsW_gf = spring_force_world_gf(p_cf_W_m, p_pt_W_m, L0)

            # (4) f_s^B
            fsB_gf = R_WB.dot(fsW_gf)

            # (5) f_t^B = ma^B + mg^B + f_s^B
            ftB_gf = maB_gf + mgB_gf + fsB_gf

            # ===== 기록 =====
            def write_v3(topic, vec_gf, frame_id):
                msg = Vector3Stamped()
                msg.header.stamp = t_ref_ros         # point_index9와 동일 timestamp
                msg.header.frame_id = frame_id
                msg.vector.x, msg.vector.y, msg.vector.z = map(float, vec_gf)
                outbag.write(topic, msg, t_ref_ros)

            frame_body  = f'{cf}/body'
            frame_world = f'{cf}/world'

            write_v3(f'/{cf}/ft_b', ftB_gf, frame_body)
            write_v3(f'/{cf}/ma_b', maB_gf, frame_body)
            write_v3(f'/{cf}/mg_b', mgB_gf, frame_body)
            write_v3(f'/{cf}/fs_b', fsB_gf, frame_body)
            write_v3(f'/{cf}/fs_w', fsW_gf, frame_world)  # 월드 스프링력도 보관(검증용)

# ===================== 엔트리 =====================
def main():
    try:
        inbag  = rosbag.Bag(IN_BAG,  'r')
    except Exception as e:
        print(f"[!] Failed to open IN_BAG: {e}")
        sys.exit(1)

    try:
        outbag = rosbag.Bag(OUT_BAG, 'w')
    except Exception as e:
        print(f"[!] Failed to open OUT_BAG for writing: {e}")
        inbag.close()
        sys.exit(1)

    try:
        compute_and_write(inbag, outbag)
    finally:
        outbag.close()
        inbag.close()
        print(f"[✓] Wrote {OUT_BAG}")

if __name__ == '__main__':
    main()
