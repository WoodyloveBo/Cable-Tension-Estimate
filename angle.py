#!/usr/bin/env python3
import rosbag
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped

# 설정
INPUT_BAG   = '20g_T3.bag'
OUTPUT_BAG  = '20g_T3_angles.bag'
SYNC_THRESH = 0.05  # 동기화 최대 허용 오차 (초)

def find_nearest(t, lst):
    """(time, msg) 리스트 lst 중 t에 가장 가까운 항목을 반환"""
    if not lst:
        return None
    times = np.array([abs(t - ti) for ti, _ in lst])
    idx = int(times.argmin())
    return lst[idx]

def compute_angle(p_drone, p_pl):
    """
    드론-페이로드 벡터와 수직축(z) 사이의 각도를 라디안 단위로 반환
    """
    v = p_drone - p_pl
    L = np.linalg.norm(v)
    if L < 1e-6:
        return None
    cos_theta = np.clip(v[2] / L, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    return theta  # radians

def main():
    rospy.init_node('compute_cable_angles', anonymous=True)

    cf4_poses = []   # [(sec, PoseStamped), ...]
    cf1_poses = []
    payloads  = []   # [(sec, PointCloud), ...]

    # 입력 bag에서 메시지 수집
    with rosbag.Bag(INPUT_BAG, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            sec = t.to_sec()
            if topic == '/natnet_ros/cf4/pose':
                cf4_poses.append((sec, msg))
            elif topic == '/natnet_ros/cf1/pose':
                cf1_poses.append((sec, msg))
            elif topic == '/point_index9':
                payloads.append((sec, msg))

    # 시간순 정렬
    cf4_poses.sort(key=lambda x: x[0])
    cf1_poses.sort(key=lambda x: x[0])
    payloads .sort(key=lambda x: x[0])

    # 출력 bag 생성
    outbag = rosbag.Bag(OUTPUT_BAG, 'w')
    try:
        for t_pl, pl_msg in payloads:
            # 동기화
            e4 = find_nearest(t_pl, cf4_poses)
            e1 = find_nearest(t_pl, cf1_poses)
            if not e4 or not e1 or not pl_msg.points:
                continue
            _, msg4 = e4
            _, msg1 = e1

            # 위치 추출
            p_pl = np.array([pl_msg.points[0].x,
                             pl_msg.points[0].y,
                             pl_msg.points[0].z])
            p4   = np.array([msg4.pose.position.x,
                             msg4.pose.position.y,
                             msg4.pose.position.z])
            p1   = np.array([msg1.pose.position.x,
                             msg1.pose.position.y,
                             msg1.pose.position.z])

            # 각도 계산 (라디안 → 도 단위 변환)
            th4 = compute_angle(p4, p_pl)
            th1 = compute_angle(p1, p_pl)
            if th4 is None or th1 is None:
                continue
            deg4 = float(np.degrees(th4))
            deg1 = float(np.degrees(th1))

            # 메시지 작성 및 기록
            stamp = rospy.Time.from_sec(t_pl)
            outbag.write('/cf4/angle', Float32(data=deg4), stamp)
            outbag.write('/cf1/angle', Float32(data=deg1), stamp)

    finally:
        outbag.close()
        print(f"Wrote cable angles (deg) to {OUTPUT_BAG}")

if __name__ == '__main__':
    main()
