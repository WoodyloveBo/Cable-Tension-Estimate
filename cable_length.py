#!/usr/bin/env python3
import rosbag
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped

# --- 설정 ---
INPUT_BAG   = '20g_T3.bag'
OUTPUT_BAG  = '20g_T3_lengths.bag'
SYNC_THRESH = 0.05  # s


def find_nearest(t, lst):
    '''(time, msg) 리스트 lst 중 t에 가장 가까운 항목을 반환''' 
    if not lst:
        return None
    times = np.abs(np.array([t - ti for ti,_ in lst]))
    idx = int(times.argmin())
    return lst[idx]


def main():
    rospy.init_node('compute_cable_length', anonymous=True)

    # 메시지 수집용 리스트
    cf4_poses = []    # [(time_sec, PoseStamped), ...]
    cf1_poses = []    # [(time_sec, PoseStamped), ...]
    payloads  = []    # [(time_sec, PointCloud), ...]

    # 입력 bag에서 토픽 읽기
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

    # 출력 bag 작성
    outbag = rosbag.Bag(OUTPUT_BAG, 'w')
    try:
        for t_pl, pl_msg in payloads:
            # 가장 가까운 드론 위치 동기화
            ent4 = find_nearest(t_pl, cf4_poses)
            ent1 = find_nearest(t_pl, cf1_poses)
            if not ent4 or not ent1 or not pl_msg.points:
                continue
            _, pose4 = ent4
            _, pose1 = ent1

            # 페이로드 위치 (첫 점)
            p_pl = np.array([pl_msg.points[0].x,
                             pl_msg.points[0].y,
                             pl_msg.points[0].z])

            # 드론 위치
            p4 = np.array([pose4.pose.position.x,
                           pose4.pose.position.y,
                           pose4.pose.position.z])
            p1 = np.array([pose1.pose.position.x,
                           pose1.pose.position.y,
                           pose1.pose.position.z])

            # 케이블 길이 계산
            L4 = float(np.linalg.norm(p4 - p_pl))
            L1 = float(np.linalg.norm(p1 - p_pl))

            # 메시지 작성
            msg4 = Float32(data=L4)
            msg1 = Float32(data=L1)
            stamp = rospy.Time.from_sec(t_pl)

            outbag.write('/cf4/cable_length', msg4, stamp)
            outbag.write('/cf1/cable_length', msg1, stamp)

    finally:
        outbag.close()
        print(f"Wrote cable lengths to {OUTPUT_BAG}")

if __name__ == '__main__':
    main()
