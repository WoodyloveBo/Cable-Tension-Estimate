#!/usr/bin/env python
import rosbag
import rospy
import math
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import PointCloud

def compute_cf4_to_point_distance():
    input_bag   = '12g_T1_updown.bag'
    output_bag  = 'distance_updown.bag'
    time_tol    = 0.01  # 초 단위 매칭 허용 오차

    latest_cf4   = None  # (PoseStamped, rospy.Time)
    latest_point = None  # (PointCloud, rospy.Time)

    # 쓰기／읽기 모드로 동시에 열기
    with rosbag.Bag(output_bag, 'w') as outbag, rosbag.Bag(input_bag, 'r') as inbag:
        for topic, msg, t in inbag.read_messages(
                topics=['/natnet_ros/cf4/pose', '/point_index4']):
            
            # CF4 포즈 수신 시 저장
            if topic == '/natnet_ros/cf4/pose':
                latest_cf4 = (msg, t)

            # point_index4 수신 시(비어있지 않으면) 저장
            elif topic == '/point_index4' and msg.points:
                latest_point = (msg, t)

            # 두 메시지가 모두 준비되었으면 타임스탬프 차이 검사
            if latest_cf4 and latest_point:
                t_cf4, t_pt = latest_cf4[1].to_sec(), latest_point[1].to_sec()
                if abs(t_cf4 - t_pt) <= time_tol:
                    # 위치 데이터 추출
                    p_cf4 = latest_cf4[0].pose.position
                    p_pt  = latest_point[0].points[0]

                    # 유클리드 거리 계산
                    dist = math.sqrt(
                        (p_cf4.x - p_pt.x)**2 +
                        (p_cf4.y - p_pt.y)**2 +
                        (p_cf4.z - p_pt.z)**2
                    )

                    # 평균 타임스탬프로 header 설정
                    avg_time = rospy.Time.from_sec((t_cf4 + t_pt) / 2.0)

                    # PointStamped 메시지 생성
                    dist_msg = PointStamped()
                    dist_msg.header.stamp    = avg_time
                    dist_msg.header.frame_id = latest_cf4[0].header.frame_id
                    dist_msg.point.x = dist
                    dist_msg.point.y = 0.0
                    dist_msg.point.z = 0.0

                    # 새 토픽에 기록
                    outbag.write('/cf4_point_distance', dist_msg, avg_time)

                    # 중복 계산 방지
                    latest_cf4 = None
                    latest_point = None

if __name__ == '__main__':
    compute_cf4_to_point_distance()
