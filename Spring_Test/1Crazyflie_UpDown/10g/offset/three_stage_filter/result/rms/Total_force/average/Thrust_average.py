#!/usr/bin/env python
import rosbag
import rospy
import sys
from geometry_msgs.msg import Vector3Stamped

def process_hovering_force(input_bag_path, output_bag_path, hover_start, hover_end):
    """
    - input_bag_path: 원본 bag 파일 경로
    - output_bag_path: 결과 bag 파일 경로
    - hover_start, hover_end: 호버링 구간 (상대 시간, 초 단위)

    /cf4/total_force 토픽의 Vector3Stamped.vector.x 값을 평균 계산합니다.
    결과:
      - /cf4/total_force : 호버링 구간 내 메시지를 상대시간 기준으로 저장
      - /cf4/total_force_avg : 호버링 구간 평균 값 저장
    """
    values = []
    filtered = []  # List of tuples (ros.Time, value, frame_id)
    ref_stamp = None

    # 1) /cf4/total_force 메시지 수집 및 필터링
    with rosbag.Bag(input_bag_path, 'r') as inbag:
        for topic, msg, _ in inbag.read_messages(topics=['/cf4/total_force']):
            if ref_stamp is None:
                ref_stamp = msg.header.stamp
            # 상대 시간 계산
            rel_duration = msg.header.stamp - ref_stamp
            rel_sec = rel_duration.to_sec()
            if hover_start <= rel_sec <= hover_end:
                val = msg.vector.z
                values.append(val)
                filtered.append((rel_duration, val, msg.header.frame_id))

    if not values:
        rospy.logerr("지정된 구간 (%.2f ~ %.2f) 내에 /cf4/total_force 메시지가 없습니다.", hover_start, hover_end)
        return

    avg_val = sum(values) / len(values)
    rospy.loginfo("호버링 구간 평균 total_force: %.4f", avg_val)

    # 2) 결과 기록
    with rosbag.Bag(output_bag_path, 'w') as outbag:
        # 호버링 구간 메시지 저장
        for rel_time, val, frame in filtered:
            out_msg = Vector3Stamped()
            out_msg.header.stamp = rel_time
            out_msg.header.frame_id = frame
            out_msg.vector.x = 0.0
            out_msg.vector.y = 0.0
            out_msg.vector.z = val
            outbag.write('/cf4/total_force', out_msg, rel_time)

                # 평균 메시지 저장
        avg_duration = rospy.Duration(0)
        avg_msg = Vector3Stamped()
        avg_msg.header.stamp = rospy.Time(0)
        avg_msg.header.frame_id = ''
        avg_msg.vector.x = 0.0
        avg_msg.vector.y = 0.0
        avg_msg.vector.z = avg_val
        outbag.write('/cf4/total_force_avg', avg_msg, avg_duration)('/cf4/total_force_avg', avg_msg, rospy.Time(0))

    rospy.loginfo("결과가 '%s'에 저장되었습니다.", output_bag_path)

if __name__ == '__main__':
    rospy.init_node('hover_total_force_average', anonymous=True)
    if len(sys.argv) != 5:
        print("Usage: %s <input.bag> <output.bag> <hover_start> <hover_end>" % sys.argv[0])
        sys.exit(1)
    process_hovering_force(
        sys.argv[1], sys.argv[2],
        float(sys.argv[3]), float(sys.argv[4])
    )
