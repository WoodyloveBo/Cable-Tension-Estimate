#!/usr/bin/env python
import rosbag
import rospy
import glob
import sys
import copy

def convert_to_relative_time(input_bag_path, output_bag_path, topics, time_offset):
    """
    지정된 topics의 메시지 타임스탬프를
    첫 메시지 배치 시각(ref_stamp)부터 0초로 시작하는 상대 시간으로 변환하고,
    time_offset(초)만큼 추가 보정하여 기록합니다.
    """
    messages = []
    ref_stamp = None

    # 1) 수집
    with rosbag.Bag(input_bag_path, 'r') as inbag:
        for topic, msg, t in inbag.read_messages(topics=topics):
            if ref_stamp is None:
                ref_stamp = t
            messages.append((topic, copy.deepcopy(msg), t))

    if ref_stamp is None:
        rospy.logerr("입력 bag에 지정된 토픽이 없습니다: %s", topics)
        return

    offset = rospy.Duration(time_offset)

    # 2) 기록
    with rosbag.Bag(output_bag_path, 'w') as outbag:
        for topic, msg, t in messages:
            # 원본 t에서 ref_stamp 빼고 offset 더하면 Duration
            rel_dur = (t - ref_stamp) + offset

            # 음수면 0으로 클램프
            if rel_dur.to_sec() < 0.0:
                new_stamp = rospy.Time(0, 0)
            else:
                new_stamp = rospy.Time(rel_dur.secs, rel_dur.nsecs)

            # header가 있으면 갱신
            if hasattr(msg, 'header'):
                msg.header.stamp = new_stamp

            # 기록
            outbag.write(topic, msg, new_stamp)

    rospy.loginfo(
        "Processed '%s' → '%s' (offset %+0.3fs)",
        input_bag_path, output_bag_path, time_offset
    )


if __name__ == '__main__':
    rospy.init_node('batch_timestamp_converter', anonymous=True)

    # 입력 파일 패턴
    total_files = sorted(glob.glob("19g_T?_total.bag"))
    if not total_files:
        rospy.logerr("No files matching '19g_T?_total.bag' found.")
        sys.exit(1)

    # 변환 대상 토픽 목록
    topics = [
        '/cf4/acceleration_force',
        '/cf4/body_force',
        '/cf4/distance_scalar_force_rms',
        '/cf4/thrust',
        '/cf4/total_force'
    ]

    # 오프셋: +2.2초
    time_offset = 0

    for inp in total_files:
        out = inp.replace('_total.bag', '_timestamp.bag')
        try:
            convert_to_relative_time(inp, out, topics, time_offset)
        except Exception as e:
            rospy.logerr("Failed on %s: %s", inp, e)
