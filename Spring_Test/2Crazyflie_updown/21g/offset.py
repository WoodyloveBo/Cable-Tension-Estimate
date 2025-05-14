#!/usr/bin/env python3
import rosbag
import rospy
import sys


def shift_timestamps(input_bag_path, output_bag_path, offset_sec):
    """
    /cf4/log1, /cf4/pose, /cf1/log1, /cf1/pose 토픽의 header.stamp를
    offset_sec(초) 만큼 뒤로 민 후 새로운 bag 파일에 저장합니다.
    """
    # 입력/출력 bag 열기
    inbag  = rosbag.Bag(input_bag_path, 'r')
    outbag = rosbag.Bag(output_bag_path, 'w')
    offset = rospy.Duration(offset_sec)

    # 처리 대상 토픽 목록
    target_topics = ['/cf4/log1', '/cf4/pose', '/cf1/log1', '/cf1/pose']

    for topic, msg, t in inbag.read_messages():
        if topic in target_topics and hasattr(msg, 'header'):
            # header.stamp에서 offset만큼 뺌
            new_stamp = msg.header.stamp - offset
            msg.header.stamp = new_stamp
            # 기록 시에도 new_stamp 사용
            outbag.write(topic, msg, new_stamp)
        else:
            # 그 외 토픽은 원래 timestamp 그대로 복사
            outbag.write(topic, msg, t)

    inbag.close()
    outbag.close()

    rospy.loginfo(
        f"[Shifted] {target_topics} 헤더 타임스탬프를 {offset_sec:.2f}s 뒤로 밀어 '{output_bag_path}' 생성 완료"
    )


def main():
    rospy.init_node('shift_log1_and_pose_timestamps', anonymous=True)

    if len(sys.argv) != 3:
        print("Usage: python shift_timestamps.py <input.bag> <output.bag>")
        sys.exit(1)

    input_bag  = sys.argv[1]
    output_bag = sys.argv[2]

    # 헤더 타임스탬프를 2.2초 뒤로 밀어서 저장
    shift_timestamps(input_bag, output_bag, offset_sec=2.2)


if __name__ == '__main__':
    main()
