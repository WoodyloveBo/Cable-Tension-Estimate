#!/usr/bin/env python
import rosbag
import rospy

def shift_timestamps(input_bag_path, output_bag_path, offset_sec):
    """
    /cf4/log1 및 /cf4/pose 토픽의 header.stamp를 offset_sec(초) 만큼 뺀 후
    새로운 bag 파일에 저장합니다.
    """
    inbag = rosbag.Bag(input_bag_path, 'r')
    outbag = rosbag.Bag(output_bag_path, 'w')
    offset = rospy.Duration(offset_sec)

    for topic, msg, t in inbag.read_messages():
        if topic in ['/cf4/log1', '/cf4/pose']:
            # header.stamp에서 offset만큼 뺌
            new_stamp = msg.header.stamp - offset
            msg.header.stamp = new_stamp
            # 기록 시시간에도 new_stamp 사용
            outbag.write(topic, msg, new_stamp)
        else:
            # 기타 토픽은 그대로 복사
            outbag.write(topic, msg, t)

    inbag.close()
    outbag.close()
    rospy.loginfo("'%s'와 '%s' 헤더 타임스탬프를 %d초 만큼 뒤로 민 새로운 bag 파일 '%s' 생성 완료",
                  '/cf4/log1', '/cf4/pose', offset_sec, output_bag_path)

if __name__ == '__main__':
    import sys
    rospy.init_node('shift_log1_and_pose_timestamps', anonymous=True)
    if len(sys.argv) != 3:
        print("Usage: python shift_timestamps.py <input.bag> <output.bag>")
        sys.exit(1)
    input_bag = sys.argv[1]
    output_bag = sys.argv[2]
    # 헤더 타임스탬프를 1초 빼기
    shift_timestamps(input_bag, output_bag, offset_sec=2)
