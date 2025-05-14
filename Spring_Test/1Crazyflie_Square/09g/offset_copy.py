#!/usr/bin/env python
import rosbag
import rospy

def shift_record_times(input_bag_path, output_bag_path, offset_sec):
    """
    /cf4/log1 및 /cf4/pose 토픽의 메시지 기록 시간을 offset_sec(초) 만큼 앞으로 당겨서
    새로운 bag 파일에 저장합니다. (msg.header.stamp은 변경하지 않음)
    """
    inbag = rosbag.Bag(input_bag_path, 'r')
    outbag = rosbag.Bag(output_bag_path, 'w')
    offset = rospy.Duration(offset_sec)

    for topic, msg, t in inbag.read_messages():
        # 이 토픽들은 기록 시간을 당겨 씁니다.
        if topic in ['/cf4/log1', '/cf4/pose']:
            new_time = t - offset
            outbag.write(topic, msg, new_time)
        else:
            # 나머지 토픽은 원본 시간 그대로
            outbag.write(topic, msg, t)

    inbag.close()
    outbag.close()
    rospy.loginfo("'%s'와 '%s' 메시지 기록 시간을 %.2f초 앞당긴 '%s' 생성 완료",
                  '/cf4/log1', '/cf4/pose', offset_sec, output_bag_path)

if __name__ == '__main__':
    import sys
    rospy.init_node('shift_record_times', anonymous=True)
    if len(sys.argv) != 3:
        print("Usage: python shift_record_times.py <input.bag> <output.bag>")
        sys.exit(1)
    input_bag  = sys.argv[1]
    output_bag = sys.argv[2]
    # 기록 시간만 2.2초 앞당김
    shift_record_times(input_bag, output_bag, offset_sec=2.2)
