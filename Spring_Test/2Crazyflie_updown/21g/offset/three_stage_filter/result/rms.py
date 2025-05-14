#!/usr/bin/env python
import rosbag
import rospy
import glob
import sys
import numpy as np
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32

def compute_rms_scalar(values):
    """주어진 scalar 값 리스트에 대한 RMS를 계산합니다."""
    if not values:
        return 0.0
    arr = np.array(values, dtype=float)
    return np.sqrt(np.mean(arr ** 2))

def find_nearest(target_time, msg_list, threshold=0.1):
    """가장 가까운 메시지를 반환 (시간 차이가 threshold 이하인 경우만)."""
    nearest, min_dt = None, None
    for t, msg in msg_list:
        dt = abs((t - target_time).to_sec())
        if min_dt is None or dt < min_dt:
            min_dt, nearest = dt, (t, msg)
    return nearest if (min_dt is not None and min_dt <= threshold) else None

def process_bag(input_bag_path, output_bag_path, window_size=50, threshold=0.1):
    # 1) cf4 데이터 수집
    scalar_cf4 = []  # (ros.Time, Float32) /cf4/distance_scalar_force
    thr_cf4    = []  # (ros.Time, Vector3Stamped) /cf4/thrust
    # 2) cf1 데이터 수집
    scalar_cf1 = []  # /cf1/distance_scalar_force
    thr_cf1    = []  # /cf1/thrust

    # 수집 단계
    with rosbag.Bag(input_bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == '/cf4/distance_scalar_force':
                scalar_cf4.append((t, msg))
            elif topic == '/cf4/thrust':
                thr_cf4.append((t, msg))
            elif topic == '/cf1/distance_scalar_force':
                scalar_cf1.append((t, msg))
            elif topic == '/cf1/thrust':
                thr_cf1.append((t, msg))

    # 타임스탬프 순 정렬
    scalar_cf4.sort(key=lambda x: x[0])
    thr_cf4.sort(key=lambda x: x[0])
    scalar_cf1.sort(key=lambda x: x[0])
    thr_cf1.sort(key=lambda x: x[0])

    # 3) Rewrite bag: copy all topics, override body_force and append RMS topics
    with rosbag.Bag(input_bag_path, 'r') as inbag, \
         rosbag.Bag(output_bag_path, 'w') as outbag:

        # First pass: copy topics, override body_force for cf4 & cf1 when thrust == 0
        for topic, msg, t in inbag.read_messages():
            if topic == '/cf4/body_force':
                pair = find_nearest(t, thr_cf4, threshold)
                if pair and pair[1].vector.x == 0.0:
                    zero_msg = Vector3Stamped(header=msg.header, vector=msg.vector)
                    zero_msg.vector.x = zero_msg.vector.y = zero_msg.vector.z = 0.0
                    outbag.write(topic, zero_msg, t)
                else:
                    outbag.write(topic, msg, t)
            elif topic == '/cf1/body_force':
                pair = find_nearest(t, thr_cf1, threshold)
                if pair and pair[1].vector.x == 0.0:
                    zero_msg = Vector3Stamped(header=msg.header, vector=msg.vector)
                    zero_msg.vector.x = zero_msg.vector.y = zero_msg.vector.z = 0.0
                    outbag.write(topic, zero_msg, t)
                else:
                    outbag.write(topic, msg, t)
            else:
                outbag.write(topic, msg, t)

        # Second pass: cf4 rolling RMS
        buffer_cf4 = []
        for t, msg in scalar_cf4:
            buffer_cf4.append(msg.data)
            if len(buffer_cf4) > window_size:
                buffer_cf4.pop(0)
            thr_pair = find_nearest(t, thr_cf4, threshold)
            if thr_pair and thr_pair[1].vector.x == 0.0:
                rms_val = 0.0
            else:
                rms_val = compute_rms_scalar(buffer_cf4)
            rms_msg = Float32(data=float(rms_val))
            outbag.write('/cf4/distance_scalar_force_rms', rms_msg, t)

        # Third pass: cf1 rolling RMS
        buffer_cf1 = []
        for t, msg in scalar_cf1:
            buffer_cf1.append(msg.data)
            if len(buffer_cf1) > window_size:
                buffer_cf1.pop(0)
            thr_pair = find_nearest(t, thr_cf1, threshold)
            if thr_pair and thr_pair[1].vector.x == 0.0:
                rms_val = 0.0
            else:
                rms_val = compute_rms_scalar(buffer_cf1)
            rms_msg = Float32(data=float(rms_val))
            outbag.write('/cf1/distance_scalar_force_rms', rms_msg, t)

    rospy.loginfo("Finished processing '%s' → '%s'", input_bag_path, output_bag_path)


if __name__ == '__main__':
    rospy.init_node('batch_rms_processing', anonymous=True)

    # Match 9g_T1_rms.bag ... 9g_T5_rms.bag (or result.bag if naming differs)
    result_files = sorted(glob.glob("21g_T?_result.bag"))
    if not result_files:
        rospy.logerr("No files matching '21g_T?_result.bag' found.")
        sys.exit(1)

    for inp in result_files:
        out = inp.replace('_result.bag', '_rms.bag')
        try:
            process_bag(inp, out)
        except Exception as e:
            rospy.logerr("Failed on %s: %s", inp, e)
