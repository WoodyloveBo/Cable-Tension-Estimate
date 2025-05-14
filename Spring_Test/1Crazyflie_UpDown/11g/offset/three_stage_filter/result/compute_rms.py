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
    nearest, min_dt = None, None
    for t, msg in msg_list:
        dt = abs((t - target_time).to_sec())
        if min_dt is None or dt < min_dt:
            min_dt, nearest = dt, (t, msg)
    return nearest if (min_dt is not None and min_dt <= threshold) else None

def process_bag(input_bag_path, output_bag_path, window_size=50, threshold=0.1):
    # Collect scalar distance_force and thrust messages
    scalar_msgs = []  # (ros.Time, Float32)
    thr_msgs    = []  # (ros.Time, Vector3Stamped)

    with rosbag.Bag(input_bag_path, 'r') as inbag:
        for topic, msg, t in inbag.read_messages():
            if topic == '/cf4/distance_scalar_force':
                scalar_msgs.append((t, msg))
            elif topic == '/cf4/thrust':
                thr_msgs.append((t, msg))
    scalar_msgs.sort(key=lambda x: x[0])
    thr_msgs.sort(key=lambda x: x[0])

    # Rewrite bag: copy all topics, override body_force when thrust == 0
    with rosbag.Bag(input_bag_path, 'r') as inbag, \
         rosbag.Bag(output_bag_path, 'w') as outbag:

        # First pass: copy topics, override body_force if needed
        for topic, msg, t in inbag.read_messages():
            if topic == '/cf4/body_force':
                pair = find_nearest(t, thr_msgs, threshold)
                if pair and pair[1].vector.x == 0.0:
                    zero_msg = Vector3Stamped()
                    zero_msg.header = msg.header
                    zero_msg.vector.x = 0.0
                    zero_msg.vector.y = 0.0
                    zero_msg.vector.z = 0.0
                    outbag.write(topic, zero_msg, t)
                else:
                    outbag.write(topic, msg, t)
            else:
                outbag.write(topic, msg, t)

        # Second pass: rolling RMS over scalar distance force
        buffer = []
        for t, msg in scalar_msgs:
            buffer.append(msg.data)
            if len(buffer) > window_size:
                buffer.pop(0)

            thr_pair = find_nearest(t, thr_msgs, threshold)
            if thr_pair and thr_pair[1].vector.x == 0.0:
                rms_val = 0.0
            else:
                rms_val = compute_rms_scalar(buffer)

            rms_msg = Float32()
            rms_msg.data = float(rms_val)
            outbag.write('/cf4/distance_scalar_force_rms', rms_msg, t)

    rospy.loginfo("Finished processing '%s' → '%s'", input_bag_path, output_bag_path)

if __name__ == '__main__':
    rospy.init_node('batch_rms_processing', anonymous=True)

    # Match 9g_T1_result.bag ... 9g_T5_result.bag
    result_files = sorted(glob.glob("11g_T?_result.bag"))
    if not result_files:
        rospy.logerr("No files matching '11g_T?_result.bag' found.")
        sys.exit(1)

    for inp in result_files:
        out = inp.replace('_result.bag', '_rms.bag')
        try:
            process_bag(inp, out)
        except Exception as e:
            rospy.logerr("Failed on %s: %s", inp, e)

