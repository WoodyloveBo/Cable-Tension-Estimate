#!/usr/bin/env python
import rosbag
import rospy
import numpy as np
from scipy.signal import butter, lfilter, medfilt

def butter_lowpass(cutoff, fs, order=2):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def moving_average(data, window_size):
    return np.convolve(data, np.ones(window_size)/window_size, mode='same')

def filter_sequence(msg_list, b, a, median_k, ma_k):
    """
    msg_list: [(t, msg), ...] 에 들어있는 msg.values[1:4] 에 필터 적용 후 반환
    """
    # 1) 원신호 분리
    ax = np.array([m.values[1] if len(m.values)>3 else 0.0 for _,m in msg_list])
    ay = np.array([m.values[2] if len(m.values)>3 else 0.0 for _,m in msg_list])
    az = np.array([m.values[3] if len(m.values)>3 else 0.0 for _,m in msg_list])

    # 2) Butterworth
    ax_b = lfilter(b, a, ax)
    ay_b = lfilter(b, a, ay)
    az_b = lfilter(b, a, az)

    # 3) Median
    ax_m = medfilt(ax_b, kernel_size=median_k)
    ay_m = medfilt(ay_b, kernel_size=median_k)
    az_m = medfilt(az_b, kernel_size=median_k)

    # 4) Moving Average
    ax_f = moving_average(ax_m, ma_k)
    ay_f = moving_average(ay_m, ma_k)
    az_f = moving_average(az_m, ma_k)

    # 5) 원 msg_list 에 덮어쓰기
    for i, (t, msg) in enumerate(msg_list):
        vals = list(msg.values)
        vals[1], vals[2], vals[3] = ax_f[i], ay_f[i], az_f[i]
        msg.values = vals

    return msg_list

def main():
    input_bag  = '20g_T2_offset.bag'
    output_bag = '20g_T2_Three_stage_filtered.bag'

    # 필터 파라미터
    fs  = 67.78
    cutoff = 5.0
    order  = 2
    b, a = butter_lowpass(cutoff, fs, order)

    median_k = 11
    ma_k     = 10

    # 메시지 분류
    cf1_msgs = []
    cf4_msgs = []
    other_msgs = []

    rospy.loginfo("Reading '%s'...", input_bag)
    with rosbag.Bag(input_bag, 'r') as bag_in:
        for topic, msg, t in bag_in.read_messages():
            if topic == "/cf1/log1":
                cf1_msgs.append((t, msg))
            elif topic == "/cf4/log1":
                cf4_msgs.append((t, msg))
            else:
                other_msgs.append((topic, msg, t))

    # 필터링 적용
    if cf1_msgs:
        cf1_msgs = filter_sequence(cf1_msgs, b, a, median_k, ma_k)
    if cf4_msgs:
        cf4_msgs = filter_sequence(cf4_msgs, b, a, median_k, ma_k)

    # 출력
    rospy.loginfo("Writing filtered bag '%s'...", output_bag)
    with rosbag.Bag(output_bag, 'w') as bag_out:
        # 1) 나머지 원본 토픽 복사
        for topic, msg, t in other_msgs:
            bag_out.write(topic, msg, t)
        # 2) 필터링된 cf1, cf4 토픽 기록
        for t, msg in cf1_msgs:
            bag_out.write("/cf1/log1_three_stage_filtered", msg, t)
        for t, msg in cf4_msgs:
            bag_out.write("/cf4/log1_three_stage_filtered", msg, t)

    rospy.loginfo("Done.")

if __name__ == '__main__':
    rospy.init_node('acceleration_combined_filter', anonymous=True)
    main()
