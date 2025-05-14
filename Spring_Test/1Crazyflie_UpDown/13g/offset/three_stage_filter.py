#!/usr/bin/env python
import rosbag
import rospy
import numpy as np
from scipy.signal import butter, lfilter, medfilt

def butter_lowpass(cutoff, fs, order=2):
    """
    Butterworth 로우패스 필터 계수를 계산합니다.
    :param cutoff: 컷오프 주파수 (Hz)
    :param fs: 샘플링 주파수 (Hz)
    :param order: 필터 차수
    :return: 필터 계수 b, a
    """
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def moving_average(data, window_size):
    """
    Moving Average 필터를 적용합니다.
    :param data: 입력 데이터 (numpy array)
    :param window_size: 윈도우 크기 (예: 5)
    :return: 필터링된 데이터
    """
    return np.convolve(data, np.ones(window_size)/window_size, mode='same')

def main():
    input_bag = '13g_T4_offset.bag'
    output_bag = '13g_T4_Three_stage_filtered.bag'
    
    # Butterworth 필터 파라미터 설정
    fs = 67.78      # 메시지 샘플링 주파수 (Hz)
    cutoff = 5.0    # 컷오프 주파수를 5Hz로 설정
    order = 2
    b, a = butter_lowpass(cutoff, fs, order=order)
    
    # Median 필터와 Moving Average 필터의 윈도우 크기 설정 (홀수여야 함)
    median_window_size = 11
    moving_average_window_size = 10
    
    # /cf4/log1 메시지 저장 (각 원소: (t, msg))
    cf4_msgs = []
    # 다른 토픽 메시지는 그대로 복사하기 위해 저장 (각 원소: (topic, msg, t))
    other_msgs = []
    
    rospy.loginfo("입력 bag 파일 '%s'에서 메시지 읽기 시작...", input_bag)
    inbag = rosbag.Bag(input_bag, 'r')
    for topic, msg, t in inbag.read_messages():
        if topic == "/cf4/log1":
            cf4_msgs.append((t, msg))
        else:
            other_msgs.append((topic, msg, t))
    inbag.close()
    rospy.loginfo("총 /cf4/log1 메시지 개수: %d", len(cf4_msgs))
    
    # /cf4/log1 메시지가 없으면 그대로 복사
    if len(cf4_msgs) == 0:
        rospy.logwarn("입력 bag 파일에 /cf4/log1 토픽 메시지가 없습니다.")
        outbag = rosbag.Bag(output_bag, 'w')
        inbag = rosbag.Bag(input_bag, 'r')
        for topic, msg, t in inbag.read_messages():
            outbag.write(topic, msg, t)
        inbag.close()
        outbag.close()
        return
    
    # 메시지를 시간 순으로 정렬
    cf4_msgs.sort(key=lambda x: x[0])
    times = [t.to_sec() if hasattr(t, "to_sec") else float(t) for t, _ in cf4_msgs]
    
    # 각 메시지에서 가속도 값 추출 (values[1],[2],[3])
    ax_list = []
    ay_list = []
    az_list = []
    for t, msg in cf4_msgs:
        if not hasattr(msg, 'values') or len(msg.values) < 4:
            rospy.logwarn("메시지에 4개 미만의 values가 있습니다.")
            ax_list.append(0.0)
            ay_list.append(0.0)
            az_list.append(0.0)
        else:
            # 가속도: [1],[2],[3] (index 0는 thrust일 가능성이 있음)
            ax_list.append(msg.values[1])
            ay_list.append(msg.values[2])
            az_list.append(msg.values[3])
    
    # numpy 배열로 변환
    ax_array = np.array(ax_list)
    ay_array = np.array(ay_list)
    az_array = np.array(az_list)
    
    # 1. Butterworth 필터 적용 (컷오프 5Hz)
    butter_ax = lfilter(b, a, ax_array)
    butter_ay = lfilter(b, a, ay_array)
    butter_az = lfilter(b, a, az_array)
    
    # 2. Sliding Median 필터 적용
    median_ax = medfilt(butter_ax, kernel_size=median_window_size)
    median_ay = medfilt(butter_ay, kernel_size=median_window_size)
    median_az = medfilt(butter_az, kernel_size=median_window_size)
    
    # 3. Moving Average 필터 적용
    final_ax = moving_average(median_ax, moving_average_window_size)
    final_ay = moving_average(median_ay, moving_average_window_size)
    final_az = moving_average(median_az, moving_average_window_size)
    
    # 메시지의 가속도 값 업데이트 (msg.values의 [1],[2],[3]에 필터링 결과 대입)
    for i, (t, msg) in enumerate(cf4_msgs):
        values_list = list(msg.values)
        values_list[1] = final_ax[i]
        values_list[2] = final_ay[i]
        values_list[3] = final_az[i]
        msg.values = values_list
        cf4_msgs[i] = (t, msg)
    
    # 새로운 bag 파일 기록:
    #  - 다른 토픽은 그대로 기록
    #  - /cf4/log1 메시지는 새로운 토픽 "/cf4/log1_filtered_butterworth_median_movingAverage"로 기록
    outbag = rosbag.Bag(output_bag, 'w')
    for topic, msg, t in other_msgs:
        outbag.write(topic, msg, t)
    for t, msg in cf4_msgs:
        outbag.write("/cf4/log1_three_stage_filtered/T1", msg, t)
    
    outbag.close()
    rospy.loginfo("새로운 bag 파일 '%s'가 생성되었습니다.", output_bag)

if __name__ == '__main__':
    rospy.init_node('acceleration_combined_filter', anonymous=True)
    main()
