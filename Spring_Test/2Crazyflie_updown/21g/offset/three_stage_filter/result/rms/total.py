#!/usr/bin/env python
import rosbag
import rospy
import glob
import sys
import numpy as np
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32

def pwm_to_thrust(pwm, pwm_min=10000, pwm_max=60000):
    """
    PWM 값을 실제 추력(T) 값으로 변환합니다.
    a, b, c 는 2차 회귀 계수입니다.
    """
    a, b, c = 2.82e-08, -0.000937, 42.45
    if pwm < pwm_min:
        return 0.0
    pwm = min(pwm, pwm_max)
    T = a * pwm**2 + b * pwm + c
    return max(T, 0.0)

def find_nearest(target_time, msg_list, threshold=0.1):
    """
    가장 가까운 메시지를 반환합니다.
    시간 차이가 threshold 이하인 경우에만 유효합니다.
    """
    nearest = None
    min_dt = None
    for t, msg in msg_list:
        dt = abs((t - target_time).to_sec())
        if min_dt is None or dt < min_dt:
            min_dt, nearest = dt, (t, msg)
    return nearest if (min_dt is not None and min_dt <= threshold) else None

def process_bag(input_bag_path, output_bag_path, threshold=0.1):
    """
    cf4 및 cf1 의 가속도, body force, 거리 기반 RMS force를 합산하여
    total_force를 생성합니다.
    """
    # 메시지 수집
    acc_cf4    = []  # /cf4/acceleration_force
    body_cf4   = []  # /cf4/body_force
    scalar_cf4 = []  # /cf4/distance_scalar_force_rms
    thr_cf4    = []  # /cf4/thrust
    acc_cf1    = []  # /cf1/acceleration_force
    body_cf1   = []  # /cf1/body_force
    scalar_cf1 = []  # /cf1/distance_scalar_force_rms
    thr_cf1    = []  # /cf1/thrust

    with rosbag.Bag(input_bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == '/cf4/acceleration_force':
                acc_cf4.append((t, msg))
            elif topic == '/cf4/body_force':
                body_cf4.append((t, msg))
            elif topic == '/cf4/distance_scalar_force_rms':
                scalar_cf4.append((t, msg))
            elif topic == '/cf4/thrust':
                thr_cf4.append((t, msg))
            elif topic == '/cf1/acceleration_force':
                acc_cf1.append((t, msg))
            elif topic == '/cf1/body_force':
                body_cf1.append((t, msg))
            elif topic == '/cf1/distance_scalar_force_rms':
                scalar_cf1.append((t, msg))
            elif topic == '/cf1/thrust':
                thr_cf1.append((t, msg))

    # 시간순 정렬
    acc_cf4.sort(key=lambda x: x[0])
    body_cf4.sort(key=lambda x: x[0])
    scalar_cf4.sort(key=lambda x: x[0])
    thr_cf4.sort(key=lambda x: x[0])
    acc_cf1.sort(key=lambda x: x[0])
    body_cf1.sort(key=lambda x: x[0])
    scalar_cf1.sort(key=lambda x: x[0])
    thr_cf1.sort(key=lambda x: x[0])

    # 새 bag 작성
    with rosbag.Bag(output_bag_path, 'w') as outbag:
        # 원본 복사 (PWM→추력 변환 포함)
        with rosbag.Bag(input_bag_path, 'r') as inbag:
            for topic, msg, t in inbag.read_messages():
                if topic in ['/cf4/thrust', '/cf1/thrust']:
                    raw_pwm = msg.vector.x
                    thrust_val = pwm_to_thrust(raw_pwm)
                    new_msg = Vector3Stamped()
                    new_msg.header = msg.header
                    new_msg.vector.x = thrust_val
                    new_msg.vector.y = 0.0
                    new_msg.vector.z = 0.0
                    outbag.write(topic, new_msg, t)
                else:
                    outbag.write(topic, msg, t)

        # cf4 total_force 계산
        for thrust_time, thr_msg in thr_cf4:
            pa = find_nearest(thrust_time, acc_cf4, threshold)
            pb = find_nearest(thrust_time, body_cf4, threshold)
            ps = find_nearest(thrust_time, scalar_cf4, threshold)
            if not (pa and pb and ps):
                rospy.logwarn(f"cf4 sync failed @ {thrust_time.to_sec():.3f}")
                continue
            _, acc_msg = pa
            _, body_msg = pb
            _, scalar_msg = ps
            F_acc = np.array([acc_msg.vector.x, acc_msg.vector.y, acc_msg.vector.z])
            F_body = np.array([body_msg.vector.x, body_msg.vector.y, body_msg.vector.z])
            F_rms  = np.array([0.0, 0.0, scalar_msg.data])
            if thr_msg.vector.x == 0.0:
                F_acc[:] = 0.0; F_body[:] = 0.0; F_rms[:] = 0.0
            F_total = F_acc + F_body + F_rms
            total = Vector3Stamped()
            total.header = thr_msg.header
            total.vector.x, total.vector.y, total.vector.z = F_total
            outbag.write('/cf4/total_force', total, thrust_time)

        # cf1 total_force 계산
        for thrust_time, thr_msg in thr_cf1:
            pa = find_nearest(thrust_time, acc_cf1, threshold)
            pb = find_nearest(thrust_time, body_cf1, threshold)
            ps = find_nearest(thrust_time, scalar_cf1, threshold)
            if not (pa and pb and ps):
                rospy.logwarn(f"cf1 sync failed @ {thrust_time.to_sec():.3f}")
                continue
            _, acc_msg = pa
            _, body_msg = pb
            _, scalar_msg = ps
            F_acc = np.array([acc_msg.vector.x, acc_msg.vector.y, acc_msg.vector.z])
            F_body = np.array([body_msg.vector.x, body_msg.vector.y, body_msg.vector.z])
            F_rms  = np.array([0.0, 0.0, scalar_msg.data])
            if thr_msg.vector.x == 0.0:
                F_acc[:] = 0.0; F_body[:] = 0.0; F_rms[:] = 0.0
            F_total = F_acc + F_body + F_rms
            total = Vector3Stamped()
            total.header = thr_msg.header
            total.vector.x, total.vector.y, total.vector.z = F_total
            outbag.write('/cf1/total_force', total, thrust_time)

    rospy.loginfo(f"Processed '{input_bag_path}' → '{output_bag_path}'")

if __name__ == '__main__':
    rospy.init_node('batch_sync_and_sum_forces', anonymous=True)
    bags = sorted(glob.glob("21g_T?_rms.bag"))
    if not bags:
        rospy.logerr("No matching '21g_T?_rms.bag' files found.")
        sys.exit(1)
    for inp in bags:
        out = inp.replace('_rms.bag', '_total.bag')
        try:
            process_bag(inp, out)
        except Exception as e:
            rospy.logerr(f"Error processing {inp}: {e}")
