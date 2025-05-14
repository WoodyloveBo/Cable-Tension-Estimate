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
    a, b, c = -1.482e-09, 0.001495, -3.448
    if pwm < pwm_min:
        return 0.0
    pwm = min(pwm, pwm_max)
    T = a * pwm**2 + b * pwm + c
    return max(T, 0.0)

def find_nearest(target_time, msg_list, threshold=0.1):
    nearest = None
    min_dt = None
    for t, msg in msg_list:
        dt = abs((t - target_time).to_sec())
        if min_dt is None or dt < min_dt:
            min_dt, nearest = dt, (t, msg)
    return nearest if (min_dt is not None and min_dt <= threshold) else None

def process_bag(input_bag_path, output_bag_path, threshold=0.1):
    # 1) 메시지 수집
    acc_msgs    = []  # /cf4/acceleration_force
    body_msgs   = []  # /cf4/body_force
    scalar_msgs = []  # /cf4/distance_scalar_force_rms
    thrust_msgs = []  # /cf4/thrust (원본 PWM)

    with rosbag.Bag(input_bag_path, 'r') as inbag:
        for topic, msg, t in inbag.read_messages():
            if topic == '/cf4/acceleration_force':
                acc_msgs.append((t, msg))
            elif topic == '/cf4/body_force':
                body_msgs.append((t, msg))
            elif topic == '/cf4/distance_scalar_force_rms':
                scalar_msgs.append((t, msg))
            elif topic == '/cf4/thrust':
                thrust_msgs.append((t, msg))

    # 시간순 정렬
    acc_msgs.sort(key=lambda x: x[0])
    body_msgs.sort(key=lambda x: x[0])
    scalar_msgs.sort(key=lambda x: x[0])
    thrust_msgs.sort(key=lambda x: x[0])

    # 2) 새로운 bag 작성
    with rosbag.Bag(output_bag_path, 'w') as outbag:
        # 2-1) 원본 메시지 복사 + thrust 변환
        with rosbag.Bag(input_bag_path, 'r') as inbag:
            for topic, msg, t in inbag.read_messages():
                if topic == '/cf4/thrust':
                    raw_pwm    = msg.vector.x
                    thrust_val = pwm_to_thrust(raw_pwm)
                    # 변환된 thrust만 x축에 저장
                    new_msg = Vector3Stamped()
                    new_msg.header = msg.header
                    new_msg.vector.x = thrust_val
                    new_msg.vector.y = 0.0
                    new_msg.vector.z = 0.0
                    outbag.write(topic, new_msg, t)
                else:
                    outbag.write(topic, msg, t)

        # 2-2) /cf4/total_force 계산
        for thrust_time, thrust_msg in thrust_msgs:
            pair_acc    = find_nearest(thrust_time, acc_msgs, threshold)
            pair_body   = find_nearest(thrust_time, body_msgs, threshold)
            pair_scalar = find_nearest(thrust_time, scalar_msgs, threshold)

            if not (pair_acc and pair_body and pair_scalar):
                rospy.logwarn("thrust@%.3f: 일부 메시지 동기화 실패", thrust_time.to_sec())
                continue

            _, acc_msg    = pair_acc
            _, body_msg   = pair_body
            _, scalar_msg = pair_scalar

            # 가속도/바디 포스
            F_acc  = np.array([acc_msg.vector.x, acc_msg.vector.y, acc_msg.vector.z])
            F_body = np.array([body_msg.vector.x, body_msg.vector.y, body_msg.vector.z])
            # 거리 기반 RMS 포스
            F_rms  = np.array([0.0, 0.0, scalar_msg.data])
            # PWM → Thrust 변환 (총합에는 반영하지 않음)
            thrust_val = pwm_to_thrust(thrust_msg.vector.x)

            # thrust_val이 0이면 모두 0
            if thrust_val == 0.0:
                F_acc[:]  = 0.0
                F_body[:] = 0.0
                F_rms[:]  = 0.0

            # 총합 (더 이상 F_thrust 없음)
            F_total = F_acc + F_body + F_rms

            total_msg = Vector3Stamped()
            total_msg.header = thrust_msg.header
            total_msg.vector.x = F_total[0]
            total_msg.vector.y = F_total[1]
            total_msg.vector.z = F_total[2]

            outbag.write('/cf4/total_force', total_msg, thrust_time)

    rospy.loginfo("완료: '%s' → '%s'", input_bag_path, output_bag_path)


if __name__ == '__main__':
    rospy.init_node('batch_sync_and_sum_forces', anonymous=True)

    rms_files = sorted(glob.glob("11g_T?_rms.bag"))
    if not rms_files:
        rospy.logerr("No files matching '11g_T?_rms.bag' found.")
        sys.exit(1)

    for inp in rms_files:
        out = inp.replace('_rms.bag', '_total.bag')
        try:
            process_bag(inp, out)
        except Exception as e:
            rospy.logerr("Failed on %s: %s", inp, e)
