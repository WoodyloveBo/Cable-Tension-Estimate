#!/usr/bin/env python
import rosbag
import rospy
import glob
import sys
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32

def pwm_to_thrust(pwm, pwm_min=10000, pwm_max=60000):
    a, b, c = -1.482e-09, 0.001495, -3.448
    if pwm < pwm_min:
        return 0.0
    pwm = min(pwm, pwm_max)
    T = a * pwm**2 + b * pwm + c
    return max(T, 0.0)

def find_nearest(target_time, msg_list, threshold=0.1):
    best = None
    mindt = None
    for t, msg in msg_list:
        dt = abs((t - target_time).to_sec())
        if mindt is None or dt < mindt:
            mindt, best = dt, (t, msg)
    return best if (mindt is not None and mindt <= threshold) else None

def normalize_and_subtract(input_bag, output_bag, threshold=0.1, time_offset=-0.5):
    # 1) acceleration_force, body_force 미리 수집
    acc_msgs, body_msgs = [], []
    with rosbag.Bag(input_bag, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == '/cf4/acceleration_force':
                acc_msgs.append((t, msg))
            elif topic == '/cf4/body_force':
                body_msgs.append((t, msg))
    acc_msgs.sort(key=lambda x: x[0])
    body_msgs.sort(key=lambda x: x[0])

    # 2) 입력 bag 읽으면서 처리 및 출력
    with rosbag.Bag(input_bag, 'r') as inbag, rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in inbag.read_messages():
            if topic == '/cf4/thrust':
                # PWM → 추력 변환 기록
                pwm      = msg.vector.x
                thrust_T = pwm_to_thrust(pwm)
                thrust_msg = Vector3Stamped()
                thrust_msg.header.stamp    = msg.header.stamp
                thrust_msg.header.frame_id = msg.header.frame_id
                thrust_msg.vector.x        = thrust_T
                thrust_msg.vector.y        = 0.0
                thrust_msg.vector.z        = 0.0
                outbag.write('/cf4/thrust', thrust_msg, t)

                # F_s 계산 및 기록
                acc_pair  = find_nearest(t, acc_msgs, threshold)
                body_pair = find_nearest(t, body_msgs, threshold)
                acc_z     = acc_pair[1].vector.z if acc_pair else 0.0
                body_z    = body_pair[1].vector.z if body_pair else 0.0
                fs_z = thrust_T - acc_z - body_z

                fs_msg = Vector3Stamped()
                fs_msg.header.stamp    = msg.header.stamp
                fs_msg.header.frame_id = msg.header.frame_id
                fs_msg.vector.x        = fs_z
                fs_msg.vector.y        = 0.0
                fs_msg.vector.z        = 0.0
                outbag.write('/cf4/F_s', fs_msg, t)

            elif topic == '/cf4/distance_scalar_force_rms':
                # timestamp를 time_offset 만큼 오프셋
                new_t = t - rospy.Duration.from_sec(abs(time_offset))
                outbag.write('/cf4/distance_scalar_force_rms', msg, new_t)

            else:
                # 그 외 토픽은 그대로 복사
                outbag.write(topic, msg, t)

    rospy.loginfo("Wrote converted thrust, F_s, and time-offset scalar to '%s'", output_bag)

if __name__ == '__main__':
    rospy.init_node('batch_compute_F_s', anonymous=True)

    rms_files = sorted(glob.glob("19g_T?_rms.bag"))
    if not rms_files:
        rospy.logerr("No files matching '19g_T?_rms.bag' found.")
        sys.exit(1)

    for inp in rms_files:
        out = inp.replace('_rms.bag', '_F_s.bag')
        try:
            normalize_and_subtract(inp, out)
        except Exception as e:
            rospy.logerr("Failed on %s: %s", inp, e)
