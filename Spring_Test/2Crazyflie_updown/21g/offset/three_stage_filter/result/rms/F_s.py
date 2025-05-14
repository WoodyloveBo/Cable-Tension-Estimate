#!/usr/bin/env python
import rosbag
import rospy
import glob
import sys
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32

def pwm_to_thrust(pwm, pwm_min=10000, pwm_max=60000):
    a, b, c = 2.82e-08, -0.000937, 42.45
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
    # cf4 메시지 수집
    acc4_msgs, body4_msgs = [], []
    with rosbag.Bag(input_bag, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == '/cf4/acceleration_force':
                acc4_msgs.append((t, msg))
            elif topic == '/cf4/body_force':
                body4_msgs.append((t, msg))
    acc4_msgs.sort(key=lambda x: x[0])
    body4_msgs.sort(key=lambda x: x[0])

    # cf1 메시지 수집
    acc1_msgs, body1_msgs = [], []
    with rosbag.Bag(input_bag, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == '/cf1/acceleration_force':
                acc1_msgs.append((t, msg))
            elif topic == '/cf1/body_force':
                body1_msgs.append((t, msg))
    acc1_msgs.sort(key=lambda x: x[0])
    body1_msgs.sort(key=lambda x: x[0])

    # 출력용 bag 생성
    with rosbag.Bag(input_bag, 'r') as inbag, \
         rosbag.Bag(output_bag, 'w') as outbag:

        for topic, msg, t in inbag.read_messages():
            # cf4 처리
            if topic == '/cf4/thrust':
                pwm      = msg.vector.x
                thrust_T = pwm_to_thrust(pwm)
                # 변환된 thrust 기록
                out_msg = Vector3Stamped()
                out_msg.header.stamp    = msg.header.stamp
                out_msg.header.frame_id = msg.header.frame_id
                out_msg.vector.x        = thrust_T
                out_msg.vector.y = out_msg.vector.z = 0.0
                outbag.write('/cf4/thrust', out_msg, t)

                # F_s4 = T - a_z - b_z
                pair_a = find_nearest(t, acc4_msgs, threshold)
                pair_b = find_nearest(t, body4_msgs, threshold)
                a_z = pair_a[1].vector.z if pair_a else 0.0
                b_z = pair_b[1].vector.z if pair_b else 0.0
                fs4 = thrust_T - a_z - b_z
                fs4_msg = Vector3Stamped()
                fs4_msg.header = out_msg.header
                fs4_msg.vector.x = fs4
                fs4_msg.vector.y = fs4_msg.vector.z = 0.0
                outbag.write('/cf4/F_s', fs4_msg, t)

            # cf1 처리
            elif topic == '/cf1/thrust':
                pwm      = msg.vector.x
                thrust_T = pwm_to_thrust(pwm)
                out_msg = Vector3Stamped()
                out_msg.header.stamp    = msg.header.stamp
                out_msg.header.frame_id = msg.header.frame_id
                out_msg.vector.x        = thrust_T
                out_msg.vector.y = out_msg.vector.z = 0.0
                outbag.write('/cf1/thrust', out_msg, t)

                pair_a = find_nearest(t, acc1_msgs, threshold)
                pair_b = find_nearest(t, body1_msgs, threshold)
                a_z = pair_a[1].vector.z if pair_a else 0.0
                b_z = pair_b[1].vector.z if pair_b else 0.0
                fs1 = thrust_T - a_z - b_z
                fs1_msg = Vector3Stamped()
                fs1_msg.header = out_msg.header
                fs1_msg.vector.x = fs1
                fs1_msg.vector.y = fs1_msg.vector.z = 0.0
                outbag.write('/cf1/F_s', fs1_msg, t)

            # distance_scalar_force_rms 토픽은 시간 오프셋 적용
            elif topic in ['/cf4/distance_scalar_force_rms', '/cf1/distance_scalar_force_rms']:
                new_t = t - rospy.Duration.from_sec(abs(time_offset))
                outbag.write(topic, msg, new_t)

            else:
                # 그 외 토픽 그대로
                outbag.write(topic, msg, t)

    rospy.loginfo("Wrote thrust conversion and F_s for cf4 & cf1 into '%s'", output_bag)


if __name__ == '__main__':
    rospy.init_node('batch_compute_F_s', anonymous=True)
    rms_files = sorted(glob.glob("21g_T?_rms.bag"))
    if not rms_files:
        rospy.logerr("No files matching '21g_T?_rms.bag' found.")
        sys.exit(1)

    for inp in rms_files:
        out = inp.replace('_rms.bag', '_F_s.bag')
        try:
            normalize_and_subtract(inp, out)
        except Exception as e:
            rospy.logerr("Failed on %s: %s", inp, e)
