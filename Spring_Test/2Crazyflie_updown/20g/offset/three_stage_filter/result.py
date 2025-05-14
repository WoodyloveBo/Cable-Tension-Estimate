#!/usr/bin/env python
import rosbag
import rospy
import glob
import sys
import tf
import numpy as np
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float32
from crazyswarm.msg import GenericLogData

def find_nearest(target_time, msg_list, threshold):
    best = None
    min_dt = None
    for t, msg in msg_list:
        dt = abs((t - target_time).to_sec())
        if min_dt is None or dt < min_dt:
            min_dt, best = dt, (t, msg)
    return best if (min_dt is not None and min_dt <= threshold) else None

def process_one_drone(drone_ns, log_msgs, pose_msgs, point_msgs, outbag,
                      m_Q, g, k_thrust, k_spring, l0, conv, threshold):
    log_msgs.sort(key=lambda x: x[0])
    pose_msgs.sort(key=lambda x: x[0])
    point_msgs.sort(key=lambda x: x[0])

    for corrected_t, log_msg in log_msgs:
        sync_pose = find_nearest(corrected_t, pose_msgs, threshold)
        if not sync_pose: 
            continue
        _, pose_msg = sync_pose

        # 1. Acceleration-based Force
        if len(log_msg.values) >= 4:
            a_world = np.array(log_msg.values[1:4]) * g
            q = [
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w
            ]
            R = tf.transformations.quaternion_matrix(q)[:3, :3].T
            a_body = R.dot(a_world)
            F_acc = m_Q * a_body * conv
            acc_msg = Vector3Stamped()
            acc_msg.header.stamp = corrected_t
            acc_msg.header.frame_id = pose_msg.header.frame_id
            acc_msg.vector.x, acc_msg.vector.y, acc_msg.vector.z = F_acc
            outbag.write(f'/{drone_ns}/acceleration_force', acc_msg, corrected_t)

        # 2. Thrust
        thr_msg = Vector3Stamped()
        thr_msg.header.stamp = corrected_t
        thr_msg.header.frame_id = pose_msg.header.frame_id
        thr_msg.vector.x = log_msg.values[0]
        outbag.write(f'/{drone_ns}/thrust', thr_msg, corrected_t)

        # 3. Spring Force (distance_scalar_force)
        sync_pt = find_nearest(corrected_t, point_msgs, threshold)
        if sync_pt:
            _, pt_msg = sync_pt
            if pt_msg.points:
                p_drone = np.array([pose_msg.pose.position.x,
                                    pose_msg.pose.position.y,
                                    pose_msg.pose.position.z])
                p_pl = np.array([pt_msg.points[0].x,
                                 pt_msg.points[0].y,
                                 pt_msg.points[0].z])
                d = np.linalg.norm(p_drone - p_pl)
                delta_l = max(0.0, d - l0)
                F_spring = k_spring * delta_l * conv
                spring_msg = Float32()
                spring_msg.data = F_spring
                outbag.write(f'/{drone_ns}/distance_scalar_force', spring_msg, corrected_t)

        # 4. Gravity Force in Body Frame
        F_w = np.array([0.0, 0.0, m_Q * g])
        F_body = R.dot(F_w) * conv
        body_msg = Vector3Stamped()
        body_msg.header.stamp = corrected_t
        body_msg.header.frame_id = pose_msg.header.frame_id
        body_msg.vector.x, body_msg.vector.y, body_msg.vector.z = F_body
        outbag.write(f'/{drone_ns}/body_force', body_msg, corrected_t)

def process_bag(input_bag, output_bag, log_offset=rospy.Duration(0), threshold=0.1):
    m_Q = 0.04  # drone mass (kg)
    g = 9.81    # gravity (m/s^2)
    l0 = 0.205  # spring rest length (m)
    conv = 1000.0 / 9.81  # N → gram-force

    # 파일 이름에 따라 cf1과 cf4의 spring constant 설정
    spring_constants = {
        "20g_T1": {"cf1": 2.3437, "cf4": 2.2733},
        "20g_T2": {"cf1": 2.4634, "cf4": 2.1477},
        "20g_T3": {"cf1": 1.9512, "cf4": 2.0072},
    }

    matched_case = None
    for key in spring_constants:
        if key in input_bag:
            matched_case = key
            break

    if not matched_case:
        rospy.logwarn("⚠️ Unknown file pattern. Using default k=2.0 for both drones.")
        k_cf1 = k_cf4 = 2.0
    else:
        k_cf1 = spring_constants[matched_case]["cf1"]
        k_cf4 = spring_constants[matched_case]["cf4"]

    # 메시지 버퍼
    cf4_logs, cf4_poses, cf_pts = [], [], []
    cf1_logs, cf1_poses, cf_pts1 = [], [], []
    others = []

    with rosbag.Bag(input_bag, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == '/cf4/log1_three_stage_filtered':
                cf4_logs.append((t + log_offset, msg))
            elif topic == '/natnet_ros/cf4/pose':
                cf4_poses.append((msg.header.stamp, msg))
            elif topic == '/cf1/log1_three_stage_filtered':
                cf1_logs.append((t + log_offset, msg))
            elif topic == '/natnet_ros/cf1/pose':
                cf1_poses.append((msg.header.stamp, msg))
            elif topic == '/point_index9':
                cf_pts.append((msg.header.stamp, msg))
                cf_pts1.append((msg.header.stamp, msg))
            else:
                others.append((topic, msg, t))

    with rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in others:
            outbag.write(topic, msg, t)

        process_one_drone('cf4', cf4_logs, cf4_poses, cf_pts, outbag,
                          m_Q, g, k_cf4, k_cf4, l0, conv, threshold)
        process_one_drone('cf1', cf1_logs, cf1_poses, cf_pts1, outbag,
                          m_Q, g, k_cf1, k_cf1, l0, conv, threshold)

    rospy.loginfo("✅ Finished processing '%s' → '%s'", input_bag, output_bag)

if __name__ == '__main__':
    rospy.init_node('batch_three_stage_to_result', anonymous=True)

    files = sorted(glob.glob("20g_T*_Three_stage_filtered.bag"))
    if not files:
        rospy.logerr("❌ No files matched pattern '20g_T*_Three_stage_filtered.bag'")
        sys.exit(1)

    for inp in files:
        out = inp.replace('_Three_stage_filtered.bag', '_result.bag')
        try:
            process_bag(inp, out)
        except Exception as e:
            rospy.logerr("❌ Failed processing %s: %s", inp, e)
