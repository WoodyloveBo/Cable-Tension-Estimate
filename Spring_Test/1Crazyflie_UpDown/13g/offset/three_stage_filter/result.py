#!/usr/bin/env python
import rosbag
import rospy
import glob
import sys
import tf
import numpy as np
from geometry_msgs.msg import Vector3Stamped, PoseStamped
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

def process_bag(input_bag_path, output_bag_path,
                log1_pose_offset=rospy.Duration(0), threshold=0.1):
    # collect
    log_msgs, pose_msgs, point_msgs = [], [], []
    with rosbag.Bag(input_bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == '/cf4/log1_three_stage_filtered/T1':
                corrected_t = msg.header.stamp + log1_pose_offset
                log_msgs.append((corrected_t, msg))
            elif topic == '/natnet_ros/cf4/pose':
                pose_msgs.append((msg.header.stamp, msg))
            elif topic == '/point_index4':
                point_msgs.append((msg.header.stamp, msg))
    log_msgs .sort(key=lambda x: x[0])
    pose_msgs.sort(key=lambda x: x[0])
    point_msgs.sort(key=lambda x: x[0])

    # constants
    m_Q, g = 0.04, 9.81
    k, l0 = 1.615, 0.205
    conv = 1000.0 / 9.81

    with rosbag.Bag(output_bag_path, 'w') as outbag:
        for corrected_t, log_msg in log_msgs:
            # accel → force
            sync = find_nearest(corrected_t, pose_msgs, threshold)
            if not sync: continue
            _, pose_msg = sync
            if len(log_msg.values) < 4: continue

            a_world = np.array(log_msg.values[1:4]) * g
            q = [pose_msg.pose.orientation.x,
                 pose_msg.pose.orientation.y,
                 pose_msg.pose.orientation.z,
                 pose_msg.pose.orientation.w]
            R = tf.transformations.quaternion_matrix(q)[:3,:3].T
            a_body = R.dot(a_world)
            F_acc_gf = (m_Q * a_body) * conv
            acc = Vector3Stamped()
            acc.header.stamp = corrected_t
            acc.header.frame_id = pose_msg.header.frame_id
            acc.vector.x, acc.vector.y, acc.vector.z = F_acc_gf
            outbag.write('/cf4/acceleration_force', acc, corrected_t)

            # thrust
            thr = Vector3Stamped()
            thr.header.stamp = corrected_t
            thr.header.frame_id = pose_msg.header.frame_id
            thr.vector.x = log_msg.values[0]
            outbag.write('/cf4/thrust', thr, corrected_t)

            # spring force
            sync_pt = find_nearest(corrected_t, point_msgs, threshold)
            if sync_pt:
                _, pt_msg = sync_pt
                if pt_msg.points:
                    p4 = pt_msg.points[0]
                    pos = np.array([pose_msg.pose.position.x,
                                    pose_msg.pose.position.y,
                                    pose_msg.pose.position.z])
                    dp = pos - np.array([p4.x, p4.y, p4.z])
                    d = np.linalg.norm(dp)
                    spring = k * (d - l0) if d > 0 else 0.0
                    spring_msg = Float32()
                    spring_msg.data = spring * conv
                    outbag.write('/cf4/distance_scalar_force', spring_msg, corrected_t)

            # gravity force
            F_w = np.array([0,0,m_Q*g])
            F_body_gf = R.dot(F_w) * conv
            body = Vector3Stamped()
            body.header.stamp = corrected_t
            body.header.frame_id = pose_msg.header.frame_id
            body.vector.x, body.vector.y, body.vector.z = F_body_gf
            outbag.write('/cf4/body_force', body, corrected_t)

    rospy.loginfo("Processed '%s' → '%s'", input_bag_path, output_bag_path)

if __name__ == '__main__':
    rospy.init_node('batch_three_stage_to_result', anonymous=True)

    # match 9g_T1_Three_stage_filtered.bag ... 9g_T5_Three_stage_filtered.bag
    files = sorted(glob.glob("13g_T?_Three_stage_filtered.bag"))
    if not files:
        rospy.logerr("No files matched pattern '13g_T?_Three_stage_filtered.bag'")
        sys.exit(1)
    for inp in files:
        out = inp.replace('_Three_stage_filtered.bag', '_result.bag')
        try:
            process_bag(inp, out)
        except Exception as e:
            rospy.logerr("Failed processing %s: %s", inp, e)
