#!/usr/bin/env python3
import rosbag
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped

# 파라미터
INPUT_BAG   = '20g_T3.bag'
OUTPUT_BAG  = '20g_T3_tension_geom.bag'
m_payload   = 20.26  # kg (20g)
SYNC_THRESH = 0.05  # s

def find_nearest(t, lst):
    if not lst:
        return None
    dists = np.abs(np.array([t - ti for ti,_ in lst]))
    idx = int(dists.argmin())
    return lst[idx]

def cable_tensions(theta1, theta2, W):
    # Solve T1*cosθ1 + T2*cosθ2 = W, T1*sinθ1 - T2*sinθ2 = 0
    A = np.array([[np.cos(theta1), np.cos(theta2)],
                  [np.sin(theta1), -np.sin(theta2)]])
    b = np.array([W, 0.0])
    T1, T2 = np.linalg.solve(A, b)
    return T1, T2

def main():
    rospy.init_node('compute_geom_tension', anonymous=True)

    cf4_poses = []  # (sec, PoseStamped)
    cf1_poses = []
    payloads  = []  # (sec, PointCloud)

    with rosbag.Bag(INPUT_BAG, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            sec = t.to_sec()
            if topic == '/natnet_ros/cf4/pose':
                cf4_poses.append((sec, msg))
            elif topic == '/natnet_ros/cf1/pose':
                cf1_poses.append((sec, msg))
            elif topic == '/point_index9':
                payloads.append((sec, msg))

    cf4_poses.sort(key=lambda x: x[0])
    cf1_poses.sort(key=lambda x: x[0])
    payloads .sort(key=lambda x: x[0])

    W = m_payload

    outbag = rosbag.Bag(OUTPUT_BAG, 'w')
    for t_pl, pl in payloads:
        ent4 = find_nearest(t_pl, cf4_poses)
        ent1 = find_nearest(t_pl, cf1_poses)
        if not ent4 or not ent1 or not pl.points:
            continue
        _, p4_msg = ent4
        _, p1_msg = ent1

        p_pl = np.array([pl.points[0].x, pl.points[0].y, pl.points[0].z])
        p4   = np.array([p4_msg.pose.position.x,
                         p4_msg.pose.position.y,
                         p4_msg.pose.position.z])
        p1   = np.array([p1_msg.pose.position.x,
                         p1_msg.pose.position.y,
                         p1_msg.pose.position.z])

        v4 = p4 - p_pl
        v1 = p1 - p_pl
        L4 = np.linalg.norm(v4)
        L1 = np.linalg.norm(v1)
        if L4 < 1e-6 or L1 < 1e-6:
            continue

        # angle to vertical
        cos4 = v4[2] / L4
        cos1 = v1[2] / L1
        theta4 = np.arccos(np.clip(cos4, -1.0, 1.0))
        theta1 = np.arccos(np.clip(cos1, -1.0, 1.0))

        T4, T1 = cable_tensions(theta4, theta1, W)

        stamp = rospy.Time.from_sec(t_pl)
        outbag.write('/cf4/tension', Float32(data=float(T4)), stamp)
        outbag.write('/cf1/tension', Float32(data=float(T1)), stamp)
        outbag.write('/total_tension',
                     Float32(data=float(T4 + T1)), stamp)
    outbag.close()
    print("Wrote geometric tensions to", OUTPUT_BAG)

if __name__ == '__main__':
    main()
