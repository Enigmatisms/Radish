"""
    根据真实位姿计算Odom消息
"""

import rosbag
import os
import sys
import numpy as np
from nav_msgs.msg import Odometry
import rospy
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def goodAngle(angle:float) -> float:
    if angle > np.pi:
        return angle - 2 * np.pi
    elif angle < -np.pi:
        return angle + 2 * np.pi
    return angle

def calculateOdom(bag_name:str, plot:bool = False):
    prefix = bag_name[:-4]
    bag_in = rosbag.Bag(bag_name, "r")
    bag_out_name = prefix + "_odom.bag"
    bag_out = rosbag.Bag(bag_out_name, 'w')
    last_x = 0
    last_y = 0
    last_angle = None
    last_t = 0
    odom_cnt = 0
    if plot:
        all_vx, all_px = [], []
        all_vy, all_py = [], []
        all_va, all_angle = [], []
        all_duration = []

    last_duration = -1.
    last_vx = -1.
    last_vy = -1.
    last_va = -1.
    for topic, msg, t in bag_in.read_messages():
        if not msg._type == "tf/tfMessage":
            bag_out.write(topic, msg, t)
            continue
        geo_msg = msg.transforms[0]
        current_t = geo_msg.header.stamp.to_sec()
        translation = geo_msg.transform.translation
        qt = geo_msg.transform.rotation
        this_x = translation.x
        this_y = translation.y
        this_angle = R.from_quat([0., 0., qt.z, qt.w]).as_rotvec()[-1]
        if not last_angle is None:
            center_t = (last_t + current_t) / 2
            center_x = (last_x + this_x) / 2
            center_y = (last_y + this_y) / 2

            duration = 0.02
            last_duration = duration
            tmp_vx = (this_x - last_x) / duration
            tmp_vy = (this_y - last_y) / duration
            vel_x = (tmp_vx * 0.8 + last_vx * 0.2) if last_vx > 0.0 else tmp_vx
            vel_y = (tmp_vy * 0.8 + last_vy * 0.2) if last_vy > 0.0 else tmp_vy
            last_vx = vel_y
            last_vy = vel_y

            diff_angle = goodAngle(this_angle - last_angle)
            tmp_va = diff_angle / duration           # 避免角度奇异
            ang_v = (tmp_va * 0.5 + last_va * 0.5) if last_va > 0.0 else tmp_va
            last_va = ang_v

            center_angle = this_angle + last_angle
            if abs(diff_angle) > np.pi:
                center_angle = goodAngle(center_angle / 2 + np.pi)
        non_flag = (last_angle is None)
        last_x = this_x
        last_y = this_y
        last_angle = this_angle
        last_t = current_t
        # 这里有问题
        if non_flag:
            bag_out.write(topic, msg, t)
            continue
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.from_seconds(center_t)
        odom_msg.header.seq = odom_cnt
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'scan'
        odom_cnt += 1

        center_qt = R.from_rotvec([0., 0., center_angle]).as_quat()
        odom_msg.pose.pose.position.x = center_x
        odom_msg.pose.pose.position.y = center_y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.w = center_qt[-1]
        odom_msg.pose.pose.orientation.z = center_qt[-2]
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.x = 0.0

        for i in range(3):
            odom_msg.pose.covariance[7 * i] = 0.01
        odom_msg.pose.covariance[-1] = 0.01
        
        odom_msg.twist.twist.linear.x = vel_x
        odom_msg.twist.twist.linear.y = vel_y
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = ang_v
        if plot:
            all_vx.append(vel_x)
            all_px.append(this_x)
            all_vy.append(vel_y)
            all_py.append(this_y)
            all_va.append(ang_v)
            all_angle.append(this_angle)
            all_duration.append(duration)

        for i in range(3):
            odom_msg.twist.covariance[7 * i] = 0.01
        odom_msg.twist.covariance[-1] = 0.01
        bag_out.write("odom", odom_msg, odom_msg.header.stamp)
        bag_out.write(topic, msg, t)
    bag_in.close()

    bag_out.close()
    if plot:
        xs = np.arange(len(all_vx))
        plt.figure(0)
        plt.subplot(2, 2, 1)
        plt.plot(xs, all_vx, label = 'vel')
        plt.plot(xs, all_px, label = 'pos')
        plt.legend()

        plt.subplot(2, 2, 2)
        plt.plot(xs, all_vy, label = 'vel')
        plt.plot(xs, all_py, label = 'pos')
        plt.legend()

        plt.subplot(2, 2, 3)
        plt.plot(xs, all_va, label = 'vel')
        plt.plot(xs, all_angle, label = 'pos')
        plt.legend()

        plt.subplot(2, 2, 4)
        plt.plot(xs, all_duration)

        plt.figure(1)
        diff_x = np.array(all_px[1:]) - np.array(all_px[:-1])
        diff_y = np.array(all_py[1:]) - np.array(all_py[:-1])
        plt.subplot(2, 1, 1)
        plt.plot(np.arange(diff_x.shape[0]), diff_x)
        plt.subplot(2, 1, 2)
        plt.plot(np.arange(diff_y.shape[0]), diff_y)
        plt.figure(2)
        plt.plot(all_px, all_py)
        plt.show()


# def simple_test():
#     for topic, msg, t in bag_in.read_messages():
#         if msg._type != "tf/tfMessage":
#             print("topic name: %s, type: %s, %f, %f"%(topic, msg._type, msg.header.stamp.to_sec(), t.to_sec()))

def traverseDirectory(path:str):
    cnt = 0
    for file in os.listdir(path):
        if not file.endswith(".bag") or file.endswith("_odom.bag"):
            continue
        print("Processing bag {%s} at '%s'"%(file, path))
        cnt += 1
        calculateOdom(path + file)
    print("%d bags at total are processed."%(cnt))

if __name__ == "__main__":
    path = 'add_odom'
    if len(sys.argv) > 1:
        path = sys.argv[1]
    # traverseDirectory(path)
    calculateOdom(path, True)

        