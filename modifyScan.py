#!/usr/bin/env python
#coding=utf8

'''
    This script is run by python2, python3 will fail on this
'''

import rosbag
import logging
from sys import argv
from math import ceil, floor, pi
from collections import deque
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

logging.basicConfig()

def scanTrimmer(bag_name):
    name_no_ext = bag_name.split('.')[0]
    # 输入trim之后的目标角度范围，单位度，需要比最大角度小
    angle_trim = abs(float(argv[2]) * pi / 180.)
    bag_in = rosbag.Bag(bag_name, "r")
    bag_out = rosbag.Bag(name_no_ext + "_t.bag", "w")
    angle_min_max_set = False
    min_trim, max_trim = 0, 0
    saved_angle_max = 0.0
    saved_angle_min = 0.0
    saved_angle_inc = 0.0
    origin_length, actual_length = 0, 0
    initialized = False
    total_length, start_id, valid_len, invalid_cnt = 0, 0, 0, 0
    max_len = 3
    queue = deque([], maxlen = max_len)
    for i, (topic, msg, t) in enumerate(bag_in.read_messages()):
        total_length += 1
        if topic in {"/scan", "/r2000_node/scan"}:
            # 修改最大值最小值，range，intensity
            topic = "scan"
            msg.header.frame_id = "scan"
            if angle_min_max_set == False:
                if angle_trim >= msg.angle_max:
                    raise RuntimeError("Trim angle should be smaller than LiDAR angle max") 
                min_trim_delta = -(angle_trim + msg.angle_min)
                max_trim_delta = msg.angle_max - angle_trim
                min_trim = int(ceil(min_trim_delta / msg.angle_increment))
                max_trim = int(floor(max_trim_delta / msg.angle_increment))
                origin_length = len(msg.ranges)
                
            msg.ranges = msg.ranges[min_trim:-max_trim]
            msg.intensities = msg.intensities[min_trim:-max_trim]
            msg.angle_min = msg.angle_min + float(min_trim) * msg.angle_increment
            msg.angle_max = msg.angle_max - float(max_trim) * msg.angle_increment
            if angle_min_max_set == False:
                saved_angle_max = msg.angle_max
                saved_angle_min = msg.angle_min
                saved_angle_inc = msg.angle_increment
                angle_min_max_set = True
                actual_length = len(msg.ranges)
        if initialized == False and len(queue) >= queue.maxlen:
            start_id = i
            initialized = True
        queue.append((topic, msg, t))
        if initialized:
            invalid_cnt += 1
            if invalid_cnt > max_len:
                (old_topic, old_msg, old_t) = queue[0]
                bag_out.write(old_topic, old_msg, old_t)
                valid_len += 1

    bag_in.close()
    bag_out.close()
    print("Process completed.")
    print("Original angle min: %f, angle max: %f, angle inc: %f"%(saved_angle_min, saved_angle_max, saved_angle_inc))
    print("Min max trim: %d, %d, actual length: %d, origin length: %d"%(min_trim, max_trim, actual_length, origin_length))
    print("Front end trim: start id: %d, final length: %d, full length: %d"%(start_id, valid_len, total_length))

def onlyScanGood(bag_name):
    name_no_ext = bag_name.split('.')[0]
    bag_in = rosbag.Bag(bag_name, "r")
    bag_out = rosbag.Bag(name_no_ext + "_good.bag", "w")
    for topic, msg, t in bag_in.read_messages():
        if topic in {"/scan", "/r2000_node/scan", "/sick_safetyscanners/scan"}:
            msg.angle_min = msg.angle_min * pi / 180.0
            msg.angle_max = msg.angle_max * pi / 180.0
            bag_out.write(topic, msg, t)

    bag_in.close()
    bag_out.close()
    print("Process completed.")

def make_gmapping(bag_name):
    name_no_ext = bag_name.split('.')[0]
    bag_in = rosbag.Bag(bag_name, "r")
    bag_out = rosbag.Bag(name_no_ext + "_good.bag", "w")
    for topic, msg, t in bag_in.read_messages():
        if topic in {"odom", "/odom"}:
            continue
        elif topic in {"/tf", "tf"}:
            transform = msg.transforms[0]
            transform.header.frame_id = "odom"
            transform.child_frame_id = "scan"
            tf_msg = TFMessage()
            tf_msg.transforms.append(transform)
            bag_out.write("tf", tf_msg, t)
        else:
            bag_out.write(topic, msg, t)
    bag_in.close()
    bag_out.close()
    print("Process completed.")

def change_topic(bag_name):
    name_no_ext = bag_name.split('.')[0]
    bag_in = rosbag.Bag(bag_name, "r")
    bag_out = rosbag.Bag(name_no_ext + "_good.bag", "w")
    for topic, msg, t in bag_in.read_messages():
        if topic in {"/Lidar", "Lidar"}:
            bag_out.write("scan", msg, t)
        elif topic in {"/Odom", "Odom"}:
            bag_out.write("odom", msg, t)
    bag_in.close()
    bag_out.close()
    print("Process completed.")

def tfRemoval(bag_name):
    name_no_ext = bag_name.split('.')[0]
    bag_in = rosbag.Bag(bag_name, "r")
    bag_out = rosbag.Bag(name_no_ext + "_st.bag", "w")
    for topic, msg, t in bag_in.read_messages():
        if not str(msg._type) == "sensor_msgs/LaserScan":
            continue
        bag_out.write(topic, msg, t)
    print("Process completed. All the messages in the output bag should be of type 'sensor_msgs/LaserScan'.")
    bag_in.close()
    bag_out.close()

def bagSparify(bag_name):
    name_no_ext = bag_name.split('.')[0]
    bag_in = rosbag.Bag(bag_name, "r")
    bag_out = rosbag.Bag(name_no_ext + "_sp.bag", "w")
    cnt = 0
    for topic, msg, t in bag_in.read_messages():
        if not str(msg._type) == "sensor_msgs/LaserScan":
            continue
        if cnt % 6 == 0:
            bag_out.write(topic, msg, t)
        cnt += 1
    print("Process completed. All the messages in the output bag should be of type 'sensor_msgs/LaserScan'.")
    bag_in.close()
    bag_out.close()

# remove all the messages whose topic is but topic_name
def conditionalRemove(bag_name, topic_name):
    name_no_ext = bag_name.split('.')[0]
    bag_in = rosbag.Bag(bag_name, "r")
    bag_out = rosbag.Bag(name_no_ext + "_st.bag", "w")
    for topic, msg, t in bag_in.read_messages():
        # print "topic name: %s"%(topic)
        if not topic == topic_name:
            continue
        bag_out.write(topic, msg, t)
    print("Process completed. All the messages in the output bag should only contain topic '%s'"%(topic_name))
    bag_in.close()
    bag_out.close()

if __name__ == "__main__":
    if len(argv) < 3:
        print("Usage: python ./modifyScan.py <input rosbag name>.bag <intended angle span in rads>")
        exit(-1)
    bag_name = argv[1]
    bagSparify(bag_name)
    # tfRemoval(bag_name)
    # conditionalRemove(bag_name, "/scan")
    