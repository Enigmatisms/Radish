#!/usr/bin/env python2
#-*-coding:utf-8-*-

"""
    Some of the rosbags may exhibit problems like: (angle_max - angle_min) / resolution != point_num - 1
    This will cause fatal errors in some of the algorithms
"""

import rosbag
from sys import argv
from math import ceil

def validate(bag_name):
    bag_in = rosbag.Bag(bag_name, "r")
    total_cnt = 0
    error_cnt = 0
    typical_point_num = 0
    typical_range = 0
    angle_min = 0.0
    angle_max = 0.0
    angle_inc = 0.0
    zero_cnt = 0
    for i, (_, msg, _) in enumerate(bag_in.read_messages()):
        if not str(msg._type) == "sensor_msgs/LaserScan":
            continue
        total_cnt += 1
        point_num = ceil((msg.angle_max - msg.angle_min) / msg.angle_increment) 
        if typical_point_num == 0:
            typical_point_num = point_num
            typical_range = len(msg.ranges)
            angle_max = msg.angle_max
            angle_min = msg.angle_min
            angle_inc = msg.angle_increment
        if len(msg.ranges) != point_num or len(msg.intensities) != point_num:
            if len(msg.intensities) == 0:
                zero_cnt += 1
                continue
            print "Scan %d should have point num: %d, yet ranges or intensities doesn't. (%d, %d)"%(i, point_num, len(msg.ranges), len(msg.intensities))
            error_cnt += 1
    print "%d messages related to laser scan, %d of them are faulty, %d of them have no intensity."%(total_cnt, error_cnt, zero_cnt)
    print "Point num: %d, range number: %d"%(typical_point_num, typical_range)
    print "Angle inc: %.7lf, should have been %.7lf"%(angle_inc, (angle_max - angle_min) / (typical_range - 1))
    bag_in.close()

# 激光雷达的角度分辨率与点数关系应该是(max - min) / inc = num - 1 而不是 (max - min) / inc = num
def fixAngleIncrement(bag_name):
    bag_in = rosbag.Bag(bag_name, "r")
    output_name = bag_name.split(".bag")[0] + "_fix.bag"
    bag_out = rosbag.Bag(output_name, "w")
    fix_num = 0
    original_max = -1.0
    original_min = -1.0
    now_max = -1.0
    saved_point_num = 0
    angle_inc = 0
    for topic, msg, t in bag_in.read_messages():
        if not str(msg._type) == "sensor_msgs/LaserScan":
            bag_out.write(topic, msg, t)
            continue
        point_num = ceil((msg.angle_max - msg.angle_min) / msg.angle_increment)
        if len(msg.ranges) != point_num:
            if original_max < 0:
                original_max = msg.angle_max
                original_min = msg.angle_min
            msg.angle_max = msg.angle_min + msg.angle_increment * (len(msg.ranges) - 1)
            if now_max < 0.0:
                now_max = msg.angle_max
                saved_point_num = len(msg.ranges) - 1
                angle_inc = msg.angle_increment
            fix_num += 1
        bag_out.write(topic, msg, t)
    print "Previous angle max: %.7lf, current angle max %.7lf"%(original_max, now_max)
    print "Angle_min: %.6lf, angle inc: %.6lf, point num: %d"%(original_min, angle_inc, saved_point_num)
    bag_out.close()
    bag_in.close()

if __name__ == "__main__":
    if len(argv) < 2:
        print "Usage: python2 ./validator.py <path to the rosbag>"
    fixAngleIncrement(argv[1])