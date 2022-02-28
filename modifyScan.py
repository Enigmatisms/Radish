#!/usr/bin/env python
#coding=utf8


'''This is a converter for the Intel Research Lab SLAM dataset
   ( http://kaspar.informatik.uni-freiburg.de/~slamEvaluation/datasets/intel.clf )
   to rosbag'''

import rosbag
import logging
from sys import argv
from math import ceil, floor

logging.basicConfig()

if __name__ == "__main__":
    if len(argv) < 3:
        print("Usage: python2 ./modifyScan.py <input rosbag name>.bag <intended angle span in rads>")
        exit(-1)
    bag_name = argv[1]
    name_no_ext = bag_name.split('.')[0]
    angle_trim = abs(float(argv[2]))
    bag_in = rosbag.Bag("./" + bag_name, "r")
    bag_out = rosbag.Bag("./" + name_no_ext + "_st.bag", "w")
    angle_min_max_set = False
    min_trim, max_trim = 0, 0
    saved_angle_max = 0.0
    saved_angle_min = 0.0
    saved_angle_inc = 0.0
    origin_length, actual_length = 0, 0
    for topic, msg, t in bag_in.read_messages():
        if topic in {"/scan", "/r2000_node/scan"}:
            # 修改最大值最小值，range，intensity
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
            bag_out.write(topic, msg, t)

    bag_in.close()
    bag_out.close()
    print "Process completed."
    print "Original angle min: %f, angle max: %f, angle inc: %f"%(saved_angle_min, saved_angle_max, saved_angle_inc)
    print "Min max trim: %d, %d, actual length: %d, origin length: %d"%(min_trim, max_trim, actual_length, origin_length)