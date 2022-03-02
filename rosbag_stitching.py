#!/usr/bin/env python
#coding=utf8

"""
    Concatenate all the rosbags in a folder to form a huge bag
"""

import rosbag
from sys import argv
import os

def conditionalRemove(bag_name, bag_out, topic_name = None):
    bag_in = rosbag.Bag(bag_name, "r")
    for topic, msg, t in bag_in.read_messages():
        # print "topic name: %s"%(topic)
        if not topic_name is None and not topic == topic_name:
            continue
        bag_out.write(topic, msg, t)
    print "Bag '%s' is merged into the current bag with messages having '%s' as topic"%(bag_name, topic_name)
    bag_in.close()

if __name__ == "__main__":
    if len(argv) < 4:
        print("Usage: python2 ./modifyScan.py <directory of rosbags> <output rosbag name>.bag <maximum number of bags to merge>")
        exit(-1)
    directory, output_name, max_bag_num = argv[1], argv[2], int(argv[3])
    if not directory[-1] == '/': directory = "%s/"%(directory)
    initial_skip = 0
    if len(argv) > 4:
        initial_skip = int(argv[4])
    bag_out = rosbag.Bag(output_name + ".bag", "w")
    all_files = os.listdir(directory)
    all_files.sort()
    for i in range(initial_skip, initial_skip + max_bag_num):
        conditionalRemove(directory + all_files[i], bag_out, "/scan")
    bag_out.close()
    print("Process completed.")

# bag_out = rosbag.Bag(name_no_ext + "_st.bag", "w")