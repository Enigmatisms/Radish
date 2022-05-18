import tqdm
import rosbag
from math import pi
from sys import argv
from sensor_msgs.msg import LaserScan


def multi2Single(bag_name):
    name_no_ext = bag_name.split('.')[0]
    bag_in = rosbag.Bag(bag_name, "r")
    bag_out = rosbag.Bag(name_no_ext + "_single.bag", "w")
    for topic, msg, t in tqdm.tqdm(bag_in.read_messages()):
        if topic in {"/horizontal_laser_2d", "horizontal_laser_2d"} and msg._type == "sensor_msgs/MultiEchoLaserScan":
            new_msg = LaserScan()
            all_ranges = []
            all_intensities = []
            for i in range(len(msg.ranges)):
                echoes = msg.ranges[i].echoes
                intensity = msg.intensities[i].echoes
                if echoes:
                    all_ranges.append(echoes[-1])
                    all_intensities.append(intensity[-1])
                else:
                    print("Anormaly: ", echoes)
                    all_ranges.append(1024.)
                    all_intensities.append(0.)
            new_msg.header = msg.header
            new_msg.angle_increment = msg.angle_increment
            new_msg.angle_max = msg.angle_max
            new_msg.angle_min = msg.angle_min
            new_msg.intensities = all_intensities
            new_msg.range_max = msg.range_max
            new_msg.range_min = msg.range_min
            new_msg.ranges = all_ranges
            new_msg.scan_time =  msg.scan_time
            new_msg.time_increment = msg.time_increment
            # print(msg.ranges[0].echoes[-1])
            # msg.angle_min = msg.angle_min * pi / 180.0
            # msg.angle_max = msg.angle_max * pi / 180.0
            topic = "scan"
            bag_out.write(topic, new_msg, t)
        else:
            bag_out.write(topic, msg, t)
    bag_in.close()
    bag_out.close()
    print("Process completed.")

if __name__ == "__main__":
    multi2Single(argv[1])