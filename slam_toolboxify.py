"""
    Convert simulation rosbag to slam-toolbox-usable bags
    (tf needed, simular to GMapping)
"""

import rosbag
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sys import argv

def toolBoxify(bag_name, output_name):
    
    name_no_ext = bag_name.split('.bag')[0]
    # 输入trim之后的目标角度范围，单位度，需要比最大角度小
    bag_in = rosbag.Bag(bag_name, "r")
    bag_out = rosbag.Bag(output_name, "w")
    odom_cnt = 0
    for i, (topic, msg, t) in enumerate(bag_in.read_messages()):
        # if topic in {"/odom", "odom"}:
        if topic in {"odom", "/odom"}:
            odom_cnt += 1
            if odom_cnt & 1:
                continue
            transform = TransformStamped()
            transform.transform.translation.x = msg.pose.pose.position.x
            transform.transform.translation.y = msg.pose.pose.position.y
            transform.transform.translation.z = msg.pose.pose.position.z

            transform.transform.rotation.w = msg.pose.pose.orientation.w
            transform.transform.rotation.x = msg.pose.pose.orientation.x
            transform.transform.rotation.y = msg.pose.pose.orientation.y
            transform.transform.rotation.z = msg.pose.pose.orientation.z
            transform.header.seq = msg.header.seq
            transform.header.stamp = msg.header.stamp
            transform.header.frame_id = "odom"
            transform.child_frame_id = "scan"
            tf_msg = TFMessage()
            tf_msg.transforms.append(transform)
            bag_out.write("tf", tf_msg, t)
        else:
            bag_out.write(topic, msg, t)
    bag_in.close()
    bag_out.close()
    # print("tf conversion completed")

if __name__ == "__main__":
    toolBoxify(argv[1], argv[2])
            
