#!/usr/bin/env python
import rospy
import rosnode
import csv
import datetime
import rosbag
import sys
import matplotlib.pyplot as plt
import argparse
import math
from math import hypot
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import yaml as yaml
start_time = None
value_dict = {}
combine = False
#bag_dir = '/home/michael/youbot_local_dev/youbot_rosbag_20180828_szenario1/fahrt3.bag'
#bag_dir = '/home/michael/youbot_local_dev/youbot_rosbag_20180828_szenario2/fahrt1.bag'
bag_dir = '/home/alex/wentz_catkin_ws/src/automatic_simulation/tests/reference_bag.bag'

def init():
    rospy.init_node("monitoring_bag_topic_extract")

def get_bag_data():
    topic_list = []
    bag = rosbag.Bag(bag_dir)
    index = []
    angels = []
    ranges = []
    for topic, msg, t in bag.read_messages():
        if topic == "/scan":
            alpha = msg.angle_increment
            sin_a = math.sin(alpha)
            cos_a = math.cos(alpha)
            c = 0.0
            for idx, r in enumerate(msg.ranges):
                if r <= 0.8:
                    if c == 0.0:
                        c = r
                        continue
                    y = (sin_a*(-1*c))
                    x = -1*r + c * cos_a
                    gamma = abs(math.atan2(y, x))
                    gamma = abs(gamma - math.pi/2)
                    angels.append(gamma)
                    index.append(idx)
                    ranges.append(r)
                    c = r
            break
    # glaetten der winkel
    angels2 = []
    for idx, val in enumerate(angels):
        if idx < 1:
            angels2.append(val)
            continue
        elif idx >= (len(angels)-2):
            angels2.append(val)
            continue
        else:
            v = sum(angels[idx-3:idx+2])/5
            angels2.append(v)

    print len(angels)
    print len(angels2)

    d = {}
    d['angel'] = angels
    d['angels2'] = angels2
    d['index'] = index
    d['ranges'] = ranges
    f1 =  yaml.dump(d, default_flow_style=False)
    try:
        f = open('/home/alex/wentz_catkin_ws/src/automatic_simulation/tests/reference_angels.yaml','w')   
        f.write(f1)
        f.close()
    except Exception as inst:
        rospy.loginfo('%s',str(inst))


if __name__ == '__main__':
    init()
    get_bag_data()
