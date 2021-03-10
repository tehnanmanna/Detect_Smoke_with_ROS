#!/usr/bin/env python
import rospy
import rosnode
import csv
import datetime
import rosbag
import sys
import os
import matplotlib.pyplot as plt
import argparse
import math
from math import hypot
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import yaml as yaml
start_time = None
value_dict = {}
combine = False
#bag_dir = '/home/michael/youbot_local_dev/youbot_rosbag_20180828_szenario1/fahrt3.bag'
#bag_dir = '/home/michael/youbot_local_dev/youbot_rosbag_20180828_szenario2/fahrt1.bag'
#bag_dir = '/home/alex/wentz_catkin_ws/src/automatic_simulation/tests/reference_bag.bag'
'''
"rosservice call /change_material \"{name: \"Gazebo/Grey\", reflectivity: 0.2, transmission:\
  \ 0.0, absorption: 0.1, angular_factor: 0.3}\""
'''

def compute_std(mean, liste):
    temp = []
    for item in liste:
        temp.append((mean - item)**2)
    nm = sum(temp)/ float(len(temp))
    return math.sqrt(nm)

def load_file(filePath,file_name):
    dict_ = {}
    rospy.loginfo("Loading: %s",filePath+"/"+file_name)
    try:
        rospy.loginfo("Loading: %s",file_name)
        file = open(filePath+file_name,'r')
        dict_ = yaml.load(file)
    except yaml.YAMLError as exc:
        print(exc)
        rospy.logerr('Failed to load: %s From: %s',file_name,filePath)
    file.close()
    return dict_

def get_params(temp):
    p = {}
    #temp = temp.split("{")[1]
    temp = temp.split(",")
    temp2 = temp[1].split(":")[1]
    p['reflectivity']=float(temp2.replace(" ", "").replace("\\",""))
    temp2 = temp[2].split(":")[1]
    temp2 = temp2.replace("\\","").replace(" ","")
    p['transmission'] = float(temp2)
    temp2 = temp[3].split(":")[1]
    temp2 = temp2.replace("\\","").replace(" ","")
    p['absorption'] = float(temp2)
    temp2 = temp[4].split(":")[1]
    temp2 = temp2.replace("\\","").replace(" ","")
    temp2 = temp2.replace("}","").replace("\"","")
    p['angular_factor'] = float(temp2)
    return p

def init():
    rospy.init_node("monitoring_bag_topic_extract")

def get_bag_data():
    path = "/home/alex/wentz_catkin_ws/src/automatic_simulation/tests/"
    ref = "reference_angels.yaml"
    ref_dict = load_file(path,ref)
    angels = ref_dict['angels2']
    indexes = ref_dict['index']
    ranges = ref_dict['ranges']
    for f in os.listdir(path):
        if rospy.is_shutdown():
            break
        if f.startswith("bag") and f.endswith(".bag"):
            print "Loading Bag: "+path+f
            bag = rosbag.Bag(path+f)
            params = {}
            scans = []
            for topic, msg, t in bag.read_messages():
                if topic == "/material_laser_scan":
                    scans.append(msg.ranges)
                if topic == "/info_vals" and not params:
                    params = get_params(msg.data.split("{")[1])
            # compute mean_err, std_dev, data_loss per value
            scan_info = {}
            for scan in scans:
                for idx, val in enumerate(scan):
                    if idx in indexes:
                        #all val should be on the plate
                        i = indexes.index(idx)
                        if idx not in scan_info.keys():
                            #print str(val)
                            scan_info[idx] = [0,0,0.0,[],0.0,0.0]
                            scan_info[idx][4] = round(ranges[i], 5)
                            scan_info[idx][5] = angels[i]
                        if val <= 0.8:
                            scan_info[idx][1] +=1
                            scan_info[idx][2] +=val
                            scan_info[idx][3].append(val)
                        else:
                            scan_info[idx][0] +=1
            final_d = {}
            final_d["params"] = params
            for key in scan_info.keys():
                final_d[key] = {}
                final_d[key]['ref_range'] = scan_info[key][4]
                final_d[key]['angle'] = scan_info[key][5]
                if scan_info[key][3]:
                    #if there is at least one element
                    mean = scan_info[key][2] / scan_info[key][1]

                    final_d[key]['mean_range'] = mean
                    std = compute_std(mean, scan_info[key][3])

                    final_d[key]['stdev'] = std
                    final_d[key]['loss'] = float(scan_info[key][0])/float((scan_info[key][1]+scan_info[key][0]))
                else:
                    final_d[key]['mean_range'] = 0.0
                    final_d[key]['stdev'] = 0.0
                    final_d[key]['loss'] = 1.0

            f1 =  yaml.dump(final_d, default_flow_style=False)
            try:
                f = open('/home/alex/wentz_catkin_ws/src/automatic_simulation/tests/yaml/'+f+'.yaml','w')   
                f.write(f1)
                f.close()
            except Exception as inst:
                rospy.loginfo('%s',str(inst))


if __name__ == '__main__':
    init()
    get_bag_data()
