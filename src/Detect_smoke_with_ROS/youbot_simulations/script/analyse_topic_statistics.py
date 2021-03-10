#!/usr/bin/env python
import rospy
import sys
import os
import yaml
from rosgraph_msgs.msg import TopicStatistics

def parse_sys_args():
    Path = ''
    if len(sys.argv) > 1:
        path = sys.argv[1]
        name = sys.argv[2]
        return path, name
    else:
        rospy.logwarn("analyse_topic_statistics require 2 args, %d are given", (len(sys.argv)-1))
        rospy.logwarn("1 Arg:   /Path/for/safing")
        rospy.logwarn("2 Arg:   Name")
        return "/home/alex","infos"

def safe_yaml_to_file(data_dict, path, name):
    f1 =  yaml.dump(data_dict, default_flow_style=False)
    try:
        if not os.path.exists(path):
            os.makedirs(path)
        f = open(path+'/'+name+'.yaml', 'w')   
        f.write(f1)
        f.close()
    except Exception as inst:
        print("Unexpected error:", sys.exc_info()[0])
        print(inst)

class StatisticsAnalyze():
    def __init__(self, path, name):
        self.path = path
        self.name = name
        self.sub = rospy.Subscriber("/statistics",
                                    TopicStatistics, self.topic_cb)
        self.data = {}
        self.time = rospy.Time.now()
        rospy.on_shutdown(self.shutdown_cb)
        self.on = True

    def topic_cb(self, msg):
        if self.on:
            name = msg.topic
            top_sub = msg.node_pub+"--->"+msg.node_sub
            start = msg.window_start
            stop = msg.window_stop
            if name not in self.data:
                self.data[name] = {}
            if top_sub not in self.data[name]:
                # start, byte/msg, stop, max_bw, min_bw, total_time, total_traffic, avg_bw, delivered, dropped
                self.data[name][top_sub] = [0,0,0,0,100000,0.0,0.0,0.0,0,0]
                self.data[name][top_sub][0] = start.to_sec()
                self.data[name][top_sub][1] = float(msg.traffic) / (msg.dropped_msgs + msg.delivered_msgs)
            self.data[name][top_sub][2] = stop.to_sec()
            bw = msg.traffic / (msg.window_stop - msg.window_start).to_sec()
            if bw > self.data[name][top_sub][3]:
                self.data[name][top_sub][3] = bw
            if bw < self.data[name][top_sub][4]:
                self.data[name][top_sub][4] = bw
            self.data[name][top_sub][5] += (msg.window_stop - msg.window_start).to_sec()
            self.data[name][top_sub][6] += msg.traffic
            self.data[name][top_sub][7] = self.data[name][top_sub][6] / self.data[name][top_sub][5]
            self.data[name][top_sub][8] += msg.delivered_msgs
            self.data[name][top_sub][9] += msg.dropped_msgs
            self.data[name][top_sub][1] = float(self.data[name][top_sub][6]) / (self.data[name][top_sub][8] +self.data[name][top_sub][9])
            self.time = rospy.Time.now()



    def shutdown_cb(self):
        self.on = False
        # start, byte/msg, stop, max_bw, min_bw, total_time, total_traffic, avg_bw, delivered, dropped
        for key1 in self.data.keys():
            for key2 in self.data[key1].keys():
                self.data[key1][key2][0] = "Start: " +str(self.data[key1][key2][0])
                self.data[key1][key2][1] = "Byte/Msg: " +str(self.data[key1][key2][1])
                self.data[key1][key2][2] = "Stop: "+str(self.data[key1][key2][2])
                self.data[key1][key2][3] = "Bw_Max: "+str(self.data[key1][key2][3])
                self.data[key1][key2][4] = "BW_Min: "+str(self.data[key1][key2][4])
                self.data[key1][key2][5] = "Total_time: "+str(self.data[key1][key2][5])
                self.data[key1][key2][6] = "Total_traff: "+str(self.data[key1][key2][6])
                self.data[key1][key2][7] = "Avg_Bw: "+str(self.data[key1][key2][7])
                self.data[key1][key2][8] = "Count_Devl: "+str(self.data[key1][key2][8])
                self.data[key1][key2][9] = "Drop_Count: "+str(self.data[key1][key2][9])

        safe_yaml_to_file(self.data, self.path, self.name)

    def run(self, rate):
        while not rospy.is_shutdown():
            rate.sleep()

def main():
    rospy.init_node("StatisticsAnalizer")
    path, name = parse_sys_args()
    r = rospy.Rate(100)
    sAnal = StatisticsAnalyze(path, name)
    sAnal.run(r)

if __name__=="__main__":
    main()
