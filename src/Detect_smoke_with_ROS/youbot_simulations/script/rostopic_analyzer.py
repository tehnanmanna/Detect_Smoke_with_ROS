#!/usr/bin/env python

#import rospy
import yaml as yaml
import sys
#import rospkg
import os
#import roslib
import re
import time
import subprocess

def get_sub_pub_infos(topics):
    temp = {}
    for topic in topics:
        temp[topic] = {}
        print topic
        try:
            infos = subprocess.check_output('rostopic info '+topic, shell=True).split('\n')
        except Exception as inst:
            print inst
            continue
        time.sleep(0.3)
        subs, pubs = False, False
        subs_l,pubs_l = [],[]
        for info in infos:
            if info.startswith('Type'):
                temp[topic]['Type'] = info.split(':')[1]
            elif info.startswith('Publishers:'):
                pubs = True
                subs = False
            elif pubs and not info.startswith('Subscribers:') and info:
                p = info[0:info.find('(')]
                p = p.replace(' ','').replace('*','')
                pubs_l.append(p)
            elif info.startswith('Subscribers:'):
                pubs = False
                subs = True
            elif subs and not info.startswith('Publishers:') and info:
                p = info[0:info.find('(')]
                p = p.replace(' ','').replace('*','')
                subs_l.append(p)
            else:
                continue
        temp[topic]['Publisher'] = pubs_l
        temp[topic]['Subscriber'] = subs_l
    return temp

def get_topic_hz_dict(path):
    temp = {}
    for file in os.listdir(path):
        temp[file] = {}
        if file.endswith(".txt"):
            rate,mini,maxi,std_dev,count = 0.0,0.0,0.0,0.0,0
            with open(path+'/'+file, 'r') as myfile:
                data=myfile.read().replace('\n',' ').replace('\t',' ').split(' ')
                for c in range(len(data)):
                    if data[c] == 'rate:':
                        rate = rate + float(data[c+1])
                        count = count + 1
                    elif data[c] == 'min:':
                        mini = mini + float(data[c+1].replace('s',''))
                    elif data[c] == 'max:':
                        maxi = maxi + + float(data[c+1].replace('s',''))
                    elif data[c] == 'dev:':
                        std_dev = std_dev + float(data[c+1].replace('s',''))
                myfile.close()
                if count == 0:
                    count = 1
                std_dev = std_dev/count
                rate = rate/count
                maxi = maxi/count
                mini = mini/count
                temp[file]['Rate'] = rate
                temp[file]['Min'] = mini
                temp[file]['Max'] = maxi
                temp[file]['Std_Dev'] = std_dev
        else:
            continue
    return temp


def get_topic_hz(topics, path):
    try:
        if not os.path.exists('/tmp/ros_topics'):
            os.makedirs('/tmp/ros_topics')
    except Exception as inst:
            print inst
    for topic in topics:
        print topic
        name = '/tmp/ros_topics/'+topic.replace('/','_')+'.txt'
        subprocess.call('touch '+name,shell=True)
        f = open(name,'w')
        subprocess.call('rostopic hz '+topic+' &',stdout=f, shell=True)
        time.sleep(2.0)
        temp = subprocess.check_output('ps ax | grep rostopic', shell=True).split('\n')
        for line in temp:
            if not line.find('hz') == -1 and not line.find(topic) == -1:
                print line
                pid = line.split(' ')[1]
                print pid
                subprocess.call('kill -s INT '+str(pid), shell=True)
            else:
                continue
        f.close()
        subprocess.call('mv /tmp/ros_topics/'+topic.replace('/','_')+'.txt '+path, shell=True)
    return get_topic_hz_dict(path)

def safe_yaml_to_file(cur_dict,path,name):
    f1 =  yaml.dump(cur_dict, default_flow_style=False)
    try:
        if not os.path.exists(path):
            os.makedirs(path)
        f = open(path+'/'+name+'.yaml', 'w')   
        f.write(f1)
        f.close()
    except Exception as inst:
        print path
        print("Unexpected error:", sys.exc_info()[0])
        print('\x1b[0;31m' +' Something went wrong with file: '+name+'  #####'+ '\x1b[0m')
        print(inst)

def main():
    path = '/home/ugv/rostopics'

    topics = subprocess.check_output('rostopic list', shell=True).split('\n')
    nodes = subprocess.check_output('rosnode list', shell=True).split('\n')
    del topics[topics.index('/rosout')]
    del topics[topics.index('')]
    print topics

    subs_pubs = get_sub_pub_infos(topics)
    #topics_freq = get_topic_hz(topics, path)
    
    safe_yaml_to_file(subs_pubs, path, 'sub_pub_info_tulf_full')
    #safe_yaml_to_file(topics_freq, path, 'topic_frequenzies')

if __name__ == '__main__':
    main()
