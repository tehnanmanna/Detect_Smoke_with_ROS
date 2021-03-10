#!/usr/bin/env python
import rospy
from observer.observer_api import ObserverApi
from observer.observer_common import *
from observer.conditions.nav_condition import NavCondition
#from observer.conditions.bool_condition import BoolCondition
#from observer.conditions.link_condition import LinkCondition
from observer.observables.collision_observable import CollisionObservable
from observer.observables.map_observable import MapObservable
from observer.observables.link_pose_observable import LinkPoseObservable
from observer.observables.pose_reached_observable_handler import PoseReachedObservableHandler
from observer.sim_state import SimState
from logger.logger_api import LoggerAPI
import subprocess
from std_msgs.msg import String
'''
roslaunch simple scenario (sim+base_bumber, controller, nav+amcl)
start this script 
in other terminal listen to /observer topic
drive around a bit
force a collision
drive to position 1 1 0 and turn in place

'''
'''
'rosservice call /change_material "{name: "Gazebo/Grey", reflectivity: 0.79, transmission: 0.03, absorption: 0.89, angular_factor: 0.5}"'
'''
def compute_angl(count):
    c = count % 100
    s = "0.1"
    if c < 20:
        s = "0.3"
    elif c >= 20 and c < 40:
        s = "0.6"
    elif c >= 40 and c < 60:
        s = "0.9"
    elif c >= 60 and c < 80:
        s = "1.2"
    elif c >= 80 and c < 100:
        s = "1.5"
    return s

def compute_refl(c):
    s = "0.2"
    if c < 100:
        s = "0.2"
    elif c >= 100 and c < 300:
        s = "0.4"
    elif c >= 300 and c < 500:
        s = "0.6"
    elif c >= 500 and c < 700:
        s = "0.8"
    elif c >= 700 and c <= 1000:
        s = "1.0"
    return s

def get_command_alternate(count):
    a,b,c = "0","0","0"

    d = compute_angl(count)
    s = str(count)
    if count < 10:
        c = s
    elif count >= 10 and count < 100:
        c,b = s[0],s[1]
    elif count >= 100:
        c, b, a = s[0],s[1],s[2]
    a = compute_refl(count)
    return 'rosservice call /change_material "{name: "Gazebo/Grey", reflectivity: %s, transmission: 0.%s, absorption: 0.%s, angular_factor: %s}"'%(a,b,c,d)

def main():
    ## commands
    rospy.init_node("material_rosbag_recorder")
    path = get_ros_sourcefolder_path()+"/automatic_simulation/tests"
    logger = LoggerAPI()
    pub = rospy.Publisher("/info_vals",String, queue_size=1)

    topics = ["/tf", "/material_laser_scan", "/clock", "/info_vals"]

    for i in xrange(1000):
        command = get_command_alternate(i)
        subprocess.call(command+' &', shell=True)
        rospy.sleep(0.5)
        logger.start_rosbag(path, "bag_"+str(i), topics)
        t = rospy.get_rostime()
        while((rospy.get_rostime() - t).to_sec() < 4.0):
            rospy.sleep(0.1)
            pub.publish(command)
        logger.stop_rosbag()
        rospy.sleep(1.0)


if __name__=="__main__":
    main()
