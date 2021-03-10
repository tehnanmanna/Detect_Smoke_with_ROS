#!/usr/bin/env python

import rospy
#import socket
#import re
#import tf
#import math
import geometry_msgs.msg
#import yaml
#import errno
# import zmq
import sys

from geometry_msgs.msg import PoseWithCovarianceStamped


def main():

    if not len(sys.argv) == 8:
        print("[Initial Pose Setter] scripts expects 7 parameters(pose, orientation): px py pz ox oy oz ow")
        return -1

    rospy.init_node('initial_pose_setter')
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    #rospy.sleep(2)

    pose = PoseWithCovarianceStamped()    

    pose.header.frame_id = '/map'
    pose.header.stamp = rospy.Time.now()
    pose.header.seq = 1
    pose.pose.pose.position.x = float(sys.argv[1])
    pose.pose.pose.position.y = float(sys.argv[2])
    pose.pose.pose.position.z = float(sys.argv[3])
    pose.pose.pose.orientation.x = float(sys.argv[4])
    pose.pose.pose.orientation.y = float(sys.argv[5])
    pose.pose.pose.orientation.z = float(sys.argv[6])
    pose.pose.pose.orientation.w = float(sys.argv[7])

    pose.pose.covariance = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.006853891945200942]

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        pub.publish(pose)
        pose.header.seq += 1
        if pose.header.seq > 20:
            break
        rate.sleep()


if __name__ == '__main__':
    main()
