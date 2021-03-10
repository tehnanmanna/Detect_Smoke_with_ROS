#!/usr/bin/env python

import rospy
import actionlib
import sys

from std_msgs.msg import String as String
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Twist as Twist
from observer.msg import ObserverLinkStates

class MoveBaseClient:
    def __init__(self):
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('>> waiting for move_base action server')
        self.move_base_client.wait_for_server()
        rospy.loginfo('>> move_base action server available')
        self.vis_sub = rospy.Subscriber('/mockVision/objects', ObserverLinkStates, vis_cb)
        self.goal_vec = []
        self.timer = rospy.Timer(rospy.Duration(2.0), self.time_cb)
        self.MB_bussy = False
        self.goal = MoveBaseGoal()

    def time_cb(self, data):
        if goal_vec:
            cur_goal = goal_vec[0]
            self.goal.target_pose.header.frame_id = 'arm_link_5'
            self.goal.target_pose.pose.position.x = cur_goal.position.x
            self.goal.target_pose.pose.position.y = cur_goal.position.y
            self.goal.target_pose.pose.position.z = cur_goal.position.z
            self.goal.target_pose.pose.orientation.x = cur_goal.orientation.x
            self.goal.target_pose.pose.orientation.y = cur_goal.orientation.y
            self.goal.target_pose.pose.orientation.z = cur_goal.orientation.z
            self.goal.target_pose.pose.orientation.w = cur_goal.orientation.w
            del goal_vec[0]
            if not self.MB_bussy:
                self.client_call()

    def client_call(self):
            self.goal.target_pose.header.stamp = rospy.Time.now()

            rospy.loginfo('Sending goal: %s', self.move_base_goal.target_pose.pose)

            self.move_base_client.send_goal(self.move_base_goal)
            rospy.loginfo("Waiting for result")
            self.MB_bussy = True
            if self.move_base_client.wait_for_result(rospy.Duration(5.0)):
                rospy.loginfo("Goal reached")
                self.goal_time = rospy.Time.now()
                self.MB_bussy = False
                #info = self.get_info()
                #self.goal_pub.publish(info)
            else:
                self.MB_bussy = False

    def vis_cb(self, data):
        for i in range(len(data.name)):
            if name[i] == 'car_wheel':
                self.goal_vec.append(data.pose[i])

def main():
    rospy.init_node('SimpleObjectFollower')
    rospy.loginfo('Simple Object Follower')

    mb_client = MoveBaseClient()

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__=='__main__':
    main()
