#!/usr/bin/env python

import rospy
import actionlib
import sys

from std_msgs.msg import String as String
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Twist as Twist

class MoveBaseClient:
    def __init__(self, goal):
        self.move_base_goal = goal

        self.goal_time = 0.0
        self.succeeded = False
        self.init_actionclient()
        self.goal_pub = rospy.Publisher('/move_base_goal1', String, queue_size = 10)
        
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel',Twist,self.twist_cb)
        self.score = 0.0
    
    def twist_cb(self, data):
        val = data.linear.x
        if (val < 0.0):
            self.score = self.score - 1.0
    
    def init_actionclient(self):
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('>> SM: waiting for move_base action server')
        self.move_base_client.wait_for_server()
        rospy.loginfo('>> SM: move_base action server available')

    def client_call(self):
            self.move_base_goal.target_pose.header.stamp = rospy.Time.now()

            rospy.loginfo('Sending goal: %s', self.move_base_goal.target_pose.pose)
            
            self.move_base_client.send_goal(self.move_base_goal)
            rospy.loginfo("Waiting for result")
            if self.move_base_client.wait_for_result(rospy.Duration(60.0)):
                rospy.loginfo("Goal reached")
                self.goal_time = rospy.Time.now()
                self.succeeded = True
                #info = self.get_info()
                #self.goal_pub.publish(info)
            else:
                pass


    def get_info(self):
        temp = 'x '+str(self.move_base_goal.target_pose.pose.position.x)+' y '+str(self.move_base_goal.target_pose.pose.position.y)+' z '+str(self.move_base_goal.target_pose.pose.position.z)+' x '+str(self.move_base_goal.target_pose.pose.orientation.x)+' y '+str(self.move_base_goal.target_pose.pose.orientation.y)+' z '+str(self.move_base_goal.target_pose.pose.orientation.z)+' w '+str(self.move_base_goal.target_pose.pose.orientation.w)+' header_time_stamp '+str(self.move_base_goal.target_pose.header.stamp)+'   \n ///Forward Score: '+str(self.score)
        return temp
            
    def run(self,rate):
        if self.succeeded:
            info = self.get_info()
            rospy.loginfo("Publish Infos")
            for i in range(10):
                self.goal_pub.publish(info)
                rate.sleep()

def main():

    if not len(sys.argv) == 8:
        print("[Move Base Client] scripts expects 7 parameters(pose, orientation): px py pz ox oy oz ow")
        return -1

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = float(sys.argv[1])
    goal.target_pose.pose.position.y = float(sys.argv[2])
    goal.target_pose.pose.position.z = float(sys.argv[3])
    goal.target_pose.pose.orientation.x = float(sys.argv[4])
    goal.target_pose.pose.orientation.y = float(sys.argv[5])
    goal.target_pose.pose.orientation.z = float(sys.argv[6])
    goal.target_pose.pose.orientation.w = float(sys.argv[7])
    rospy.init_node('MoveBaseClient')
    rospy.loginfo('MoveBaseClient')
    move_base_c = MoveBaseClient(goal)
    rate = rospy.Rate(20)
    move_base_c.client_call()
    while not rospy.is_shutdown():
        move_base_c.run(rate)
        if move_base_c.succeeded:
            break

if __name__ == '__main__':
    main()
    
