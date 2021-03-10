#!/usr/bin/env python

import rospy
#import actionlib
import sys

from std_msgs.msg import String as String
#from move_base_msgs.msg import MoveBaseAction
#from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Twist as Twist
'''
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
        temp = 'x '+str(self.move_base_goal.target_pose.pose.position.x)+' y '+str(self.move_base_goal.target_pose.pose.position.y)+' z '+str(self.move_base_goal.target_pose.pose.position.z)+' x '+str(self.move_base_goal.target_pose.pose.orientation.x)+' y '+str(self.move_base_goal.target_pose.pose.orientation.y)+' z '+str(self.move_base_goal.target_pose.pose.orientation.z)+' w '+str(self.move_base_goal.target_pose.pose.orientation.w)+' header_time_stemp '+str(self.move_base_goal.target_pose.header.stamp)+'   \n ///Forward Score: '+str(self.score)
        return temp
            
    def run(self,rate):
        if self.succeeded:
            info = self.get_info()
            rospy.loginfo("Publish Infos")
            for i in range(10):
                self.goal_pub.publish(info)
                rate.sleep()
'''

def main():

    if not len(sys.argv) == 4:
        print("[CMD_VEL_PUB] scripts expects 3 parameters(vx, vy, vtheta)")
        return -1
        
    cmd_vel = Twist()
    cmd_vel.linear.x = float(sys.argv[1])
    cmd_vel.linear.y = float(sys.argv[2])
    cmd_vel.angular.z = float(sys.argv[3])

    rospy.init_node('CMD_VEL_PUB')
    rospy.loginfo('CMD_VEL_PUB')
    rate = rospy.Rate(30)
    cmd_vel_pub = rospy.Publisher("/gazebo/cmd_vel", Twist, queue_size = 10)
    goal_pub = rospy.Publisher('/cmd_vel_goal', String, queue_size = 10)
    s_time = rospy.get_time()
    now = rospy.get_rostime()
    while now.secs == 0 and now.nsecs == 0:
        now = rospy.get_rostime()
        rate.sleep()
    rospy.loginfo('Secs: %i   Nsecs: %i',now.secs,now.nsecs)
    
    while (rospy.get_rostime() - now) <= rospy.Duration(12.0):
        cmd_vel_pub.publish(cmd_vel)
        rate.sleep()
    cmd_vel.linear.x = 0.0
    cmd_vel.linear.y = 0.0
    cmd_vel.angular.z = 0.0
    cmd_vel_pub.publish(cmd_vel)
    goal_pub.publish("Finished: "+str(rospy.get_time() - s_time))

if __name__ == '__main__':
    main()
    
