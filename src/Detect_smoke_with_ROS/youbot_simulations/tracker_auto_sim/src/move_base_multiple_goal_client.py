#!/usr/bin/env python

import rospy
import actionlib
import sys
import yaml as yaml

from std_msgs.msg import String as String
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Twist as Twist

class MoveBaseClient:
    def __init__(self, poses, s_time):
        #self.move_base_goal = goal
        self.poses = poses
        self.start_time = s_time
        self.goals = []
        self.set_goals()
        self.goal_time = 0.0
        self.succeeded = False
        self.abort = False
        self.init_actionclient()
        self.goal_pub = rospy.Publisher('/move_base_goal', String, queue_size = 10)
        
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel',Twist,self.twist_cb)
        self.score = 0.0
    
    def set_goals(self):
        #print self.poses
        for i in range(len(self.poses)):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = self.poses[i][1]
            goal.target_pose.pose.position.y = self.poses[i][2]
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = 0.0
            goal.target_pose.pose.orientation.w = 1.0
            self.goals.append(goal)

    def twist_cb(self, data):
        val = data.linear.x
        if (val < 0.0):
            self.score = self.score - 1.0
    
    def init_actionclient(self):
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('waiting for move_base action server')
        self.move_base_client.wait_for_server()
        rospy.loginfo('move_base action server available')

    def client_call(self):
        self.first_goal_time = rospy.get_rostime()
        for goal in self.goals:
            goal.target_pose.header.stamp = rospy.get_rostime()

            rospy.loginfo('Sending goal: %s', goal.target_pose.pose)
            self.move_base_client.send_goal(goal)
            rospy.loginfo("Waiting for result")
            if self.move_base_client.wait_for_result(rospy.Duration(45.0)):
                rospy.loginfo("Goal reached")
                #info = self.get_info()
                #self.goal_pub.publish(info)
            else:
                rospy.logwarn("Goal not reached. Aborting!")
                self.abort = True
                break
        self.goal_time = rospy.get_rostime()
        self.succeeded = True


    def get_info(self):
        temp = '  Start_Time: '+str(self.start_time)+'\n  '+ '1_Goal_Sended: '+str(self.first_goal_time)+'\n  '+ 'Finished_Time: '+str(self.goal_time)+'\n  '+ 'Start-1_Goal-Duration: '+str((self.first_goal_time - self.start_time).to_sec()) + '\n  ' + 'MoveBase_Duration: '+str((self.goal_time - self.first_goal_time).to_sec())
        print temp
        return temp
            
    def run(self):
        if self.succeeded or self.abort:
            info = self.get_info()
            if self.abort:
                info = info + '\n  Status: Aborted'
            else:
                info = info + '\n Status: Succeded'
            rospy.loginfo("Publish Infos")
            for i in range(10):
                self.goal_pub.publish(info)


def main():
    rospy.init_node('MoveBaseClient')
    rospy.loginfo('MoveBaseClient')
    while (not rospy.get_rostime()):
        pass
    s_time = rospy.get_rostime()
    path = '/home/alex/wentz_catkin_ws/src/youbot_simulation/tracker_auto_sim/locations.yaml'
    loc_file = {}
    try:
        rospy.loginfo("Loading: %s",path)
        file = open(path)
        loc_file = yaml.load(file)
    except yaml.YAMLError as exc:
        print(exc)
    file.close()
    count = len(loc_file)
    poses = []
    for c in range(count):
        poses.append(loc_file[c])
    move_base_c = MoveBaseClient(poses, s_time)

    rate = rospy.Rate(20)
    move_base_c.client_call()
    while not rospy.is_shutdown():
        rate.sleep()
        move_base_c.run()
        if move_base_c.succeeded or move_base_c.abort:
            break

if __name__ == '__main__':
    main()
