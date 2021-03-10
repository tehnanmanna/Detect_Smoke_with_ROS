#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty as Empty
from gazebo_msgs.msg import ContactsState as ContactsState
#from gazebo_msgs.msg import ContactState as ContactState

''' 
This node call the observer debug service each time there is a collision
in the simulation.
'''

class BaseBumperCounter:
    def __init__(self):
        self.enabled = False
        self.base_bumper_sub = rospy.Subscriber('/youbot_base_bumper', ContactsState, self.collision_cb)
        rospy.sleep(5.0)
        self.debug_service = rospy.ServiceProxy('/collision_counter', Empty)
        self.enabled = True
        
    def collision_cb(self, data):
        if data.states and self.enabled:
            self.debug_service()

def main():
    rospy.init_node('BaseBumperCounter')
    rospy.loginfo('BaseBumperCounter')
    
    rate = rospy.Rate(30)
    collision_counter = BaseBumperCounter()
    
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
