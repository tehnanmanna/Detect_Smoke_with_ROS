#!/usr/bin/env python
import rospy

from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import LinkState

class LinkStateClient:
    def __init__(self):
        self.sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.links_cb, queue_size=1)
        self.pub = rospy.Publisher("/robot/footprint/state", LinkState, queue_size = 1)
        self.robot_base_name = "youbot::base_footprint"

    def links_cb(self,data):
        pos = None
        try:
            index = data.name.index(self.robot_base_name)
            pos = data.pose[index]
            vel = data.twist[index]
        except Exception as e:
            print e

        if pos:
            ls = LinkState()
            ls.reference_frame = "/world"
            ls.pose = pos
            ls.twist = vel
            ls.link_name = self.robot_base_name
            self.pub.publish(ls)

def main():
    rospy.init_node('Link_states_single_link_pub')
    rospy.loginfo('Link_states_single_link_pub')

    rate = rospy.Rate(20)
    client = LinkStateClient()
    while not rospy.is_shutdown():
        rate.sleep()

if __name__=="__main__":
    main()

