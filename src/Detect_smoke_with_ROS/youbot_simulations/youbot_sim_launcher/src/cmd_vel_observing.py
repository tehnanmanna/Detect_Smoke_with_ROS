#!/usr/bin/env python

import rospy

import numpy as np
from std_msgs.msg import String as String
from geometry_msgs.msg import Twist as Twist

class SimpleCmdVelObserver:
    def __init__(self,rate):
        self.rate = rate
        self.start = False
        self.s_time = None
        self.e_time = None
        self.cmd_sub = rospy.Subscriber("/gazebo/cmd_vel", Twist, self.cmd_vel_callback)
        self.goal_pub = rospy.Publisher('/cmd_vel_zero', String, queue_size = 1)

    def cmd_vel_callback(self, data):
        if not self.start:
            vel = np.abs(data.linear.x)
            if vel > 0.005:
                self.start = True
                self.s_time = rospy.get_time()
        elif self.start:
            vel = np.abs(data.linear.x)
            if vel < 0.001:
                self.e_time = rospy.get_time()
                self.publish_info()
                self.reset()

    def publish_info(self):
        self.goal_pub.publish("Finished after: "+str(self.e_time - self.s_time))


    def reset():
        self.start = False

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

def main():
#    if not len(sys.argv) == 4:
#        print("[CMD_VEL_PUB] scripts expects 3 parameters(vx, vy, vtheta)")
#        return -1
#    cmd_vel = Twist()
#    cmd_vel.linear.x = float(sys.argv[1])
#    cmd_vel.linear.y = float(sys.argv[2])
#    cmd_vel.angular.z = float(sys.argv[3])

    rospy.init_node('cmd_vel_observing')
    rospy.loginfo('cmd_vel_observing')
    rate = rospy.Rate(30)
    now = rospy.get_rostime()
    while now.secs == 0 and now.nsecs == 0:
        now = rospy.get_rostime()
        rate.sleep()
    cmd_vel_o = SimpleCmdVelObserver(rate)
    cmd_vel_o.run()

if __name__ == '__main__':
    main()
