#!/usr/bin/env python
import actionlib
import roslib
import rospy

roslib.load_manifest('rbe3002')

class mnlc_d_star():

    def __init__(self):
        rospy.loginfo("Initializing mnlc_d_star.")
        rospy.init_node("mnlc_d_star")
        
        rospy.sleep(1)
        rospy.loginfo("mnlc_d_star node ready.")


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    mnlc_d_star().run()

