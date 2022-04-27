#!/usr/bin/env python
import actionlib
import roslib
import rospy

roslib.load_manifest('rbe3002')

class mnlc_dwa():

    def __init__(self):
        rospy.loginfo("Initializing mnlc_dwa.")
        rospy.init_node("mnlc_dwa")
        
        rospy.sleep(1)
        rospy.loginfo("mnlc_dwa node ready.")


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    mnlc_dwa().run()

