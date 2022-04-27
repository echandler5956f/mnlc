#!/usr/bin/env python
import actionlib
import roslib
import rospy

roslib.load_manifest('rbe3002')

class mnlc_local_rrt_detector():

    def __init__(self):
        rospy.loginfo("Initializing mnlc_local_rrt_detector.")
        rospy.init_node("mnlc_local_rrt_detector")
        
        rospy.sleep(1)
        rospy.loginfo("mnlc_local_rrt_detector node ready.")


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    mnlc_local_rrt_detector().run()

