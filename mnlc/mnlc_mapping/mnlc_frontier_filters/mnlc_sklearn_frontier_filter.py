#!/usr/bin/env python
import actionlib
import roslib
import rospy

roslib.load_manifest('rbe3002')

class mnlc_sklearn_frontier_filter():

    def __init__(self):
        rospy.loginfo("Initializing mnlc_sklearn_frontier_filter.")
        rospy.init_node("mnlc_sklearn_frontier_filter")
        
        rospy.sleep(1)
        rospy.loginfo("mnlc_sklearn_frontier_filter node ready.")


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    mnlc_sklearn_frontier_filter().run()

