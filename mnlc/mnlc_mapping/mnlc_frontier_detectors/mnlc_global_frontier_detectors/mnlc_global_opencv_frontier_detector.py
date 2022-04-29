#!/usr/bin/env python

from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import OccupancyGrid, GridCells
from visualization_msgs.msg import Marker
from rbe3002.msg import PointArray
from nav_msgs.srv import GetMap
import move_base_msgs.msg as mb
import rbe3002.msg as rbe
import std_srvs.srv
import std_msgs.msg
import numpy as np
import actionlib
import roslib
import rospy
import math
import copy
import sys
import cv2
import tf

roslib.load_manifest('rbe3002')


class mnlc_global_opencv_frontier_detector():

    def __init__(self):
        self.error = False
        rospy.loginfo("Initializing mnlc_global_opencv_frontier_detector.")
        rospy.init_node("mnlc_global_opencv_frontier_detector",
                        disable_signals=True)
        self.initialize_params()
        rospy.sleep(self.timeout * 7)
        # give gazebo a chance to warm up so rtabmap doesnt raise an error about not having a map
        self.safe_start()
        rospy.loginfo("mnlc_global_opencv_frontier_detector node ready.")

    def initialize_params(self):
        # grid cost to be considered an obstacle
        self.obstacle_cost = rospy.get_param('obstacle_cost', 90)
        self.ctrl_invl = rospy.get_param(
            'ctrl_invl', 0.01)  # [s] control loop interval
        self.ctrl_rate = rospy.Rate(1/self.ctrl_invl)
        # [s] standard service timeout limit
        self.timeout = rospy.get_param('timeout', 1.0)
        # number of grid cells to pad the c_scpace with
        self.padding = rospy.get_param('padding', 4)
        # width-number of cells left and right to add to the visited list
        self.visited_width = rospy.get_param('visited_width', 20)
        # height-number of cells above and below to add to the visited list
        self.visited_height = rospy.get_param('visited_height', 20)
        self.listener = tf.TransformListener()
        self.map_metadata = OccupancyGrid()
        self.visited = []
        self.points = Marker()
        self.cspace = None
        # t = rospy.Timer(self.ctrl_invl, self.update_map())
        self.latest_map = OccupancyGrid()
        rtab_map_pub = rospy.Subscriber('/latest_map', OccupancyGrid, self.update_map, queue_size=1)


    def initialize_marker(self, map):
        self.points.header.frame_id = map.header.frame_id
        self.points.header.stamp = rospy.Time.now()
        self.points.ns = "markers"
        self.points.id = 0
        self.points.type = Marker.POINTS
        self.points.action = Marker.ADD
        self.points.pose.orientation.w = 1.0
        self.points.scale.x = self.points.scale.y = 0.3
        (self.points.color.r, self.points.color.g, self.points.color.b,
         self.points.color.a) = (255.0/255.0, 0.0/255.0, 0.0/255.0, 1)
        self.points.lifetime == rospy.Duration()

    def safe_start(self):
        rospy.wait_for_service(
            '/rtabmap/get_map', timeout=rospy.Duration(self.timeout))
        try:
            m = rospy.ServiceProxy('/rtabmap/get_map', GetMap)
        except rospy.ServiceException as e:
            rospy.logerr("RTabMap Mappping service call failed: %s" % e)
            self.error_handler()
            return
        rospy.loginfo("RTabMap Mappping service call successful.")
        map = m()
        if map.map.info.resolution == 0:
            rospy.logerr(
                "RTabMap has zero resolution. Exiting...")
            self.error_handler()
            return
        self.map_metadata = map.map
        self.initialize_marker(map.map)
        rospy.wait_for_service(
            '/begin_phase1', timeout=rospy.Duration(self.timeout))
        try:
            tmp = rospy.ServiceProxy('/begin_phase1', std_srvs.srv.Empty)
        except rospy.ServiceException as e:
            rospy.logerr("Begin phase1 service call failed: %s" % e)
            self.error_handler()
            return
        rospy.loginfo("Begin phase1 service call successful.")
        temp = tmp()
        self.listener.waitForTransform(
            '/odom', '/base_footprint', rospy.Time(0), timeout=rospy.Duration(self.timeout))
        flag = 0
        while flag == 0:
            try:
                rospy.loginfo('Waiting for the robot transform')
                (trans, rot) = self.listener.lookupTransform(
                    '/odom', '/base_footprint', rospy.Time(0))
                flag = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.error_handler()
                return
        self.detected_points_pub = rospy.Publisher(
            '/OpenCVFrontierDetector/detected_points', PointStamped, queue_size=10)
        self.shapes_pub = rospy.Publisher(
            'OpenCVFrontierDetector/shapes', Marker, queue_size=1)
        self.detect_frontiers()

    def detect_frontiers(self):
        exploration_goal = PointStamped()
        exploration_goal.header.frame_id = self.points.header.frame_id
        exploration_goal.point.z = 0
        while not rospy.is_shutdown():
            latest_map = self.latest_map
            img_map = np.zeros(
                (latest_map.info.height, latest_map.info.width, 1), np.uint8)
            for i in range(0, latest_map.info.height):
                for j in range(0, latest_map.info.width):
                    if latest_map.data[i * latest_map.info.width + j] == 100:
                        img_map[i, j] = 0
                    elif latest_map.data[i * latest_map.info.width + j] == 0:
                        img_map[i, j] = 255
                    elif latest_map.data[i * latest_map.info.width + j] == -1:
                        img_map[i, j] = 205
            original = cv2.inRange(img_map, 0, 1)
            edges = cv2.Canny(img_map, 0, 255)
            tmp, contours, hierarchy = cv2.findContours(
                original, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(original, contours, -1, (255, 255, 255), 5)
            original = cv2.bitwise_not(original)
            result = cv2.bitwise_and(original, edges)
            frontier = copy.copy(result)
            tmp, contours, hierarchy = cv2.findContours(
                frontier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(frontier, contours, -1, (255, 255, 255), 2)
            tmp, contours, hierarchy = cv2.findContours(
                frontier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            frontiers = []
            if len(contours) > 0:
                for contour in contours:
                    moment = cv2.moments(contour)
                    cx = int(moment['m10']/moment['m00'])
                    cy = int(moment['m01']/moment['m00'])
                    p = (cx, cy)
                    if p not in self.visited:
                        xr = cx * latest_map.info.resolution + latest_map.info.origin.position.x
                        yr = cy * latest_map.info.resolution + latest_map.info.origin.position.y
                        point = [np.array([xr, yr])]
                        if len(frontiers) > 0:
                            frontiers = np.vstack([frontiers, point])
                        else:
                            frontiers = point
            for frontier in frontiers:
                exploration_goal.header.stamp = rospy.Time(0)
                exploration_goal.point.x = frontier[0]
                exploration_goal.point.y = frontier[1]
                self.detected_points_pub.publish(exploration_goal)
                self.points.points = [exploration_goal.point]
                self.shapes_pub.publish(self.points)

    def update_visited(self):
        flag = 0
        while flag == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(
                    "odom", "base_footprint", rospy.Time(0))
                flag = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return
        x = int(math.floor(
            (trans[0] - self.map_metadata.info.origin.position.x) / self.map_metadata.info.resolution))
        y = int(math.floor(
            (trans[1] - self.map_metadata.info.origin.position.y) / self.map_metadata.info.resolution))
        visited_width = self.visited_width
        visited_height = self.visited_height
        for i in range(int(x) - int(math.floor(visited_width/2)), int(x) + int(math.floor(visited_width/2))):
            for j in range(int(y) - int(math.floor(visited_height/2)), int(y) + int(math.floor(visited_height/2))):
                point = (i, j)
                if point not in self.visited:
                    self.visited.append(point)

    def safe_start_phase_2(self):
        rospy.signal_shutdown(
            "Mapping complete. Node is now uneeded. Shutting down mnlc_global_opencv_frontier_detector.")
        sys.exit(0)

    def update_map(self, map):
        self.latest_map = map
        # self.latest_map = cv2.imread(r'/home/quant/.ros/global_costmap.pgm', -1)

    def error_handler(self):
        self.error = True

    def run(self):
        while not rospy.is_shutdown() and self.error == False:
            rospy.spin()


if __name__ == '__main__':
    try:
        mnlc_global_opencv_frontier_detector().run()
    except rospy.ROSInterruptException:
        pass
