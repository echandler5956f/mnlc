#!/usr/bin/env python

from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from sklearn.cluster import MeanShift
from rbe3002.msg import PointArray
from nav_msgs.srv import GetMap
import std_srvs.srv
import std_msgs.msg
import numpy as np
import roslib
import rospy
import math
import copy
import tf

roslib.load_manifest('rbe3002')

class mnlc_sklearn_frontier_filter():

    def __init__(self):
        rospy.loginfo("Initializing mnlc_sklearn_frontier_filter.")
        rospy.init_node("mnlc_sklearn_frontier_filter")
        self.initialize_params()
        rospy.sleep(self.timeout * 5)
        # give gazebo a chance to warm up so rtabmap doesnt raise an error about not having a map
        rospy.loginfo("mnlc_sklearn_frontier_filter node ready.")
        self.safe_start()

    def initialize_params(self):
        # grid cost to be considered an obstacle
        self.obstacle_cost = rospy.get_param('obstacle_cost', 90)
        self.ctrl_invl = rospy.get_param(
            'ctrl_invl', 0.01)  # [s] control loop interval
        self.ctrl_rate = rospy.Rate(1/self.ctrl_invl)
        # [s] standard service timeout limit
        self.timeout = rospy.get_param('timeout', 1.0)
        self.filter_clear = rospy.get_param('filter_clear', 0.0)
        self.info_radius = rospy.get_param('info_radius', 1.0)
        self.threshold = rospy.get_param('threshold', 70)
        self.marker = Marker()
        self.latest_map = OccupancyGrid()
        self.frontiers = []
        self.next_time = rospy.get_time()
        self.detected_points_sub = rospy.Subscriber(
            '/detected_points', PointStamped, self.update_frontiers, queue_size=10)
        rtab_map_sub = rospy.Subscriber('/latest_map', OccupancyGrid, self.update_map, queue_size=1)
        self.assigned_points_pub = rospy.Publisher(
            '/frontier_filter/filtered_points_markers', Marker, queue_size=10)
        self.filter_pub = rospy.Publisher(
            '/frontier_filter/filtered_points', PointArray, queue_size=10)

    def initialize_marker(self, map):
        self.marker.header.frame_id = map.header.frame_id
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "filtered_markers"
        self.marker.id = 4
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = self.marker.scale.y = 0.3
        (self.marker.color.r, self.marker.color.g, self.marker.color.b,
        self.marker.color.a) = (255.0/255.0, 255.0/255.0, 0.0/255.0, 1)
        self.marker.lifetime == rospy.Duration()

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
        self.detected_points_pub = rospy.Subscriber(
            '/OpenCVFrontierDetector/detected_points', PointStamped, queue_size=10)
        self.shapes_pub = rospy.Publisher(
            'OpenCVFrontierDetector/shapes', Marker, queue_size=10)
        self.filter_potential_frontiers()

    def filter_potential_frontiers(self):
        point = PointStamped()
        point.header.frame_id = 'map'
        point_array = PointArray()
        self.marker.points = []
        while not rospy.is_shutdown():
            time_init = rospy.get_time()
            centroids = []
            front = copy.copy(self.frontiers)
            if len(front) > 1:
                ms = MeanShift(bandwidth = 0.3)
                ms.fit(front)
                centroids = ms.cluster_centers_
            if len(front) == 1:
                centroids = front
            self.frontiers = copy.copy(centroids)
            i = 0
            while i < len(centroids):
                mapdata = self.latest_map
                cond = False
                point.point.x = centroids[i][0]
                point.point.y = centroids[i][1]
                x = np.array([point.point.x,
                              point.point.y])
                cond = (mapdata.data[int((math.floor((x[1] - mapdata.info.origin.position.y)/mapdata.info.resolution) *
                                          mapdata.info.width) + (math.floor((x[0] - mapdata.info.origin.position.x)/mapdata.info.resolution)))] > self.threshold) or cond
                if (cond or (self.informationGain([centroids[i][0], centroids[i][1]], self.info_radius * 0.5)) < 0.2):
                    centroids = np.delete(centroids, (i), axis=0)
                    i = i - 1
                i += 1
                point_array.points = []
                for cent in centroids:
                    point.point.x = cent[0]
                    point.point.y = cent[1]
                    point_array.points.append(copy.copy(point))
                    self.marker.points=[point.point]
                self.filter_pub.publish(point_array)
                self.assigned_points_pub.publish(self.marker)
            print("Calculating mnlc Filter took: ", rospy.get_time() - time_init, ".")

    def update_frontiers(self, frontier):
        if rospy.get_time() > self.next_time:
            self.frontiers = []
            self.next_time = rospy.get_time() + self.filter_clear
        if len(self.frontiers) > 0:
            self.frontiers = np.vstack((self.frontiers, [np.array([frontier.point.x, frontier.point.y])]))
        else:
            self.frontiers = [np.array([frontier.point.x, frontier.point.y])]

    def informationGain(self, point, radius):
        mapdata = self.latest_map
        infoGain = 0
        index = int((math.floor((point[1] - mapdata.info.origin.position.y)/mapdata.info.resolution) *
                     mapdata.info.width) + (math.floor((point[0] - mapdata.info.origin.position.x)/mapdata.info.resolution)))
        r_region = int(radius/mapdata.info.resolution)
        init_index = index-r_region * (mapdata.info.width + 1)
        for n in range(0, 2 * r_region + 1):
            start = n * mapdata.info.width + init_index
            end = start + 2 * r_region
            limit = ((start/mapdata.info.width) + 2) * mapdata.info.width
            for i in range(start, end + 1):
                if (i >= 0 and i < limit and i < len(mapdata.data)):
                    if(mapdata.data[i] == -1 and np.linalg.norm(np.array(point)-np.array([mapdata.info.origin.position.x +
                                                                                          (i - (i//mapdata.info.width) * (mapdata.info.width)) * mapdata.info.resolution, mapdata.info.origin.position.y +
                                                                                          (i//mapdata.info.width) * mapdata.info.resolution])) <= radius):
                        infoGain += 1
        return infoGain * (mapdata.info.resolution ** 2)

    def update_map(self, map):
        self.latest_map = map

    def error_handler(self):
        self.error = True
        
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    mnlc_sklearn_frontier_filter().run()

