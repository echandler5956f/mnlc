#!/usr/bin/env python

from sklearn.cluster import MeanShift, estimate_bandwidth
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from rbe3002.msg import PointArray
from nav_msgs.srv import GetMap
import std_srvs.srv
import numpy as np
import roslib
import rospy
import math
import copy

roslib.load_manifest('rbe3002')


class mnlc_sklearn_frontier_filter():

    def __init__(self):
        self.error = False
        rospy.loginfo("Initializing mnlc_sklearn_frontier_filter.")
        rospy.init_node("mnlc_sklearn_frontier_filter")
        self.initialize_params()
        rospy.sleep(self.start_time)
        # give gazebo a chance to warm up so rtabmap doesnt raise an error about not having a map
        rospy.loginfo("mnlc_sklearn_frontier_filter node ready.")
        self.safe_start()

    def initialize_params(self):
        self.start_time = rospy.get_param('/frontier_filter/start_time')
        # grid cost to be considered an obstacle
        self.obstacle_cost = rospy.get_param('/frontier_filter/obstacle_cost')
        self.timeout = rospy.get_param('/controller/timeout')
        self.filter_clear = rospy.get_param('/frontier_filter/filter_clear')
        self.info_radius = rospy.get_param('/frontier_filter/info_radius')
        self.marker = Marker()
        self.latest_map = OccupancyGrid()
        self.frontiers = []
        self.next_time = rospy.get_time()
        self.detected_points_sub = rospy.Subscriber(
            '/detected_points', PointStamped, self.update_frontiers, queue_size=1000)
        rtab_map_sub = rospy.Subscriber(
            '/mnlc_global_costmap_opencv/cspace', OccupancyGrid, self.update_map, queue_size=1)
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
        self.filter_potential_frontiers()

    def filter_potential_frontiers(self):
        point = PointStamped()
        point.header.frame_id = 'map'
        point_array = PointArray()
        self.marker.points = []
        while not rospy.is_shutdown():
            time_init = rospy.get_time()
            mapdata = self.latest_map
            centroids = []
            front = copy.copy(self.frontiers)
            if len(front) > 1:
                bandwidth = estimate_bandwidth(front, quantile=0.3)
                if bandwidth == 0.0:
                    bandwidth = 0.3
                ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
                ms.fit(front)
                centroids = ms.cluster_centers_
            if len(front) == 1:
                centroids = front
            self.frontiers = copy.copy(centroids)
            i = 0
            while i < len(centroids):
                cond = False
                point.point.x = centroids[i][0]
                point.point.y = centroids[i][1]
                x = np.array([point.point.x,
                              point.point.y])
                grid = np.array([int(math.floor(x[0] - mapdata.info.origin.position.x) /
                                     mapdata.info.resolution), int(math.floor((x[1] - mapdata.info.origin.position.y) /
                                                                              mapdata.info.resolution))])
                data = int(
                    mapdata.data[grid[0] + (grid[1] * mapdata.info.width)])
                cond = data >= int(self.obstacle_cost) or data == -1
                centroid = np.array([centroids[i][0], centroids[i][1]])
                radius = self.info_radius
                info_gain = 0
                index = int((math.floor((centroid[1] - mapdata.info.origin.position.y)/mapdata.info.resolution) *
                             mapdata.info.width) + (math.floor((centroid[0] - mapdata.info.origin.position.x)/mapdata.info.resolution)))
                r_region = int(radius/mapdata.info.resolution)
                init_index = index-r_region * (mapdata.info.width + 1)
                for n in range(0, 2 * r_region + 1):
                    start = n * mapdata.info.width + init_index
                    end = start + 2 * r_region
                    limit = ((start/mapdata.info.width) + 2) * \
                        mapdata.info.width
                    for j in range(start, end + 1):
                        if (j >= 0 and j < limit and j < len(mapdata.data)):
                            p = np.array([mapdata.info.origin.position.x + (j - (j/mapdata.info.width) * (mapdata.info.width)) * mapdata.info.resolution,
                                          mapdata.info.origin.position.y + (j/mapdata.info.width) * mapdata.info.resolution])
                            if (mapdata.data[j] == -1 and np.linalg.norm(centroid-p) <= radius):
                                info_gain += 1
                info_gain = info_gain * (mapdata.info.resolution ** 2)
                if cond or info_gain < 0.2:
                    centroids = np.delete(centroids, (i), axis=0)
                    i = i - 1
                i += 1
                point_array.points = []
                self.marker.points = []
            if len(centroids != 0):
                for cent in centroids:
                    point.point.x = cent[0]
                    point.point.y = cent[1]
                    point_array.points.append(copy.copy(point))
                    self.marker.points.append(copy.copy(point.point))
                self.filter_pub.publish(point_array)
                self.assigned_points_pub.publish(self.marker)
            # print("Calculating mnlc Filter took: ", rospy.get_time() - time_init, ".")

    def update_frontiers(self, frontier):
        if len(self.frontiers) > 0:
            self.frontiers = np.vstack(
                (self.frontiers, [np.array([frontier.point.x, frontier.point.y])]))
        else:
            self.frontiers = [np.array([frontier.point.x, frontier.point.y])]

    def update_map(self, map):
        self.latest_map = map

    def error_handler(self):
        self.error = True

    def run(self):
        while not rospy.is_shutdown() and self.error == False:
            rospy.spin()


if __name__ == '__main__':
    try:
        mnlc_sklearn_frontier_filter().run()
    except rospy.ROSInterruptException:
        pass
