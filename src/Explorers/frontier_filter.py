#!/usr/bin/env python

from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from sklearn.cluster import MeanShift
from rbe3002.msg import PointArray
import std_msgs.msg
import numpy as np
import rospy
import math
import copy


class FrontierFilter(object):

    def __init__(self):
        rospy.init_node("FrontierFilter")
        rospy.loginfo("Initializing Frontier Filtering node.")
        self.latest_frontiers_pub = rospy.Subscriber(
            '/OpenCVFrontierDetector/detected_points', PointStamped, self.update_frontiers, queue_size=5)
        self.rtab_map_sub = rospy.Subscriber(
            '/MNCLGlobalCostmap/cspace', OccupancyGrid, self.update_rtabmap, queue_size=1)
        # self.potential_frontiers_pub = rospy.Subscriber(
        #     '/OpenCVFrontierDetector/potential_frontiers', PointArray, self.update_frontiers, queue_size=1)
        self.loc_or_map_mode_sub = rospy.Subscriber(
            '/MNCLGlobalController/loc_or_map_mode', std_msgs.msg.Bool, self.update_loc_or_map, queue_size=1)
        self.assigned_points_pub = rospy.Publisher(
            'frontier_filter/filtered_points_markers', Marker, queue_size=1)
        # Start by mapping until frontier search algorithm visits a certain percentage of cells
        self.loc_or_map_mode = False
        self.frontiers = []
        self.unkown_indices = []
        self.marker = Marker()
        self.first_map = True
        # self.frontiers = PointArray()
        self.rtabmap = OccupancyGrid()
        rospy.sleep(1)
        rospy.loginfo("FrontierFilter node ready.")

    def update_rtabmap(self, map):
        # if not self.loc_or_map_mode:
        if self.first_map:
            self.marker.header.frame_id = map.header.frame_id
            self.marker.header.stamp = rospy.Time.now()
            self.marker.ns = "filtered_markers"
            self.marker.id = 0
            self.marker.type = Marker.POINTS
            self.marker.action = Marker.ADD
            self.marker.pose.orientation.w = 1.0
            self.marker.scale.x = self.marker.scale.y = 0.3
            (self.marker.color.r, self.marker.color.g, self.marker.color.b,
            self.marker.color.a) = (255.0/255.0, 0.0/255.0, 255.0/255.0, 1)
            self.marker.lifetime == rospy.Duration()
            self.first_map = False
        self.rtabmap = map

    def update_frontiers(self, frontier):
        # if not self.loc_or_map_mode:
            # for i in range(len(frontiers.points)):
            #     if frontiers.points[i] not in self.frontiers.points:
            #         self.frontiers.points.append(frontiers.points[i])
        if len(self.frontiers) > 0:
            self.frontiers = np.vstack((self.frontiers, [np.array([frontier.point.x, frontier.point.y])]))
        else:
            self.frontiers = [np.array([frontier.point.x, frontier.point.y])]

    def update_loc_or_map(self, loc_or_map):
        self.loc_or_map_mode = loc_or_map


class SKLearnFilter(FrontierFilter):

    def __init__(self):
        super(SKLearnFilter, self).__init__()
        rospy.loginfo("Initializing SKLearnFilter node.")
        self.filter_pub = rospy.Publisher(
            '/frontier_filter/filtered_points', PointArray, queue_size=10)
        self.info_radius = rospy.get_param('info_radius')
        self.threshold = rospy.get_param('threshold')
        rospy.sleep(2)
        # while self.rtabmap.header.seq > 1 and len(self.rtabmap.data) > 1:
        #     pass
        rospy.loginfo("SKLearnFilter node ready.")
        self.filter_potential_frontiers()

    def filter_potential_frontiers(self):
        point = PointStamped()
        point.header.frame_id = 'map'
        point_array = PointArray()
        while not rospy.is_shutdown(): #and not self.loc_or_map_mode:
            self.marker.points = []
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
                mapdata = self.rtabmap
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
                rospy.sleep(0.01)

    def informationGain(self, point, radius):
        mapdata = self.rtabmap
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

    def run(self):
        # exit the node to free up resources when we are done mapping
        # if not self.loc_or_map_mode:
            rospy.spin()

if __name__ == '__main__':
    SKLearnFilter().run()
