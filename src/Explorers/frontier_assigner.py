#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped, PointStamped
from rbe3002.msg import PointArray, Pose2d
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
import std_msgs.msg
import numpy as np
import rospy
import copy
import math


class FrontierAssigner(object):

    def __init__(self):
        rospy.init_node("FrontierAssigner")
        rospy.loginfo("Initializing Frontier Assigner node.")
        self.info_radius = rospy.get_param('info_radius')
        self.info_multiplier = rospy.get_param('info_multiplier')
        self.hysteresis_radius = rospy.get_param('hysteresis_radius')
        self.hysteresis_gain = rospy.get_param('hysteresis_gain')
        self.rtab_map_sub = rospy.Subscriber(
            '/MNCLGlobalCostmap/cspace', OccupancyGrid, self.update_rtabmap, queue_size=1)
        self.filtered_frontiers_sub = rospy.Subscriber(
            '/frontier_filter/filtered_points', PointArray, self.update_filtered_frontiers, queue_size=10)
        self.pose2d_sub = rospy.Subscriber(
            '/MNCLGlobalController/pose2d', Pose2d, self.update_pose2d, queue_size=1)
        self.assigned_goal = rospy.Publisher(
            'move_base_simple/goal', PoseStamped, queue_size=1)
        self.is_busy_sub = rospy.Subscriber(
            "/MNCLGlobalCostmap/is_busy", std_msgs.msg.Bool, self.update_is_busy, queue_size=1)
        self.loc_or_map_mode_sub = rospy.Subscriber(
            '/MNCLGlobalController/loc_or_map_mode', std_msgs.msg.Bool, self.update_loc_or_map, queue_size=1)
        self.assigned_points_pub = rospy.Publisher(
            'OpenCVFrontierAssigner/goal_points', Marker, queue_size=1)
        # Start by mapping until frontier search algorithm visits a certain percentage of cells
        self.is_busy = False
        self.first_map = True
        self.loc_or_map_mode = False
        self.pose2d = Pose2d()
        self.filtered_frontiers = []
        self.rtabmap = OccupancyGrid()
        self.points = Marker()
        rospy.sleep(1)
        rospy.loginfo("Frontier Assigner node ready.")

    def update_rtabmap(self, map):
        # if not self.loc_or_map_mode:
        if self.first_map:
            self.points.header.frame_id = map.header.frame_id
            self.points.header.stamp = rospy.Time.now()
            self.points.ns = "goal_markers"
            self.points.id = 0
            self.points.type = Marker.POINTS
            self.points.action = Marker.ADD
            self.points.pose.orientation.w = 1.0
            self.points.scale.x = self.points.scale.y = 0.3
            (self.points.color.r, self.points.color.g, self.points.color.b,
            self.points.color.a) = (0.0/255.0, 255.0/255.0, 0.0/255.0, 1)
            self.points.lifetime == rospy.Duration()
            self.first_map = False
        self.rtabmap = map

    def update_pose2d(self, pose2d):
        # if not self.loc_or_map_mode:
            self.pose2d = pose2d

    def update_is_busy(self, is_busy):
        self.is_busy = is_busy

    def update_filtered_frontiers(self, point_array):
        # if not self.loc_or_map_mode:
            # self.filtered_frontiers = filtered_frontiers
            # print(self.filtered_frontiers)
        for point in point_array.points:
            self.filtered_frontiers.append(np.array([point.point.x, point.point.y]))

    def update_loc_or_map(self, loc_or_map):
        self.loc_or_map_mode = loc_or_map


class MapFrontierAssigner(FrontierAssigner):

    def __init__(self):
        super(MapFrontierAssigner, self).__init__()
        rospy.loginfo("Initializing MapFrontierAssigner node.")
        rospy.sleep(2)
        # while self.rtabmap.header.seq > 1 and len(self.rtabmap.data) > 1:
        #     pass
        rospy.loginfo("MapFrontierAssigner node ready.")
        self.map_frontier_assigner()

    def map_frontier_assigner(self):
        exploration_goal = PointStamped()
        exploration_goal.header.frame_id = self.points.header.frame_id
        exploration_goal.point.z = 0
        while not rospy.is_shutdown(): # and not self.loc_or_map_mode:
            # if len(self.filtered_frontiers.points) > 0:
                # print(self.filtered_frontiers.points)
                centroids = copy.copy(self.filtered_frontiers)
                info_gain = []
                for i in range(len(centroids)):
                    info_gain.append(self.informationGain(
                        [centroids[i][0], centroids[i][1]]))
                info_gain = self.discount([self.pose2d.cx, self.pose2d.cy], centroids, info_gain)
                rev_rec = []
                centroid_rec = []
                for i in range(len(centroids)):
                    cost = np.linalg.norm([self.pose2d.cx -centroids[i][0], self.pose2d.cy - centroids[i][1]])
                    information_gain = info_gain[i]
                    if np.linalg.norm([self.pose2d.cx - centroids[i][0], self.pose2d.cy -  centroids[i][1]]) <= self.hysteresis_radius:
                        information_gain *= self.hysteresis_gain
                    rev = information_gain * self.info_multiplier - cost
                    rev_rec.append(rev)
                    centroid_rec.append(centroids[i])
                # print(rev_rec)
                if len(rev_rec) >= 1:
                    best_frontier = centroid_rec[rev_rec.index(max(rev_rec))]
                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = '/map'
                    goal_pose.header.stamp = rospy.Time.now()
                    goal_pose.pose.position.x = best_frontier[0]
                    goal_pose.pose.position.y = best_frontier[1]
                    # self.assigned_goal.publish(goal_pose)
                    exploration_goal.header.stamp = rospy.Time(0)
                    exploration_goal.point.x =  goal_pose.pose.position.x
                    exploration_goal.point.y = goal_pose.pose.position.y
                    self.points.points = [exploration_goal.point]
                    self.assigned_points_pub.publish(self.points)
                    # print(goal_pose)
                    # wait for the path to finish before calculating the next goal frontier
                    # while self.is_busy:
                    #     pass

    def informationGain(self, point):
        mapdata = self.rtabmap
        info_gain = 0
        index = int((math.floor((point[1] - mapdata.info.origin.position.y)/mapdata.info.resolution) *
                     mapdata.info.width) + (math.floor((point[0] - mapdata.info.origin.position.x)/mapdata.info.resolution)))
        r_region = int(self.info_radius/mapdata.info.resolution)
        init_index = index-r_region * (mapdata.info.width + 1)
        for n in range(0, 2 * r_region + 1):
            start = n * mapdata.info.width + init_index
            end = start + 2 * r_region
            limit = ((start/mapdata.info.width) + 2) * mapdata.info.width
            for i in range(start, end + 1):
                if (i >= 0 and i < limit and i < len(mapdata.data)):
                    if(mapdata.data[i] == -1 and np.linalg.norm(np.array(point)-np.array([mapdata.info.origin.position.x +
                                                                                          (i - (i//mapdata.info.width) * (mapdata.info.width)) * mapdata.info.resolution, mapdata.info.origin.position.y +
                                                                                          (i//mapdata.info.width) * mapdata.info.resolution])) <= self.info_radius):
                        info_gain += 1
        return info_gain * (mapdata.info.resolution ** 2)

    def discount(self, point, centroids, info_gain):
        mapdata = self.rtabmap
        index = int((math.floor((point[1] - mapdata.info.origin.position.y)/mapdata.info.resolution) *
                     mapdata.info.width) + (math.floor((point[0] - mapdata.info.origin.position.x)/mapdata.info.resolution)))
        r_region = int(self.info_radius/mapdata.info.resolution)
        init_index = index-r_region*(mapdata.info.width + 1)
        for n in range(0, 2 * r_region + 1):
            start = n * mapdata.info.width + init_index
            end = start + 2 * r_region
            limit = ((start/mapdata.info.width) + 2) * mapdata.info.width
            for i in range(start, end + 1):
                if (i >= 0 and i < limit and i < len(mapdata.data)):
                    for j in range(0, len(centroids)):
                        current_pt = centroids[j]
                        if(mapdata.data[i] == -1 and np.linalg.norm(np.array([mapdata.info.origin.position.x +
                                                                              (i - (i//mapdata.info.width) * (mapdata.info.width)) * mapdata.info.resolution, mapdata.info.origin.position.y +
                                                                              (i//mapdata.info.width) * mapdata.info.resolution])-np.array([current_pt[0], current_pt[1]])) <= self.info_radius and np.linalg.norm(np.array([mapdata.info.origin.position.x +
                                                                                                                                                                                               (i - (i//mapdata.info.width) * (mapdata.info.width)) * mapdata.info.resolution, mapdata.info.origin.position.y +
                                                                                                                                                                                               (i//mapdata.info.width) * mapdata.info.resolution])-point) <= self.info_radius):
                            info_gain[j] -= (mapdata.info.resolution *
                                             mapdata.info.resolution)
        return info_gain

    def run(self):
        # exit the node to free up resources when we are done mapping
        # if not self.loc_or_map_mode:
            rospy.spin()


if __name__ == '__main__':
    MapFrontierAssigner().run()
