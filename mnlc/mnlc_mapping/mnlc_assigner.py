#!/usr/bin/env python

from geometry_msgs.msg import PointStamped, PoseStamped, Point
from rbe3002.msg import PointArray, Pose2d
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import std_srvs.srv
import std_msgs.msg
import numpy as np
import roslib
import rospy
import math
import copy

roslib.load_manifest('rbe3002')


class mnlc_assigner():

    def __init__(self):
        self.state = 1
        self.error = False
        rospy.loginfo("Initializing mnlc_assigner.")
        rospy.init_node("mnlc_assigner")
        self.initialize_params()
        rospy.sleep(self.start_time)
        # give gazebo a chance to warm up so rtabmap doesnt raise an error about not having a map
        rospy.loginfo("mnlc_assigner node ready.")
        self.safe_start()

    def initialize_params(self):
        self.start_time = rospy.get_param('/frontier_assigner/start_time')
        # grid cost to be considered an obstacle
        self.obstacle_cost = rospy.get_param('/frontier_assigner/obstacle_cost')
        # [s] standard service timeout limit
        self.timeout = rospy.get_param('/controller/timeout')
        self.info_radius = rospy.get_param('/frontier_assigner/info_radius')
        self.info_multiplier = rospy.get_param(
            '/frontier_assigner/info_multiplier')
        self.hysteresis_radius = rospy.get_param(
            '/frontier_assigner/hysteresis_radius')
        self.hysteresis_gain = rospy.get_param(
            '/frontier_assigner/hysteresis_gain')
        self.filter_clear = rospy.get_param('/frontier_assigner/filter_clear')
        self.filter_limit = rospy.get_param('/frontier_assigner/filter_limit')
        self.marker = Marker()
        self.latest_map = OccupancyGrid()
        self.filtered_frontiers = []
        self.pose2d = Pose2d()
        self.points = Marker()
        self.visited = dict()
        self.next_time = rospy.get_time()
        self.filtered_frontiers_sub = rospy.Subscriber(
            '/frontier_filter/filtered_points', PointArray, self.update_filtered_frontiers, queue_size=10)
        self.filtered_frontiers_sub = rospy.Subscriber(
            '/opencv_points', PointStamped, self.update_opencv_points, queue_size=10)
        self.rtab_map_sub = rospy.Subscriber(
            '/mnlc_global_costmap_opencv/cspace', OccupancyGrid, self.update_map, queue_size=1)
        self.assigned_points_pub = rospy.Publisher(
            '/assigned_frontiers', Marker, queue_size=10)
        self.goal_publisher = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=1)
        self.get_current_cell = rospy.Subscriber(
            'mnlc_controller/current_cell', Point, self.update_visited, queue_size=1)
        self.state_machine_sub = rospy.Subscriber('/mnlc_state_machine', std_msgs.msg.Int8, self.update_state, queue_size=1)

    def initialize_marker(self, map):
        self.points.header.frame_id = map.header.frame_id
        self.points.header.stamp = rospy.Time.now()
        self.points.ns = "goal_markers"
        self.points.id = 6
        self.points.type = Marker.POINTS
        self.points.action = Marker.ADD
        self.points.pose.orientation.w = 1.0
        self.points.scale.x = self.points.scale.y = 0.3
        (self.points.color.r, self.points.color.g, self.points.color.b,
         self.points.color.a) = (0.0/255.0, 255.0/255.0, 136.0/255.0, 1)
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
        self.latest_map = map.map
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
        self.map_frontier_assigner()

    def map_frontier_assigner(self):
        exploration_goal = PointStamped()
        exploration_goal.header.frame_id = self.points.header.frame_id
        exploration_goal.point.z = 0
        while not rospy.is_shutdown():
            if self.state == 1:
                time_init = rospy.get_time()
                mapdata = self.latest_map
                centroids = copy.copy(self.filtered_frontiers)
                info_gain = []
                x = self.pose2d.cx
                y = self.pose2d.cy
                position = np.array([x, y])
                radius = self.info_radius
                for i in range(len(centroids)):
                    centroid = np.array([centroids[i][0], centroids[i][1]])
                    info_gain.append(self.informationGain(
                        mapdata, centroid, radius))
                info_gain = self.discount(
                    mapdata, [x, y], centroids, info_gain, radius)
                rev_rec = []
                centroid_rec = []
                for i in range(len(centroids)):
                    cost = np.linalg.norm(position - centroids[i])
                    information_gain = info_gain[i]
                    if np.linalg.norm(position - centroids[i]) <= self.hysteresis_radius:
                        information_gain *= self.hysteresis_gain
                    rev = information_gain * self.info_multiplier - cost
                    rev_rec.append(rev)
                    centroid_rec.append(centroids[i])
                if len(rev_rec) > 0:
                    best_frontier = centroid_rec[rev_rec.index(max(rev_rec))]
                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = '/map'
                    goal_pose.header.stamp = rospy.Time.now()
                    goal_pose.pose.position.x = best_frontier[0]
                    goal_pose.pose.position.y = best_frontier[1]
                    self.goal_publisher.publish(goal_pose)
                    exploration_goal.header.stamp = rospy.Time(0)
                    exploration_goal.point.x = goal_pose.pose.position.x
                    exploration_goal.point.y = goal_pose.pose.position.y
                    self.points.points = [exploration_goal.point]
                    self.assigned_points_pub.publish(self.points)
                # print("Calculating mnlc Assigner took: ", rospy.get_time() - time_init, ".")

    def informationGain(self, mapdata, point, radius):
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
                    p = np.array([mapdata.info.origin.position.x + (i - (i/mapdata.info.width) * (mapdata.info.width)) * mapdata.info.resolution,
                                  mapdata.info.origin.position.y + (i/mapdata.info.width) * mapdata.info.resolution])
                    if (mapdata.data[i] == -1 and np.linalg.norm(point-p) <= radius):
                        infoGain += 1.0
        return infoGain * (mapdata.info.resolution ** 2)

    def discount(self, mapdata, point, centroids, info_gain, r):
        visited = self.visited
        index = int((math.floor((point[1] - mapdata.info.origin.position.y)/mapdata.info.resolution) *
                     mapdata.info.width) + (math.floor((point[0] - mapdata.info.origin.position.x)/mapdata.info.resolution)))
        r_region = int(r/mapdata.info.resolution)
        init_index = index-r_region*(mapdata.info.width + 1)
        for n in range(0, 2 * r_region + 1):
            start = n * mapdata.info.width + init_index
            end = start + 2 * r_region
            limit = ((start/mapdata.info.width) + 2) * mapdata.info.width
            for i in range(start, end + 1):
                if (i >= 0 and i < limit and i < len(mapdata.data)):
                    for j in range(0, len(centroids)):
                        current_pt = centroids[j]
                        p = np.array([mapdata.info.origin.position.x + (i - (i/mapdata.info.width) * (mapdata.info.width)) * mapdata.info.resolution,
                                      mapdata.info.origin.position.y + (i/mapdata.info.width) * mapdata.info.resolution])
                        if (mapdata.data[i] == -1 and np.linalg.norm(p-current_pt) <= r and np.linalg.norm(p-point) <= r):
                            info_gain[j] -= mapdata.info.resolution ** 2
                        pp = (current_pt[0], current_pt[1])
                        x = int(math.floor((pp[0] - mapdata.info.origin.position.x) /
                            mapdata.info.resolution))
                        y = int(math.floor((pp[1] - mapdata.info.origin.position.y) /
                            mapdata.info.resolution))
                        ppw = (x, y)
                        if ppw in visited:
                            # print("in visited")
                            info_gain[j] -= 0.375
                        # if (mapdata.data[j] > self.obstacle_cost):
                        #     info_gain[j] -= 0.5
        return info_gain

    def update_filtered_frontiers(self, point_array):
        if rospy.get_time() > self.next_time:
            self.filtered_frontiers = []
            self.next_time = rospy.get_time() + self.filter_clear
        i = 0
        for point in point_array.points:
            if i > self.filter_limit:
                break
            self.filtered_frontiers.append(
                np.array([point.point.x, point.point.y]))
            i = i + 1

    def update_opencv_points(self, point):
        self.filtered_frontiers.append(
                np.array([point.point.x, point.point.y]))

    def update_visited(self, visited):
        x = visited.x
        y = visited.y
        visited_width = 20
        visited_height = 20
        xmin = int(x) - int(math.floor(visited_width / 2))
        xmax = int(x) + int(math.floor(visited_width / 2))
        ymin = int(y) - int(math.floor(visited_height / 2))
        ymax = int(y) + int(math.floor(visited_height / 2))
        visited = {(xs, ys): 0 for xs in range(xmin, xmax, 1)
                   for ys in range(ymin, ymax, 1)}
        self.visited.update(visited)

    def update_state(self, state):
        self.state = state.data

    def update_map(self, map):
        self.latest_map = map
        # print(self.latest_map.info)

    def error_handler(self):
        self.error = True

    def run(self):
        while not rospy.is_shutdown() and self.error == False:
            rospy.spin()


if __name__ == '__main__':
    try:
        mnlc_assigner().run()
    except rospy.ROSInterruptException:
        pass
