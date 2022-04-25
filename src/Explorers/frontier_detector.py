#!/usr/bin/env python

from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import OccupancyGrid, GridCells
from visualization_msgs.msg import Marker
from rbe3002.msg import PointArray
import std_msgs.msg
import numpy as np
import rospy
import math
import copy
import cv2


class FrontierDetector(object):

    def __init__(self):
        rospy.init_node("FrontierDetector")
        rospy.loginfo("Initializing Frontier Detecting node.")
        self.sim = rospy.get_param('sim')
        self.padding = rospy.get_param('padding')
        self.new_map_written_sub = rospy.Subscriber(
            '/MNCLGlobalCostmap/new_map_written', std_msgs.msg.String, self.update_map)
        self.c_space_sub = rospy.Subscriber(
            '/MNCLGlobalCostmap/cspace', OccupancyGrid, self.update_cspace_map, queue_size=1)
        self.rtab_map_sub = rospy.Subscriber(
            '/MNCLGlobalCostmap/map', OccupancyGrid, self.update_rtabmap, queue_size=1)
        self.current_cell = rospy.Subscriber(
            'MNCLGlobalController/current_cell', Point, self.update_visited, queue_size=1)
        self.visited_pub = rospy.Publisher(
            'FrontierDetector/visited', GridCells, queue_size=1)
        self.loc_or_map_mode_sub = rospy.Subscriber(
            '/MNCLGlobalController/loc_or_map_mode', std_msgs.msg.Bool, self.update_loc_or_map, queue_size=1)
        # Start by mapping until frontier search algorithm visits a certain percentage of cells
        self.loc_or_map_mode = False
        self.visited = []
        self.latest_map = None
        self.first_map = True
        self.new_commands = False
        self.old_commands = self.new_commands
        self.map = OccupancyGrid()
        self.rtabmap = OccupancyGrid()
        self.points = Marker()
        rospy.sleep(1)
        rospy.loginfo("Frontier Detecting node ready.")

    def commands(self, flag):
        self.new_commands = not self.new_commands

    def compare_commands(self):
        # if self.new_commands != self.old_commands:
        #     self.old_commands= self.new_commands
        #     return False
        return True

    def update_loc_or_map(self, loc_or_map):
        self.loc_or_map_mode = loc_or_map

    def update_map(self, tmp):
        # if not self.loc_or_map_mode:
            self.latest_map = cv2.imread(r'/home/quant/.ros/rtabmap.pgm', -1)

    def update_rtabmap(self, map):
        # if not self.loc_or_map_mode:
            self.rtabmap = map

    def update_cspace_map(self, map):
        # if not self.loc_or_map_mode:
            self.map = map
            if self.first_map:
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
                self.first_map = False

    def update_visited(self, visited):
        if not self.loc_or_map_mode:
            x = visited.x
            y = visited.y
            visited_width = 20
            visited_height = 20
            # visited_neighbors = ((x-6, y+6), (x-5, y+6), (x-4, y+6), (x-3, y+6), (x-2, y+6), (x-1, y+6), (x+0, y+6), (x+1, y+6), (x+1, y+6), (x+3, y+6), (x+4, y+6), (x+5, y+6), (x+6, y+6),
            #                      (x-6, y+5), (x-5, y+5), (x-4, y+5), (x-3, y+5), (x-2, y+5), (x-1, y+5), (x+0, y+5), (x+1, y+5), (x+1, y+5), (x+3, y+5), (x+4, y+5), (x+5, y+5), (x+6, y+5),
            #                      (x-6, y+4), (x-5, y+4), (x-4, y+4), (x-3, y+4), (x-2, y+4), (x-1, y+4), (x+0, y+4), (x+1, y+4), (x+1, y+4), (x+3, y+4), (x+4, y+4), (x+5, y+4), (x+6, y+4),
            #                      (x-6, y+3), (x-5, y+3), (x-4, y+3), (x-3, y+3), (x-2, y+3), (x-1, y+3), (x+0, y+3), (x+1, y+3), (x+1, y+3), (x+3, y+3), (x+4, y+3), (x+5, y+3), (x+6, y+3),
            #                      (x-6, y+2), (x-5, y+2), (x-4, y+2), (x-3, y+2), (x-2, y+2), (x-1, y+2), (x+0, y+2), (x+1, y+2), (x+1, y+2), (x+3, y+2), (x+4, y+2), (x+5, y+2), (x+6, y+2),
            #                      (x-6, y+1), (x-5, y+1), (x-4, y+1), (x-3, y+1), (x-2, y+1), (x-1, y+1), (x+0, y+1), (x+1, y+1), (x+1, y+1), (x+3, y+1), (x+4, y+1), (x+5, y+1), (x+6, y+1),
            #                      (x-6, y+0), (x-5, y+0), (x-4, y+0), (x-3, y+0), (x-2, y+0), (x-1, y+0), (x+0, y+0), (x+1, y+0), (x+1, y+0), (x+3, y+0), (x+4, y+0), (x+5, y+0), (x+6, y+0),
            #                      (x-6, y-1), (x-5, y-1), (x-4, y-1), (x-3, y-1), (x-2, y-1), (x-1, y-1), (x+0, y-1), (x+1, y-1), (x+1, y-1), (x+3, y-1), (x+4, y-1), (x+5, y-1), (x+6, y-1),
            #                      (x-6, y-2), (x-5, y-2), (x-4, y-2), (x-3, y-2), (x-2, y-2), (x-1, y-2), (x+0, y-2), (x+1, y-2), (x+1, y-2), (x+3, y-2), (x+4, y-2), (x+5, y-2), (x+6, y-2),
            #                      (x-6, y-3), (x-5, y-3), (x-4, y-3), (x-3, y-3), (x-2, y-3), (x-1, y-3), (x+0, y-3), (x+1, y-3), (x+1, y-3), (x+3, y-3), (x+4, y-3), (x+5, y-3), (x+6, y-3),
            #                      (x-6, y-4), (x-5, y-4), (x-4, y-4), (x-3, y-4), (x-2, y-4), (x-1, y-4), (x+0, y-4), (x+1, y-4), (x+1, y-4), (x+3, y-4), (x+4, y-4), (x+5, y-4), (x+6, y-4),
            #                      (x-6, y-5), (x-5, y-5), (x-4, y-5), (x-3, y-5), (x-2, y-5), (x-1, y-5), (x+0, y-5), (x+1, y-5), (x+1, y-5), (x+3, y-5), (x+4, y-5), (x+5, y-5), (x+6, y-5),
            #                      (x-6, y-6), (x-5, y-6), (x-4, y-6), (x-3, y-6), (x-2, y-6), (x-1, y-6), (x+0, y-6), (x+1, y-6), (x+1, y-6), (x+3, y-6), (x+4, y-6), (x+5, y-6), (x+6, y-6))
            for i in range(int(x) - int(math.floor(visited_width/2)), int(x) + int(math.floor(visited_width/2))):
                for j in range(int(y) - int(math.floor(visited_height/2)), int(y) + int(math.floor(visited_height/2))):
                    # point = Point()
                    # point.x = i
                    # point.y = j
                    point = (i, j)
                    if point not in self.visited:
                        # self.visited.cells.append(point)
                        self.visited.append(point)
            # self.visited_pub.publish(self.visited)


class OpenCVFrontierDetector(FrontierDetector):

    def __init__(self):
        super(OpenCVFrontierDetector, self).__init__()
        rospy.loginfo("Beginning OpenCVFrontierDetector initialization.")
        self.detected_points_pub = rospy.Publisher(
            '/OpenCVFrontierDetector/detected_points', PointStamped, queue_size=10)
        # self.potential_frontiers_pub = rospy.Publisher(
        #     '/OpenCVFrontierDetector/potential_frontiers', PointArray, queue_size=1)
        self.shapes_pub = rospy.Publisher(
            'OpenCVFrontierDetector/shapes', Marker, queue_size=1)
        self.new_commands_sub = rospy.Subscriber(
            "new_commands", std_msgs.msg.Bool, self.commands, queue_size=1)
        rospy.sleep(2)
        # while self.rtabmap.header.seq > 1 and len(self.rtabmap.data) > 1:
        #     pass
        rospy.loginfo("OpenCVFrontierDetector is ready.")
        self.detect_frontiers()

    def detect_frontiers(self):
        # if not self.loc_or_map_mode:
            # rate = rospy.Rate(5)
            exploration_goal = PointStamped()
            exploration_goal.header.frame_id = self.points.header.frame_id
            exploration_goal.point.z = 0
            while not rospy.is_shutdown(): # and not self.loc_or_map_mode:
                latest_map = self.rtabmap
                img_map = np.zeros((self.rtabmap.info.height, self.rtabmap.info.width, 1), np.uint8)
                for i in range(0,latest_map.info.height):
                    for j in range(0,latest_map.info.width):
                        if latest_map.data[i * latest_map.info.width + j] == 100:
                            img_map[i, j]=0
                        elif latest_map.data[i * latest_map.info.width + j] == 0:
                            img_map[i,j]=255
                        elif latest_map.data[i * latest_map.info.width + j] == -1:
                            img_map[i, j]=205
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
                        # # print(self.visited)
                        if p not in self.visited:
                            xr = cx * latest_map.info.resolution + latest_map.info.origin.position.x
                            yr = cy * latest_map.info.resolution + latest_map.info.origin.position.y
                            point = [np.array([xr, yr])]
                            if len(frontiers) > 0:
                                frontiers = np.vstack([frontiers, point])
                            else:
                                frontiers = point
                # lop = PointArray()
                for frontier in frontiers:
                    exploration_goal.header.stamp = rospy.Time(0)
                    exploration_goal.point.x = frontier[0]
                    exploration_goal.point.y = frontier[1]
                    # lop.points.append(exploration_goal)
                    self.detected_points_pub.publish(exploration_goal)
                    self.points.points = [exploration_goal.point]
                    self.shapes_pub.publish(self.points)
                # print(lop)
                # self.potential_frontiers_pub.publish(lop)
            # rate.sleep()

    def run(self):
        # exit the node to free up resources when we are done mapping
        if not self.loc_or_map_mode:
            rospy.spin()


if __name__ == '__main__':
    OpenCVFrontierDetector().run()
