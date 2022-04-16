#!/usr/bin/env python

from nav_msgs.msg import GridCells, OccupancyGrid, Path
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, PoseStamped
from priority_queue import PriorityQueue
from nav_msgs.srv import GetPlan, GetMap
# from collections import OrderedDict
from scipy.spatial import distance
import numpy as np
import rospy
import time
import math
import cv2
# import sys
import tf

class PathPlanner:

    def __init__(self):
        """
        Class constructor
        """
        # REQUIRED CREDIT
        # Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        # Create a new service called "plan_path" that accepts messages of
        # type GetPlan and calls self.plan_path() when a message is received
        p = rospy.Service('plan_path', GetPlan, self.plan_path)
        # Create a publisher for the C-space (the enlarged occupancy grid)
        # The topic is "/path_planner/cspace", the message type is OccupencyGrid
        self.c_space_pub = rospy.Publisher(
            '/path_planner/cspace', OccupancyGrid, queue_size=1)
        # Create publishers for A* (expanded cells, frontier, ...)
        # Choose the topic names, the message type is GridCells
        self.expanded_cells_pub = rospy.Publisher(
            '/path_planner/expanded_cells', GridCells, queue_size=1)
        self.frontier_pub = rospy.Publisher(
            '/path_planner/frontier', GridCells, queue_size=1)
        self.a_star_pub = rospy.Publisher('/path_planner/a_star', Path, queue_size=1)
        self.odom_listener = tf.TransformListener()
        self.firstMap = True
        self.pointMatrix = None
        # Initialize the request counter
        # TODO
        # Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")

    @staticmethod
    def grid_to_index(mapdata, x, y):
        timeInit = rospy.get_time()
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        # REQUIRED CREDIT
        index = x + (y * mapdata.info.width)
        # print("Calculating grid_to_index took: ", rospy.get_time() - timeInit)
        return index
        
    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        timeInit = rospy.get_time()
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        # REQUIRED CREDIT
        dist = (math.sqrt(pow(x2-x1, 2)+pow(y2-y1, 2)))
        # print("Calculating euclidean_distance took: ", rospy.get_time() - timeInit)
        return dist

    @staticmethod
    def arc_length(x1, y1, x2, y2, ctheta):
        timeInit = rospy.get_time()
        theta = math.atan2(y2 - y1, x2 - x1) - ctheta
        arcLength = 1.61803399 * theta * (PathPlanner.euclidean_distance(x1, y1, x2, y2) / 2)
        # print("Calculating arc_length took: ", rospy.get_time() - timeInit)
        return arcLength

    @staticmethod
    def grid_to_world(mapdata, x, y):
        timeInit = rospy.get_time()
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        # REQUIRED CREDIT
        point = Point()
        # points are in the center of the grid square
        point.x = (x * mapdata.info.resolution) + \
            mapdata.info.origin.position.x + (mapdata.info.resolution/2)
        point.y = (y * mapdata.info.resolution) + \
            mapdata.info.origin.position.y + (mapdata.info.resolution/2)
        point.z = 0
        # print("Calculating grid_to_world took: ", rospy.get_time() - timeInit)
        return point

    @staticmethod
    def world_to_grid(mapdata, wp):
        timeInit = rospy.get_time()
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        # REQUIRED CREDIT
        x = int(math.floor((wp.x - mapdata.info.origin.position.x) /
                mapdata.info.resolution))
        y = int(math.floor((wp.y - mapdata.info.origin.position.y) /
                mapdata.info.resolution))
        # print("Calculating world_to_grid took: ", rospy.get_time() - timeInit)
        return (x, y)

    @staticmethod
    def path_to_poses(mapdata, path):
        timeInit = rospy.get_time()
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        # REQUIRED CREDIT
        pathOfPoses = []
        for cell in path:
            pose = PoseStamped()
            (pose.pose.position.x,  pose.pose.position.y) = PathPlanner.grid_to_world(cell)
            pose.header.stamp = rospy.Time.now()
            pathOfPoses.append(pose)
            # orientation at each point will be overriden by the local planner, so it is not dealt with here
        pathOfPoses.reverse()
        # print("Calculating path_to_poses took: ", rospy.get_time() - timeInit)
        return pathOfPoses

    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        timeInit = rospy.get_time()
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        # REQUIRED CREDIT
        if x > 0 and x < mapdata.info.width and y > 0 and y < mapdata.info.height:
            cell_index = PathPlanner.grid_to_index(mapdata, x, y)
            # print(cell_index)
            cell_probability = mapdata.data[cell_index]
            if cell_probability != -1 and cell_probability < 90:
            # if cell_probability < 90:
                # print("Calculating blank took: ", rospy.get_time() - timeInit)
                return True
        # print("Calculating is_cell_walkable took: ", rospy.get_time() - timeInit)
        return False

    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        timeInit = rospy.get_time()
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        # REQUIRED CREDIT
        cell_neighbors = []
        neighbours = [(x + 1, y + 1), (x + 1, y), (x + 1, y - 1), (x, y + 1),
                      (x, y - 1), (x - 1, y + 1), (x - 1, y), (x - 1, y - 1)]
        for cell in neighbours:
            # print("Im a cell!", cell)
            if(PathPlanner.is_cell_walkable(mapdata, cell[0], cell[1])):
                cell_neighbors.append(cell)
            # print("I am walkable!", PathPlanner.is_cell_walkable(mapdata, cell[0], cell[1]))
        # print("Calculating neighbors_of_8 took: ", rospy.get_time() - timeInit)
        return cell_neighbors
    
    def neighbors_of_6(self, mapdata, currentCell):
        # all cells except for sideways
        timeInit = rospy.get_time()
        point, theta = self.whichCellAmIFacing(mapdata, currentCell)
        cell_neighbors = []
        neighbours = [(int(currentCell[0] + mapdata.info.resolution * math.cos(theta - math.pi/4)), int(currentCell[1] + mapdata.info.resolution * math.sin(theta - math.pi/4))), 
                      (int(currentCell[0] + mapdata.info.resolution * math.cos(theta)), int(currentCell[1] + mapdata.info.resolution * math.sin(theta))), 
                      (int(currentCell[0] + mapdata.info.resolution * math.cos(theta + math.pi/4)), int(currentCell[1] + mapdata.info.resolution * math.sin(theta + math.pi/4))),

                      (int(currentCell[0] + mapdata.info.resolution * math.cos(-theta - math.pi/4)), int(currentCell[1] + mapdata.info.resolution * math.sin(-theta - math.pi/4))), 
                      (int(currentCell[0] + mapdata.info.resolution * math.cos(-theta)), int(currentCell[1] + mapdata.info.resolution * math.sin(-theta))), 
                      (int(currentCell[0] + mapdata.info.resolution * math.cos(-theta + math.pi/4)), int(currentCell[1] + mapdata.info.resolution * math.sin(-theta + math.pi/4)))]
        for cell in neighbours:
            if(PathPlanner.is_cell_walkable(mapdata, cell[0], cell[1])):
                cell_neighbors.append(cell)
        # print("Calculating neighbors_of_6 took: ", rospy.get_time() - timeInit)
        return cell_neighbors
    
    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        timeInit = rospy.get_time()
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        # REQUIRED CREDIT
        cell_neighbors = []
        neighbours = [(x + 1, y), (x, y + 1), (x, y - 1), (x - 1, y)]
        for cell in neighbours:
            if(PathPlanner.is_cell_walkable(mapdata, cell[0], cell[1])):
                cell_neighbors.append(cell)
        # print("Calculating neighbors_of_4 took: ", rospy.get_time() - timeInit)
        return cell_neighbors

    def neighbors_of_3(self, mapdata, currentCell):
        # the three forward most cells relative to the current heading
        timeInit = rospy.get_time()
        point, theta = self.whichCellAmIFacing(mapdata, currentCell)
        cell_neighbors = []
        neighbours = [(int(currentCell[0] + mapdata.info.resolution * math.cos(theta - math.pi/4)), int(currentCell[1] + mapdata.info.resolution * math.sin(theta - math.pi/4))), 
                      (int(currentCell[0] + mapdata.info.resolution * math.cos(theta)), int(currentCell[1] + mapdata.info.resolution * math.sin(theta))), 
                      (int(currentCell[0] + mapdata.info.resolution * math.cos(theta + math.pi/4)), int(currentCell[1] + mapdata.info.resolution * math.sin(theta + math.pi/4)))]
        for cell in neighbours:
            if(PathPlanner.is_cell_walkable(mapdata, cell[0], cell[1])):
                cell_neighbors.append(cell)
        # print("Calculating neighbors_of_3 took: ", rospy.get_time() - timeInit)
        return cell_neighbors

    @staticmethod
    def request_map():
        timeInit = rospy.get_time()
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        # REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        m = rospy.ServiceProxy('/static_map', GetMap)
        map = m()
        # print(map.map.info)
        # print("Calculating request_map took: ", rospy.get_time() - timeInit)
        return map.map

    def whichCellAmIFacing(self, mapdata, currentCell):
        timeInit = rospy.get_time()
        (tr, rot) = self.odom_listener.lookupTransform(
                "odom", "base_footprint", rospy.Time(0))
        roll, pitch, ctheta = euler_from_quaternion(rot)
        posx = int(currentCell[0] + mapdata.info.resolution * math.cos(int((math.pi/4) * (ctheta / (math.pi/4)))))
        posy = int(currentCell[1] + mapdata.info.resolution * math.sin(int((math.pi/4) * (ctheta / (math.pi/4)))))
        point = Point()
        point.x = posx
        point.y = posy
        # print(ctheta)
        # print("Calculating whichCellAmIFacing took: ", rospy.get_time() - timeInit)
        return self.world_to_grid(mapdata, point), ctheta

    def angleToGoalCell(self, mapdata, startCell, goalCell):
        timeInit = rospy.get_time()
        point, ctheta = self.whichCellAmIFacing(mapdata, startCell)
         # print("Calculating angleToGoalCell took: ", rospy.get_time() - timeInit)
        return math.atan2(goalCell[1] - point[1], goalCell[0] - point[0]) - ctheta

    def filter_map(self, mapdata, padding):
        timeInit = rospy.get_time()
        cspace = mapdata
        path = r'/home/quant/rbe3002_ws/src/rbe3002/maps/simple_map.pgm'
        img = cv2.imread(path, -1)
        kernel1 = np.ones((padding,padding), np.float)
        img_erosion = cv2.erode(img, kernel = kernel1, iterations = 1)
        # ksize and sigma are constants that require manual tuning on a map-by-map basis
        gaussian_blur = cv2.GaussianBlur(src=img_erosion, ksize=(3,3),sigmaX=0, sigmaY=0)
        flipped = np.flipud(gaussian_blur)
        inverted = cv2.bitwise_not(flipped)
        norm_image = cv2.normalize(inverted, None, alpha=0, beta=100, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        dictOfPoints = self.pointMatrix
        dataFromGridC = norm_image.flatten('C')
        dataFromGridF = norm_image.flatten('F')
        dataC = tuple(np.array(dataFromGridC, dtype = int)) # for visualizing the cspace. not necessary and for maximum performance keep it off
        dataF = tuple(np.array(dataFromGridF, dtype = int))
        cspaceDict = {k: v for k, v in zip(sorted(dictOfPoints.keys()), dataF)}
        cspace.header.stamp = rospy.Time.now()
        cspace.header.frame_id = 'map'
        cspace.data = dataC
        # print(cspace.data)
        # print(sorted(cspaceDict.keys()))
        # for key in cspaceDict.keys():
        #     cell_index = PathPlanner.grid_to_index(mapdata, key[0], key[1])
        #     cell_probability = cspace.data[cell_index]
        #     print("Map       : ", cell_probability)
        #     print("Dictionary: ", cspaceDict[key])
        self.c_space_pub.publish(cspace)
        print("Calculating filter_map took: ", rospy.get_time() - timeInit)
        return cspaceDict

    def distance_to_nearest_obstacle(self, obstacleList, x, y):
            timeInit = rospy.get_time()
            dx = [x - p[0] for p in obstacleList]
            dy = [y - p[1] for p in obstacleList]
            dist = np.hypot(dx, dy)
            index = np.argmin(dist)
            # print("Calculating distance_to_nearest_obstacle took: ", rospy.get_time() - timeInit)
            return dist[index]

    def a_star(self, mapdata, cspaceDict, start, goal):
        timeInit = rospy.get_time()
        # REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" %
                      (start[0], start[1], goal[0], goal[1]))
        frontier = PriorityQueue()
        frontierGrid = GridCells()
        frontierGrid.cell_height = mapdata.info.resolution
        frontierGrid.cell_width = mapdata.info.resolution
        frontier.put(start, 0)
        goal_np = np.array(goal)
        came_from = dict()
        cost_so_far = dict()
        # point = Point()
        # point.z = 0.0
        came_from[start] = None
        cost_so_far[start] = 0
        # conX = mapdata.info.origin.position.x + (mapdata.info.resolution/2)
        # conY = mapdata.info.origin.position.y + (mapdata.info.resolution/2)
        while not frontier.empty():
            current = frontier.get()
            if current == goal:
                print("Goal Found!")
                break
            x = current[0]
            y = current[1]
            cell_neighbors = [(x + 1, y + 1), (x + 1, y), (x + 1, y - 1), (x, y + 1),
                              (x, y - 1), (x - 1, y + 1), (x - 1, y), (x - 1, y - 1)]
            # just a simple list composition to speed things up
            valid_neighbors = [cell for cell in cell_neighbors if ((cell[0] > 0) and (cell[0] < mapdata.info.width) and (cell[1] > 0) and (cell[1] < mapdata.info.height) and (cspaceDict[cell] < 90))]
            # orthogonals = [cell for index, cell in enumerate(cell_neighbors) if ((index % 2 == 0) and (cell[0] > 0) and (cell[0] < mapdata.info.width) and (cell[1] > 0) and (cell[1] < mapdata.info.height) and (cspaceDict[cell] < 90))]
            # print("Neighbors: ", neighbors)
            for next in valid_neighbors:
                # # rospy.sleep(0.0125)
                # turningCost = 0.0
                # if next not in orthogonals:
                #     turningCost = 1
                # kinodynamicCost = 0.0
                # if next not in self.neighbors_of_6(mapdata, current):
                #     kinodynamicCost = 3
                # elif next not in self.neighbors_of_3(mapdata, current):
                #         kinodynamicCost = 2
                new_cost = cost_so_far[current] + cspaceDict[(next[0], next[1])]# + turningCost + kinodynamicCost
                # print("New cost: ", new_cost)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    next_np = np.array(next)
                    priority = new_cost + distance.euclidean(goal_np, next_np)
                    # unused, ctheta = self.whichCellAmIFacing(mapdata, current)
                    # priority = new_cost + PathPlanner.arc_length(goal[0], goal[1], next[0], next[1], ctheta)
                    frontier.put(next, priority)
                    # point.x = (x * mapdata.info.resolution) + conX
                    # point.y = (y * mapdata.info.resolution) + conY
                    frontierGrid.cells.append(self.grid_to_world(mapdata, current[0], current[1]))
                    came_from[next] = current
        frontierGrid.cells.reverse()
        frontierGrid.header.frame_id = "map"
        frontierGrid.header.stamp = rospy.Time.now()
        self.frontier_pub.publish(frontierGrid)
        path = self.reconstruct_path(came_from, start, goal)
        print("Calculating a_star took: ", rospy.get_time() - timeInit)
        return path

    @staticmethod
    def optimize_path(path):
        timeInit = rospy.get_time()
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        # EXTRA CREDIT
        rospy.loginfo("Optimizing path")
        # print("Calculating optimize_path took: ", rospy.get_time() - timeInit)
        return path

    def path_to_message(self, mapdata, path):
        timeInit = rospy.get_time()
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        # REQUIRED CREDIT
        rospy.loginfo("Returning a Path message")
        # print(path)
        pathMsg = Path()
        its = 0
        for point in path:
            # print(point)
            # print(its)
            pose = PoseStamped()
            pointRet = PathPlanner.grid_to_world(mapdata, point[0], point[1])
            pose.pose.position.x = pointRet.x
            pose.pose.position.y = pointRet.y
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pathMsg.poses.append(pose)
            its = its + 1
        pathMsg.header.frame_id = "map"
        self.a_star_pub.publish(pathMsg)
        # print("Calculating path_to_message took: ", rospy.get_time() - timeInit)
        return pathMsg

    def reconstruct_path(self, came_from, start, goal):
        timeInit = rospy.get_time()
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        # print("Calculating reconstruct_path took: ", rospy.get_time() - timeInit)
        return path

    def plan_path(self, msg):
        timeInit = rospy.get_time()
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        # Request the map
        # In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        # print(mapdata)
        if self.firstMap: # only form the array of points once (this means the map has a fixed size)
            begin = time.time()
            N = mapdata.info.width
            M = mapdata.info.height
            self.pointMatrix = {(x,y):0 for x in range(N) for y in range(M)}
            # initializes a point:probability dictionary for very fast lookups
            # this also implies that the map size remains static
            self.firstMap = False
            end = time.time()
            # print(self.pointMatrix)
            print("Generating points from map size took: ", end - begin)
        # Calculate the C-space and publish it
        # cspacedata = self.calc_cspace(mapdata, 2)
        cspaceDict = self.filter_map(mapdata, 3)
        # print(cspaceDict)
        # print("I made it!")
        # self.c_space_pub.publish(cspace)
        # Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        # path = self.a_star(cspace, start, goal) # takes in the cspace as an occupancy grid
        path = self.a_star(mapdata, cspaceDict, start, goal) # takes in the cspace as a dictionary
        # Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        # Return a Path message
        # print("Calculating plan_path took: ", rospy.get_time() - timeInit)
        return self.path_to_message(mapdata, waypoints)

    def to_tuple(self, lst):
        return tuple(self.to_tuple(i) if isinstance(i, list) else i for i in lst)

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


if __name__ == '__main__':
    PathPlanner().run()
