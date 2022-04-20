#!/usr/bin/env python

from nav_msgs.msg import OccupancyGrid, Path, GridCells
from priority_queue_vert import PriorityQueue, Priority
from geometry_msgs.msg import PoseStamped, Point
from priority_queue import PriorityQueueDep
from scipy.spatial import distance
from nav_msgs.srv import GetPlan
import std_msgs.msg
import numpy as np
import rospy
import math


OBSTACLE_COST = 90


class MNCLGlobalPlanner(object):
    # always generate a plan, regardless of whether we are mapping or navigating 
    # (we use the global plan to navigate to frontiers while mapping)
    def __init__(self):
        rospy.init_node("MNCLGlobalPlanner")
        rospy.loginfo("Initializing MNCL Global Planner.")
        self.loc_or_map_mode_sub = rospy.Subscriber(
            '/MNCLGlobalController/loc_or_map_mode', std_msgs.msg.Bool, self.update_loc_or_map, queue_size=3)
        self.loc_or_map_mode = False
        self.sim = rospy.get_param('sim')
        self.ground_truth_map = OccupancyGrid()
        self.frontier_nodes = GridCells()
        self.new_commands = False
        self.old_commands = self.new_commands
        rospy.sleep(1)
        rospy.loginfo("Global Planner node ready.")

    def update_mapdata(self, cspace):
        self.ground_truth_map = cspace

    def euclidean_norm(self, u, v):
        return distance.euclidean(u, v)

    def commands(self, flag):
        self.new_commands = not self.new_commands

    def compare_commands(self):
        # if self.new_commands != self.old_commands:
        #     self.old_commands= self.new_commands
        #     return False
        return True

    def update_loc_or_map(self, loc_or_map):
        self.loc_or_map_mode = loc_or_map


class a_star(MNCLGlobalPlanner):

    def __init__(self):
        super(a_star, self).__init__()
        rospy.loginfo("Initializing A* global planner.")
        self.gtm_sub = rospy.Subscriber(
            '/MNCLGlobalCostmap/cspace', OccupancyGrid,  self.update_mapdata)
        self.a_star_pub = rospy.Publisher('a_star/path', Path, queue_size=1)
        self.frontier_pub = rospy.Publisher(
            '/a_star/frontier', GridCells, queue_size=1)
        self.new_commands_sub = rospy.Subscriber(
            "new_commands", std_msgs.msg.Bool, self.commands, queue_size=3)
        rospy.Service('plan_path', GetPlan, self.plan_path)
        rospy.loginfo("A* ready.")

    def plan_path(self, msg):
        rospy.loginfo("A* is planning the path.")
        self.new_commands = self.old_commands
        res = self.ground_truth_map.info.resolution
        ox = self.ground_truth_map.info.origin.position.x
        oy = self.ground_truth_map.info.origin.position.y
        x1 = int(math.floor((msg.start.pose.position.x - ox) / res))
        y1 = int(math.floor((msg.start.pose.position.y - oy) / res))
        start = (x1, y1)
        x2 = int(math.floor((msg.goal.pose.position.x - ox) / res))
        y2 = int(math.floor((msg.goal.pose.position.y - oy) / res))
        goal = (x2, y2)
        path = self.a_star(start, goal)
        path_msg = Path()
        for point in path:
            pose = PoseStamped()
            point_ret = Point()
            point_ret.x = (point[0] * res) + ox + (res/2)
            point_ret.y = (point[1] * res) + oy + (res/2)
            point_ret.z = 0
            pose.pose.position.x = point_ret.x
            pose.pose.position.y = point_ret.y
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            path_msg.poses.append(pose)
        path_msg.header.frame_id = "map"
        self.a_star_pub.publish(path_msg)
        self.frontier_pub.publish(self.frontier_nodes)
        rospy.loginfo("A* has published the path.")
        return path_msg

    def a_star(self, start, goal):
        rospy.loginfo("Beginning A* algorithm.")
        time_init = rospy.get_time()
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)." %
                      (start[0], start[1], goal[0], goal[1]))
        frontier = PriorityQueueDep()
        frontier.put(start, 0)
        goal_np = np.array(goal)
        came_from = dict()
        cost_so_far = dict()
        came_from[start] = None
        cost_so_far[start] = 0
        data = self.ground_truth_map.data
        width = self.ground_truth_map.info.width
        height = self.ground_truth_map.info.height
        res = self.ground_truth_map.info.resolution
        ox = self.ground_truth_map.info.origin.position.x
        oy = self.ground_truth_map.info.origin.position.y
        self.frontier_nodes.cell_height = self.ground_truth_map.info.resolution
        self.frontier_nodes.cell_width = self.ground_truth_map.info.resolution
        while not frontier.empty() and self.compare_commands():
            current = frontier.get()
            if current == goal:
                rospy.loginfo("Path has been found.")
                break
            x = current[0]
            y = current[1]
            cell_neighbors = [(x + 1, y + 1), (x + 1, y), (x + 1, y - 1), (x, y + 1),
                              (x, y - 1), (x - 1, y + 1), (x - 1, y), (x - 1, y - 1)]
            valid_neighbors = [cell for cell in cell_neighbors if ((cell[0] > 0) and (cell[0] < width) and (
                cell[1] > 0) and (cell[1] < height) and (data[cell[0] + (cell[1] * width)] < OBSTACLE_COST))]
            for next in valid_neighbors:
                new_cost = cost_so_far[current] + \
                    data[next[0] + (next[1] * width)]
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    next_np = np.array(next)
                    priority = new_cost + 3 * \
                        distance.euclidean(goal_np, next_np)
                    frontier.put(next, priority)
                    came_from[next] = current
                    point = Point()
                    point.x = (x * res) + ox + res/2
                    point.y = (y * res) + oy + res/2
                    self.frontier_nodes.cells.append(point)
        self.frontier_nodes.cells.reverse()
        self.frontier_nodes.header.frame_id = "map"
        self.frontier_nodes.header.stamp = rospy.Time.now()
        path = self.reconstruct_path(came_from, start, goal)
        rospy.loginfo("A* has successfully found the optimal path.")
        print("Calculating A* took: ", rospy.get_time() - time_init, ".")
        return path

    def reconstruct_path(self, came_from, start, goal):
        rospy.loginfo("Reconstructing path.")
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        rospy.loginfo("Path reconstructed.")
        return path

    def run(self):
        rospy.spin()


class d_star(MNCLGlobalPlanner):

    def __init__(self):
        super(d_star, self).__init__()
        rospy.loginfo("Initializing D* global planner.")
        self.gtm_sub = rospy.Subscriber(
            '/MNCLGlobalCostmap/cspace', OccupancyGrid,  self.update_mapdata)
        self.d_star_pub = rospy.Publisher('d_star/path', Path, queue_size=1)
        self.frontier_pub = rospy.Publisher(
            '/d_star/frontier', GridCells, queue_size=1)
        self.new_commands_sub = rospy.Subscriber(
            "new_commands", bool, self.commands, queue_size=3)
        self.first_plan = True
        rospy.Service('plan_path', GetPlan, self.plan_path)
        rospy.loginfo("D* ready.")

    def plan_path(self, msg):
        rospy.loginfo("D* is planning the path.")
        if self.first_plan:
            self.neoc = None
            self.km = 0
            self.frontier_queue = PriorityQueue()
            # print(self.ground_truth_map)
            self.rhs = np.ones(
                (self.ground_truth_map.info.width, self.ground_truth_map.info.height)) * 100
            self.g = self.rhs.copy()
            self.first_plan = False
        x1 = int(math.floor((msg.start.pose.position.x - self.ground_truth_map.info.origin.position.x) /
                            self.ground_truth_map.info.resolution))
        y1 = int(math.floor((msg.start.pose.position.y - self.ground_truth_map.info.origin.position.y) /
                            self.ground_truth_map.info.resolution))
        start = (x1, y1)
        x2 = int(math.floor((msg.goal.pose.position.x - self.ground_truth_map.info.origin.position.x) /
                            self.ground_truth_map.info.resolution))
        y2 = int(math.floor((msg.goal.pose.position.y - self.ground_truth_map.info.origin.position.y) /
                            self.ground_truth_map.info.resolution))
        goal = (x2, y2)
        self.current = start
        self.goal = goal
        self.last = start
        self.rhs[self.goal] = 0
        self.frontier_queue.insert(self.goal, Priority(
            self.euclidean_norm(start, goal), 0))
        path = self.d_star(start)
        path_msg = Path()
        for point in path:
            pose = PoseStamped()
            point_ret = Point()
            point_ret.x = (point[0] * self.ground_truth_map.info.resolution) + \
                self.ground_truth_map.info.origin.position.x + \
                (self.ground_truth_map.info.resolution/2)
            point_ret.y = (point[1] * self.ground_truth_map.info.resolution) + \
                self.ground_truth_map.info.origin.position.y + \
                (self.ground_truth_map.info.resolution/2)
            point_ret.z = 0
            pose.pose.position.x = point_ret.x
            pose.pose.position.y = point_ret.y
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            path_msg.poses.append(pose)
        path_msg.header.frame_id = "map"
        self.d_star_pub.publish(path_msg)
        rospy.loginfo("D* has published the path.")
        return path_msg

    def calc_priority(self, vertex):
        # rospy.loginfo("D* is calculating vertex priority.")
        k1 = min(self.g[vertex], self.rhs[vertex]) + \
            self.euclidean_norm(self.current, vertex) + self.km
        k2 = min(self.g[vertex], self.rhs[vertex])
        # rospy.loginfo("D* has finished calculating vertex priority.")
        return Priority(k1, k2)

    def cost(self, u, v):
        # rospy.loginfo("D* is calculating graph edge cost.")
        x1 = u[0]
        y1 = u[1]
        x2 = v[0]
        y2 = v[1]
        if not (((x1 > 0) and (x1 < self.ground_truth_map.info.width) and (
            y1 > 0) and (y1 < self.ground_truth_map.info.height) and (self.ground_truth_map.data[x1 + (y1 * self.ground_truth_map.info.width)] < OBSTACLE_COST))
                or ((x2 > 0) and (x2 < self.ground_truth_map.info.width) and (
                    y2 > 0) and (y2 < self.ground_truth_map.info.height) and (self.ground_truth_map.data[x2 + (y2 * self.ground_truth_map.info.width)] < OBSTACLE_COST))):
            # rospy.loginfo("D* found an uncrossable edge.")
            return float('inf')
        else:
            # rospy.loginfo("D* has finished calculating the graph edge cost.")
            return self.euclidean_norm(u, v)

    def contain(self, u):
        # rospy.loginfo(
        #     "D* is checking if a vertex is already in the frontier queue heap.")
        return u in self.frontier_queue.vertices_in_heap

    def update_vertex(self, u):
        # rospy.loginfo("D* is updating a vertex.")
        if self.g[u] != self.rhs[u] and self.contain(u):
            self.frontier_queue.update(u, self.calc_priority(u))
        elif self.g[u] != self.rhs[u] and not self.contain(u):
            self.frontier_queue.insert(u, self.calc_priority(u))
        elif self.g[u] == self.rhs[u] and self.contain(u):
            self.frontier_queue.remove(u)
        # rospy.loginfo("D* has finished updating a vertex.")

    def compute_path(self):
        # rospy.loginfo("D* is computing the optimal path.")
        time_init = rospy.get_time()
        while self.frontier_queue.top_key() < self.calc_priority(self.current) or self.rhs[self.current] > self.g[self.current] and self.compare_commands():
            current = self.frontier_queue.top()
            ok = self.frontier_queue.top_key()
            nk = self.calc_priority(current)
            if ok < nk:
                self.frontier_queue.update(current, nk)
            elif self.g[current] > self.rhs[current]:
                self.g[current] = self.rhs[current]
                self.frontier_queue.remove(current)
                x = current[0]
                y = current[1]
                cell_neighbors = [(x + 1, y + 1), (x + 1, y), (x + 1, y - 1), (x, y + 1),
                                  (x, y - 1), (x - 1, y + 1), (x - 1, y), (x - 1, y - 1)]
                for cell in cell_neighbors:
                    if ((cell[0] > 0) and (cell[0] < self.ground_truth_map.info.width) and (
                            cell[1] > 0) and (cell[1] < self.ground_truth_map.info.height) and (self.ground_truth_map.data[cell[0] + (cell[1] * self.ground_truth_map.info.width)] < OBSTACLE_COST and cell != self.goal)):
                        self.rhs[cell] = min(self.rhs[cell], self.cost(
                            cell, current) + self.g[current])
                    self.update_vertex(cell)
            else:
                x = current[0]
                y = current[1]
                cell_neighbors = [(x + 1, y + 1), (x + 1, y), (x + 1, y - 1), (x, y + 1),
                                  (x, y - 1), (x - 1, y + 1), (x - 1, y), (x - 1, y - 1)]
                go = self.g[current]
                self.g[current] = float('inf')
                valid_neighbors = [cell for cell in cell_neighbors if ((cell[0] > 0) and (cell[0] < self.ground_truth_map.info.width) and
                                                                       (cell[1] > 0) and (cell[1] < self.ground_truth_map.info.height) and
                                                                       (self.ground_truth_map.data[cell[0] + (cell[1] * self.ground_truth_map.info.width)] < OBSTACLE_COST))]
                valid_neighbors.append(current)
                for cell in valid_neighbors:
                    if self.rhs[cell] == self.cost(cell, current) + go and cell != self.goal:
                        min_s = float('inf')
                        _x = cell[0]
                        _y = cell[1]
                        cell_neighbors_of_neighbors = [(_x + 1, _y + 1), (_x + 1, _y), (_x + 1, _y - 1), (_x, _y + 1),
                                                       (_x, _y - 1), (_x - 1, _y + 1), (_x - 1, _y), (_x - 1, _y - 1)]
                        neighbors_of_neighbors = [cell for cell in cell_neighbors_of_neighbors if ((cell[0] > 0) and (cell[0] < self.ground_truth_map.info.width) and
                                                                                                   (cell[1] > 0) and (cell[1] < self.ground_truth_map.info.height) and
                                                                                                   (self.ground_truth_map.data[cell[0] + (cell[1] * self.ground_truth_map.info.width)] < OBSTACLE_COST))]
                        for _cell in neighbors_of_neighbors:
                            temp = self.cost(cell, _cell) + self.g[_cell]
                            if min_s > temp:
                                min_s = temp
                        self.rhs[cell] = min_s
                    self.update_vertex(current)
        # print("Calculating D* took: ", rospy.get_time() - time_init, ".")
        # rospy.loginfo("D* has calculated the optimal path.")

    def rescan(self):
        # rospy.loginfo("D* is scanning for changed vertices.")
        neoc = self.neoc
        self.neoc = None
        # rospy.loginfo("D* has finished scanning vertices.")
        return neoc

    def d_star(self, start):
        rospy.loginfo("Executing D* from (%d,%d) to (%d,%d)." %
                      (start[0], start[1], self.goal[0], self.goal[1]))
        path = [start]
        self.current = start
        self.last = self.current
        time_init = rospy.get_time()
        self.compute_path()
        print("Calculating D* took: ", rospy.get_time() - time_init, ".")
        while self.current != self.goal and self.compare_commands():
            if self.rhs[self.current] != float('inf'):
                rospy.loginfo("No valid path.")
            x = self.current[0]
            y = self.current[1]
            cell_neighbors = [(x + 1, y + 1), (x + 1, y), (x + 1, y - 1), (x, y + 1),
                              (x, y - 1), (x - 1, y + 1), (x - 1, y), (x - 1, y - 1)]
            valid_neighbors = [cell for cell in cell_neighbors if ((cell[0] > 0) and (cell[0] < self.ground_truth_map.info.width) and
                                                                   (cell[1] > 0) and (cell[1] < self.ground_truth_map.info.height) and
                                                                   (self.ground_truth_map.data[cell[0] + (cell[1] * self.ground_truth_map.info.width)] < OBSTACLE_COST))]
            min_s = float('inf')
            arg_min = None
            for cell in valid_neighbors:
                temp = self.cost(self.current, cell) + self.g[cell]
                if temp < min_s:
                    min_s = temp
                    arg_min = cell
            self.current = arg_min
            path.append(self.current)
            ceoc = self.rescan()
            if ceoc:
                self.km += self.euclidean_norm(self.last, self.current)
                self.last = self.current
                vertices = ceoc.vertices
                for vertex in vertices:
                    v = vertex.pos
                    neighboring_vertices = v.edges_and_c_old
                    for u, co in neighboring_vertices.items():
                        cn = self.cost(u, v)
                        if (co > cn) and (u != self.goal):
                            self.rhs[u] = min(
                                self.rhs[u], self.cost(u, v) + self.g[v])
                        elif self.rhs[u] == co + self.g[v]:
                            if u != self.goal:
                                min_s = float('inf')
                                neighboring_neighbors = [cell for cell in cell_neighbors if ((cell[0] > 0) and (cell[0] < self.ground_truth_map.info.width) and
                                                                                             (cell[1] > 0) and (cell[1] < self.ground_truth_map.info.height) and
                                                                                             (self.ground_truth_map.data[cell[0] + (cell[1] * self.ground_truth_map.info.width)] < OBSTACLE_COST))]
                                for neighbor in neighboring_neighbors:
                                    temp = self.cost(u, cell) + self.g[cell]
                                    if min_s > temp:
                                        min_s = temp
                                self.rhs[u] = min_s
                            self.update_vertex(u)
            self.compute_path()
        print("Calculating D* took: ", rospy.get_time() - time_init, ".")
        rospy.loginfo("D* has completed its objective.")
        return path

    def run(self):
        rospy.spin()


class Vertex:
    def __init__(self, pos):
        self.pos = pos
        self.edges_and_costs = {}

    def add_edge_with_cost(self, succ, cost):
        if succ != self.pos:
            self.edges_and_costs[succ] = cost

    @property
    def edges_and_c_old(self):
        return self.edges_and_costs


class Vertices:
    def __init__(self):
        self.list = []

    def add_vertex(self, v):
        self.list.append(v)

    @property
    def vertices(self):
        return self.list


if __name__ == '__main__':
    a_star().run()
