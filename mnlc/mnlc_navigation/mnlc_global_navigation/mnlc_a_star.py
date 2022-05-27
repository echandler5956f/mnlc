#!/usr/bin/env python

from nav_msgs.msg import OccupancyGrid, Path, GridCells
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.srv import GetPlan, GetMap
from queues import PriorityQueueDep
from scipy.spatial import distance
import move_base_msgs.msg as mb
import std_srvs.srv
import numpy as np
import actionlib
import roslib
import rospy
import math

roslib.load_manifest('rbe3002')


class mnlc_a_star():

    def __init__(self):
        self.error = False
        rospy.loginfo("Initializing mnlc_a_star.")
        rospy.init_node("mnlc_a_star")
        self.initialize_params()
        rospy.sleep(self.start_time)
        self.safe_start()
        # give gazebo a chance to warm up so rtabmap doesnt raise an error about not having a map
        rospy.loginfo("mnlc_a_star node ready.")

    def initialize_params(self):
        self.start_time = rospy.get_param('/a_star/start_time')
        # grid cost to be considered an obstacle
        self.obstacle_cost = rospy.get_param('/a_star/obstacle_cost')
        # [s] standard service timeout limit
        self.timeout = rospy.get_param('/controller/timeout')
        self.local_costmap = OccupancyGrid()
        self.global_costmap = OccupancyGrid()
        self.update_global_costmap_sub = rospy.Subscriber(
            '/mnlc_global_costmap_opencv/cspace', OccupancyGrid, self.update_global_costmap, queue_size=1)
        self.frontier_pub = rospy.Publisher(
            '/a_star/frontier', GridCells, queue_size=1)

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
        rospy.Service('/plan_path', GetPlan, self.plan_path)

    def plan_path(self, msg):
        rospy.loginfo("A* is planning the path.")
        res = self.global_costmap.info.resolution
        ox = self.global_costmap.info.origin.position.x
        oy = self.global_costmap.info.origin.position.y
        x1 = int(math.floor((msg.start.pose.position.x - ox) / res))
        y1 = int(math.floor((msg.start.pose.position.y - oy) / res))
        start = (x1, y1)
        x2 = int(math.floor((msg.goal.pose.position.x - ox) / res))
        y2 = int(math.floor((msg.goal.pose.position.y - oy) / res))
        goal = (x2, y2)
        path = self.a_star(start, goal)
        path_msg = Path()
        path_msg.header.frame_id = '/map'
        if path == -1:
            empty_pose = PoseStamped()
            empty_pose.header.frame_id = '/map'
            empty_pose.header.stamp = rospy.Time.now()
            path_msg.poses = [empty_pose]
            path_msg.header.stamp = rospy.Time.now()
            rospy.loginfo(
                "A* has not found a path within the alotted time. Send a new goal point.")
            return path_msg
        for point in path:
            pose = PoseStamped()
            point_ret = Point()
            point_ret.x = (point[0] * res) + ox + (res/2)
            point_ret.y = (point[1] * res) + oy + (res/2)
            point_ret.z = 0
            pose.pose.position.x = point_ret.x
            pose.pose.position.y = point_ret.y
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = '/map'
            path_msg.poses.append(pose)
        path_msg.header.stamp = rospy.Time.now()
        rospy.loginfo("A* has computed the path.")
        return path_msg

    def a_star(self, start, goal):
        rospy.loginfo("Beginning A* algorithm.")
        time_init = rospy.get_time()
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)." %
                      (start[0], start[1], goal[0], goal[1]))
        frontier_nodes = GridCells()
        obstacle_cost = self.obstacle_cost
        frontier = PriorityQueueDep()
        frontier.put(start, 0)
        goal_np = np.array(goal)
        came_from = dict()
        cost_so_far = dict()
        came_from[start] = None
        cost_so_far[start] = 0
        data = self.global_costmap.data
        width = self.global_costmap.info.width
        height = self.global_costmap.info.height
        res = self.global_costmap.info.resolution
        ox = self.global_costmap.info.origin.position.x
        oy = self.global_costmap.info.origin.position.y
        frontier_nodes.cell_height = self.global_costmap.info.resolution
        frontier_nodes.cell_width = self.global_costmap.info.resolution
        start_time = rospy.get_time()
        while not frontier.empty():
            current = frontier.get()
            if current == goal:
                rospy.loginfo("Path has been found.")
                break
            time = rospy.get_time()
            if time >= start_time + 10.0:
                rospy.logwarn("A* has not found a path in less than 10.0 seconds. Requesting new path...")
                return -1
            x = current[0]
            y = current[1]
            cell_neighbors = [(x + 1, y + 1), (x + 1, y), (x + 1, y - 1), (x, y + 1),
                              (x, y - 1), (x - 1, y + 1), (x - 1, y), (x - 1, y - 1)]
            valid_neighbors = [cell for cell in cell_neighbors if ((cell[0] > 0) and (cell[0] < width) and (
                cell[1] > 0) and (cell[1] < height) and (data[cell[0] + (cell[1] * width)] < obstacle_cost))]
            for next in valid_neighbors:
                c = data[next[0] + (next[1] * width)]
                if c == -1:
                    c = 25
                new_cost = cost_so_far[current] + c
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    next_np = np.array(next)
                    priority = new_cost + 4.0 * distance.euclidean(goal_np, next_np)
                    frontier.put(next, priority)
                    came_from[next] = current
                    point = Point()
                    point.x = (x * res) + ox + res/2
                    point.y = (y * res) + oy + res/2
                    frontier_nodes.cells.append(point)
        frontier_nodes.cells.reverse()
        frontier_nodes.header.frame_id = "map"
        frontier_nodes.header.stamp = rospy.Time.now()
        self.frontier_pub.publish(frontier_nodes)
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

    def safe_start_phase_2(self):
        rospy.wait_for_service(
            '/round_up_nodes', timeout=rospy.Duration(self.timeout))
        try:
            tmp = rospy.ServiceProxy('/round_up_nodes', std_srvs.srv.Empty)
        except rospy.ServiceException as e:
            rospy.logerr("Round up nodes service call failed: %s" % e)
            self.error_handler()
            return
        rospy.loginfo("Round up nodes service call successful.")
        temp = tmp()
        rospy.wait_for_service(
            '/begin_phase2', timeout=rospy.Duration(self.timeout))
        try:
            tmp2 = rospy.ServiceProxy('/begin_phase2', std_srvs.srv.Empty)
        except rospy.ServiceException as e:
            rospy.logerr("Begin phase2 service call failed: %s" % e)
            self.error_handler()
            return
        rospy.loginfo("Begin phase2 service call successful.")
        temp2 = tmp2()
        self.phase2_client = actionlib.SimpleActionClient(
            '/mnlc/navigate_to_origin', mb.MoveBaseAction)

    def final(self):
        self.phase3_client = actionlib.SimpleActionClient(
            '/mnlc/navigation', mb.MoveBaseAction)
        self.phase3_client.wait_for_server(
            timeout=rospy.Duration(self.timeout))

    def update_global_costmap(self, map):
        if map.info.resolution != 0.0:
            self.global_costmap = map
        else:
            rospy.logwarn("Receiving a global costmap with 0 resolution!")

    def update_local_costmap(self, map):
        if map.info.resolution != 0.0:
            self.local_costmap = map
        else:
            rospy.logwarn("Receiving a local costmap with 0 resolution!")

    def error_handler(self):
        self.error = True

    def run(self):
        while not rospy.is_shutdown() and self.error == False:
            rospy.spin()


if __name__ == '__main__':
    try:
        mnlc_a_star().run()
    except rospy.ROSInterruptException:
        pass
