#!/usr/bin/env python

from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import OccupancyGrid, GridCells
from visualization_msgs.msg import Marker
from rbe3002.msg import PointArray
from nav_msgs.srv import GetMap
import move_base_msgs.msg as mb
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


class mnlc_global_rrt_detector():

    def __init__(self):
        self.error = False
        rospy.loginfo("Initializing mnlc_global_rrt_detector.")
        rospy.init_node("mnlc_global_rrt_detector")
        self.initialize_params()
        rospy.sleep(self.timeout * 5)
        self.safe_start()
        rospy.sleep(self.timeout * 5)
        # give gazebo a chance to warm up so rtabmap doesnt raise an error about not having a map
        rospy.loginfo("mnlc_global_rrt_detector node ready.")
        self.detect_frontiers()

    def initialize_params(self):
        self.bounding_points = []
        self.bounding_point_sub = rospy.Subscriber(
            name='/bp', data_class=PointStamped, callback=self.set_bounding_points, queue_size=50)
        print('hello')
        # grid cost to be considered an obstacle
        self.obstacle_cost = rospy.get_param('obstacle_cost', 90)
        self.ctrl_invl = rospy.get_param(
            'ctrl_invl', 0.01)  # [s] control loop interval
        self.ctrl_rate = rospy.Rate(1/self.ctrl_invl)
        # [s] standard service timeout limit
        self.timeout = rospy.get_param('timeout', 1.0)
        self.eta = rospy.get_param('eta', 0.5)  # how greedy the search is
        self.latest_map = OccupancyGrid()

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
        self.rtab_map_sub = rospy.Subscriber(
            '/latest_map', OccupancyGrid, self.update_map, queue_size=1)
        self.detected_points_pub = rospy.Publisher(
            '/mnlc_global_rrt_detector/detected_points', PointStamped, queue_size=100)
        self.shapes_pub = rospy.Publisher(
            'mnlc_global_rrt_detector/shapes', Marker, queue_size=50)
        self.state_machine = rospy.Subscriber(
            'mnlc_state_machine', std_msgs.msg.Int8, self.update_state_machine, queue_size=1)

    def detect_frontiers(self):
        points = Marker()
        points.points.append(self.bounding_points[4].point)
        points.points.append(self.bounding_points[3].point)
        points.points.append(self.bounding_points[2].point)
        points.points.append(self.bounding_points[1].point)
        points.points.append(self.bounding_points[0].point)
        line = Marker()
        points.header.frame_id = self.map_metadata.header.frame_id
        line.header.frame_id = self.map_metadata.header.frame_id
        points.header.stamp = rospy.Time(0)
        line.header.stamp = rospy.Time(0)
        points.ns = line.ns = "markers"
        points.id = 0
        line.id = 1
        points.type = points.POINTS
        line.type = line.LINE_LIST
        points.action = points.ADD
        line.action = line.ADD
        points.pose.orientation.w = 1.0
        line.pose.orientation.w = 1.0
        line.scale.x = 0.03
        line.scale.y = 0.03
        points.scale.x = 0.3
        points.scale.y = 0.3
        line.color.r = 9.0/255.0
        line.color.g = 91.0/255.0
        line.color.b = 236.0/255.0
        points.color.r = 255.0/255.0
        points.color.g = 0.0/255.0
        points.color.b = 0.0/255.0
        points.color.a = 1.0
        line.color.a = 1.0
        points.lifetime = rospy.Duration()
        line.lifetime = rospy.Duration()
        temp1 = []
        temp1.append(points.points[0].x)
        temp1.append(points.points[0].y)
        temp2 = []
        temp2.append(points.points[2].x)
        temp2.append(points.points[0].y)
        ix = math.sqrt(pow(temp1[0] - temp2[0], 2) +
                       pow(temp1[1] - temp2[1], 2))
        temp1 = []
        temp2 = []
        temp1.append(points.points[0].x)
        temp1.append(points.points[0].y)
        temp2.append(points.points[0].x)
        temp2.append(points.points[2].y)
        iy = math.sqrt(pow(temp1[0] - temp2[0], 2) +
                       pow(temp1[1] - temp2[1], 2))
        temp1 = []
        temp2 = []
        self.sx = (points.points[0].x + points.points[2].x) / 2
        self.sy = (points.points[0].y + points.points[2].y) / 2
        tr = Point()
        tr = points.points[4]
        u = []
        cx = []
        cx.append(tr.x)
        cx.append(tr.y)
        u.append(cx)
        self.xn = []
        points.points = []
        self.shapes_pub.publish(points)
        eta = self.eta
        while 1:
            point = PointStamped()
            point.header.frame_id = '/map'
            point.header.stamp = rospy.Time.now()
            _rand = []
            xrt = (np.random.random() * ix) - (ix * 0.5) + self.sx
            yrt = (np.random.random() * iy) - (iy * 0.5) + self.sy
            _rand.append(xrt)
            _rand.append(yrt)
            x_nearest = self.find_nearest(u, _rand)
            # if math.sqrt(pow(x_nearest[1] - _rand[1], 2) + pow(x_nearest[0] - _rand[0], 2)) <= self.eta:
            #     self.xn = _rand
            # else:
            #     m = (_rand[1] - x_nearest[1]) / (_rand[0] - x_nearest[0])
            #     if _rand[0] - x_nearest[0] < 0.0:
            #         sign = -1.0
            #     else:
            #         sign = 1.0
            #     self.xn.append(
            #         sign * (math.sqrt(pow(self.eta, 2) / (pow(m, 2) + 1))) + x_nearest[0])
            #     self.xn.append(m * (self.xn[0] - x_nearest[0]) + x_nearest[1])
            #     if _rand[0] == x_nearest[0]:
            #         self.xn[0] = x_nearest[0]
            #         self.xn[1] = x_nearest[1] + self.eta
            self.xn = self.clear(x_nearest, _rand, eta)
            obs_free = self.check_obs(self.latest_map, x_nearest, self.xn)
            if obs_free == -1:
                point.point.x = self.xn[0]
                point.point.y = self.xn[1]
                point.point.z = 0.0
                points.points.append(point.point)
                self.shapes_pub.publish(points)
                self.detected_points_pub.publish(point)
                points.points = []
            elif obs_free == 1:
                u.append(self.xn)
                point.point.x = self.xn[0]
                point.point.y = self.xn[1]
                point.point.z = 0.0
                line.points.append(point.point)
                point.point.x = x_nearest[0]
                point.point.y = x_nearest[1]
                point.point.z = 0
                line.points.append(point.point)
            self.shapes_pub.publish(line)
            self.ctrl_rate.sleep()

    def find_nearest(self, u, x):
        min = math.sqrt(pow(u[0][1] - x[1], 2) + pow(u[0][0] - x[0], 2))
        min_index = 0
        for i in range(len(u)):
            temp = math.sqrt(pow(u[i][1] - x[1], 2) +
                             pow(u[i][0] - x[0], 2))
            if temp <= min:
                min = temp
                min_index = i
        return u[min_index]

    def clear(self, x_nearest, _rand, eta):
        x_new = []
        if math.sqrt(pow(x_nearest[1] - _rand[1], 2) + pow(x_nearest[0] - _rand[0], 2)) <= eta:
            x_new = _rand
        else:
            m = (_rand[1] - x_nearest[1]) / (_rand[0] - x_nearest[0])
            if _rand[0] - x_nearest[0] < 0.0:
                sign = -1.0
            else:
                sign = 1.0
            x_new.append(
            sign * (math.sqrt(pow(eta, 2) / (pow(m, 2) + 1)) + x_nearest[0]))
            x_new.append(m * (x_new[0] - x_nearest[0]) + x_nearest[1])
            if _rand[0] == x_nearest[0]:
                x_new[0] = x_nearest[0]
                x_new[1] = x_nearest[1] + eta
        return x_new

    def check_obs(self, latest_map, x_near, x_new):
        eta = self.map_metadata.info.resolution * 0.2
        steps = int(math.ceil(math.sqrt(pow(x_new[1] - x_near[1], 2) +
                                        pow(x_new[0] - x_near[0], 2)) / (eta)))
        xi = x_near
        obstacle = 0
        unkown = 0
        for i in range(steps):
            xi = self.clear(xi, x_new, eta)
            data = latest_map.data[int(math.floor(((xi[1]-self.sy)/latest_map.info.resolution) *
                                       latest_map.info.width)+(math.floor((xi[0]-self.sx)/latest_map.info.resolution)))]
            if data == 100:
                obstacle = 1
            if data == -1:
                unkown = 1
                break
        out = 0
        self.xn = xi
        if (unkown == 1):
          out = -1
        if (obstacle == 1):
          out = 0
        if (obstacle != 1 and unkown != 1):
          out = 1
        return out
        # return -1 if unkown == 1 else 0 if obstacle == 1 else 1 if obstacle != 1 and unkown != 1 else 0

    def update_state_machine(self, state):
        if state == 2:
            rospy.signal_shutdown(
                "Mapping complete. Node is now uneeded. Shutting down mnlc_global_rrt_detector.")
            sys.exit(0)

    def set_bounding_points(self, msg):
        self.bounding_points.append(msg)

    def update_map(self, map):
        self.latest_map = map

    def error_handler(self):
        self.error = True

    def run(self):
        while not rospy.is_shutdown() and self.error == False:
            rospy.spin()


if __name__ == '__main__':
    try:
        mnlc_global_rrt_detector().run()
    except rospy.ROSInterruptException:
        pass
