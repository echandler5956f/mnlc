#!/usr/bin/env python

from geometry_msgs.msg import PointStamped, Point
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
import sys
import tf

roslib.load_manifest('rbe3002')


class mnlc_local_rrt_detector():

    def __init__(self):
        self.state = 1
        self.error = False
        rospy.loginfo("Initializing mnlc_local_rrt_detector.")
        rospy.init_node("mnlc_local_rrt_detector", anonymous=True)
        self.initialize_params()
        rospy.sleep(self.controller_start_time)
        self.safe_start()
        rospy.sleep(self.start_time)
        # give gazebo a chance to warm up so rtabmap doesnt raise an error about not having a map
        rospy.loginfo("mnlc_local_rrt_detector node ready.")
        self.detect_frontiers()

    def initialize_params(self):
        self.controller_start_time = rospy.get_param('/controller/start_time')
        self.start_time = rospy.get_param('/local_rrt_detector/start_time')
        self.bounding_points = []
        self.bounding_point_sub = rospy.Subscriber(
            name='/bounding_points', data_class=PointStamped, callback=self.set_bounding_points, queue_size=5)
        # grid cost to be considered an obstacle
        self.obstacle_cost = rospy.get_param('/controller/obstacle_cost')
        # [s] standard service timeout limit
        self.timeout = rospy.get_param('/controller/timeout')
        # how greedy the search is
        self.eta = rospy.get_param('/local_rrt_detector/eta')
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
            '/detected_points', PointStamped, queue_size=1000)
        self.shapes_pub = rospy.Publisher(
            'mnlc_local_rrt_detector/shapes', Marker, queue_size=1000)
        self.state_machine = rospy.Subscriber(
            'mnlc_state_machine', std_msgs.msg.Int8, self.update_state_machine, queue_size=1)

    def detect_frontiers(self):
        points, line = Marker(), Marker()
        points.points = self.bounding_points
        points.header.frame_id, line.header.frame_id = self.map_metadata.header.frame_id, self.map_metadata.header.frame_id
        points.header.stamp, line.header.stamp = rospy.Time(0), rospy.Time(0)
        points.ns = line.ns = "markers"
        points.id, line.id = 0, 1
        points.type, line.type = points.POINTS, line.LINE_LIST
        points.action, line.action = points.ADD, line.ADD
        points.pose.orientation.w, line.pose.orientation.w = 1.0, 1.0
        line.scale.x, line.scale.y = 0.01, 0.01
        points.scale.x, points.scale.y = 0.1, 0.1
        points.color.r, points.color.g, points.color.b, points.color.a = 0.0 / \
            255.0, 0.0/255.0, 153.0/255.0, 1.0
        line.color.r, line.color.g, line.color.b, line.color.a = 124.0 / \
            255.0, 163.0/255.0, 193.0/255.0, 1.0
        points.lifetime, line.lifetime = rospy.Duration(), rospy.Duration()
        ix = math.sqrt(pow(points.points[0].x - points.points[2].x, 2) +
                       pow(points.points[0].y - points.points[0].y, 2))
        iy = math.sqrt(pow(points.points[0].x - points.points[0].x, 2) +
                       pow(points.points[0].y - points.points[2].y, 2))
        sx = (points.points[0].x + points.points[2].x) * 0.5
        sy = (points.points[0].y + points.points[2].y) * 0.5
        tr = Point()
        tr = points.points[4]
        v = [[tr.x, tr.y]]
        points.points = []
        rnew = []
        eta = self.eta
        point = PointStamped()
        point.header.frame_id = self.latest_map.header.frame_id
        p = Point()
        res = self.latest_map.info.resolution
        ox = self.latest_map.info.origin.position.x
        obstacle_cost = self.obstacle_cost
        listener = tf.TransformListener()
        while self.state != 2 and not rospy.is_shutdown() and self.error == False:
            time_init = rospy.get_time()
            data = self.latest_map.data
            width = self.latest_map.info.width
            rand = [(np.random.random() * ix) - (ix * 0.5) + sx,
                    (np.random.random() * iy) - (iy * 0.5) + sy]
            voi = v[0]
            min = math.sqrt(pow(voi[1] - rand[1], 2) +
                            pow(voi[0] - rand[0], 2))
            min_index = 0
            temp = 0.0
            for i in range(len(v)):
                vi = v[i]
                temp = math.sqrt(pow(vi[1] - rand[1], 2) +
                                 pow(vi[0] - rand[0], 2))
                if temp <= min:
                    min = temp
                    min_index = i
            near = v[min_index]
            new = []
            if math.sqrt(pow(near[1] - rand[1], 2) + pow(near[0] - rand[0], 2)) <= eta:
                new = rand
            else:
                m = (rand[1] - near[1]) / (rand[0] - near[0])
                sign = -1.0 if rand[0] - near[0] < 0.0 else 1.0
                new.append(
                    (sign * (math.sqrt(pow(eta, 2)) / ((pow(m, 2)) + 1))) + near[0])
                new.append(m * (new[0] - near[0]) + near[1])
                if rand[0] == near[0]:
                    new[0] = near[0]
                    new[1] = near[1] + eta
            rnew = new
            rez = res * 0.1
            norm = math.sqrt(pow(rnew[1] - near[1], 2) +
                             pow(rnew[0] - near[0], 2))
            steps = int(math.ceil(norm) / rez)
            xi = near
            obstacle = 0
            unkown = 0
            for step in range(steps):
                xn = rnew
                new = []
                if math.sqrt(pow(xi[1] - xn[1], 2) + pow(xi[0] - xn[0], 2)) <= rez:
                    new = xn
                else:
                    m = (xn[1] - xi[1]) / (xn[0] - xi[0])
                    sign = -1.0 if xn[0] - xi[0] < 0.0 else 1.0
                    new.append(
                        (sign * (math.sqrt(pow(rez, 2)) / ((pow(m, 2)) + 1))) + xi[0])
                    new.append(m * (new[0] - xi[0]) + xi[1])
                    if xn[0] == xi[0]:
                        new[0] = xi[0]
                        new[1] = xi[1] + rez
                xi = new
                c_data = data[int(
                    (math.floor((xi[1] - ox) / res) * width) + (math.floor((xi[0] - ox) / res)))]
                if c_data >= obstacle_cost:
                    obstacle = 1
                if c_data == -1:
                    unkown = 1
                    break
            rnew = xi
            obs_free = 1 if obstacle != 1 and unkown != 1 else 0 if obstacle == 1 else - \
                1 if unkown == 1 else 0
            if obs_free == -1:
                point.header.stamp = rospy.Time(0)
                point.point.x, point.point.y, point.point.z = rnew[0], rnew[1], 0.0
                points.points.append(point.point)
                self.shapes_pub.publish(points)
                self.detected_points_pub.publish(point)
                points.points = []
                v = []
                flag = 0
                while flag == 0:
                    try:
                        (trans, rot) = listener.lookupTransform(
                            '/map', 'base_footprint', rospy.Time(0))
                        flag = 1
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        flag = 0
                rnew[0] = trans[0]
                rnew[1] = trans[1]
                v.append(rnew)
                line.points = []
            elif obs_free == 1:
                v.append(copy.copy(rnew))
                p.x, p.y, p.z = rnew[0], rnew[1], 0.0
                line.points.append(copy.copy(p))
                p.x, p.y, p.z = near[0], near[1], 0.0
                line.points.append(copy.copy(p))
            self.shapes_pub.publish(line)
            # print("Calculating local rrt frontiers took: ", rospy.get_time() - time_init, ".")        

    def update_state_machine(self, state):
        self.state = state
        if state == 2:
            rospy.signal_shutdown(
                "Mapping complete. Node is now uneeded. Shutting down mnlc_local_rrt_detector.")
            sys.exit(0)

    def set_bounding_points(self, msg):
        self.bounding_points.append(copy.copy(msg.point))

    def update_map(self, map):
        self.latest_map = map

    def error_handler(self):
        self.error = True

    def run(self):
        while not rospy.is_shutdown() and self.error == False:
            rospy.spin()


if __name__ == '__main__':
    try:
        mnlc_local_rrt_detector().run()
    except rospy.ROSInterruptException:
        pass
