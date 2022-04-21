#!/usr/bin/env python

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry, Path, OccupancyGrid, GridCells
from geometry_msgs.msg import Twist, Point, PoseStamped
from rbe3002.msg import PointArray, Pose2d
from sensor_msgs.msg._Imu import Imu
from scipy.spatial import distance
from nav_msgs.srv import GetPlan
import pid_controller
import std_msgs.msg
import std_srvs.srv
import numpy as np
import rospy
import time
import math
import tf


class MNCLGlobalController(object):

    def __init__(self):
        rospy.loginfo("Initializing MNCLGlobalController.")
        rospy.init_node("MNCLGlobalController")
        # rospy.wait_for_service('/rtabmap/set_mode_mapping')
        # try:
        #     self.rtab_map_srv = rospy.ServiceProxy(
        #         '/rtabmap/set_mode_mapping', std_srvs.srv.Empty)
        # except rospy.ServiceException as e:
        #     print("RTabMap Mappping service call failed: %s" % e)
        self.sim = rospy.get_param('sim')
        self.rad_tolerance = rospy.get_param('rad_tolerance')
        self.tolerance = rospy.get_param('tolerance')
        self.alpha = rospy.get_param('alpha')
        self.beta = rospy.get_param('beta')
        self.w_base = rospy.get_param('w_base')  # [m] wheel base
        self.ctrl_invl = rospy.get_param(
            'ctrl_invl')  # [s] control loop interval
        # percentage as a decimal (i.e. 0.9 for 90% completion)
        self.desired_map_completion = rospy.get_param('desired_map_completion')
        self.spacing = rospy.get_param('spacing')
        self.cmd_vel_pub = rospy.Publisher(
            '/cmd_vel', Twist, None, queue_size=5)
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.update_odometry)
        self.imu_sub = rospy.Subscriber(
            '/imu', Imu, self.update_imu)
        self.new_commands_sub = rospy.Subscriber(
            'move_base_simple/goal', PoseStamped, self.commands, queue_size=1)
        self.new_commands_recieved_pub = rospy.Publisher(
            "new_commands", std_msgs.msg.Bool, queue_size=3)
        self.rtab_map_sub = rospy.Subscriber(
            '/MNCLGlobalCostmap/map', OccupancyGrid, self.update_rtabmap, queue_size=1)
        self.send_current_cell = rospy.Publisher(
            'MNCLGlobalController/current_cell', Point, queue_size=1)
        self.is_busy_pub = rospy.Publisher(
            "/MNCLGlobalCostmap/is_busy", std_msgs.msg.Bool, queue_size=1)
        self.visited_sub = rospy.Subscriber(
            'FrontierDetector/visited', GridCells, self.check_visited, queue_size=1)
        self.pose2d_pub = rospy.Publisher(
            '/MNCLGlobalController/pose2d', Pose2d, queue_size=1)
        self.filtered_frontiers_sub = rospy.Subscriber(
            '/frontier_filter/filtered_points', PointArray, self.update_filtered_frontiers, queue_size=1)
        self.loc_or_map_mode_pub = rospy.Publisher(
            '/MNCLGlobalController/loc_or_map_mode', std_msgs.msg.Bool, queue_size=3)
        # if localizing send True. If mapping send False
        # Start by mapping until frontier search algorithm visits a certain percentage of cells
        self.loc_or_map_mode = False
        self.filtered_frontiers = PointArray()
        self.visited = GridCells()
        self.rtabmap = OccupancyGrid()
        self.new_commands = False
        self.old_commands = self.new_commands
        self.odom_l = tf.TransformListener()
        self.odom_br = tf.TransformBroadcaster()
        self.c_pose = PoseStamped()
        self.c_pose.pose.position = (0.0, 0.0, 0.0)
        self.c_pose.pose.orientation = (0.0, 0.0, 0.0, 1.0)
        self.c_pose.header.stamp = rospy.Time.now()
        self.odom_br.sendTransform(
            self.c_pose.pose.position, self.c_pose.pose.orientation, self.c_pose.header.stamp, "base_footprint", "odom")
        self.cx, self.cy, self.ctheta = 0.0, 0.0, 0.0
        self.turningController = pid_controller.PIDController()
        if(self.sim):  # PID values for use in simulation
            self.turningController.PIDController(0.5, 0.0, 0.0, 0.0)
        else:  # PID values for use on physical robot
            # TODO tune PID values on robot
            self.turningController.PIDController(0.25, 0.0, 0.0, 0.0)
        rospy.sleep(1)
        rospy.loginfo("MNCLGlobalController node ready.")

    def commands(self, flag):
        time_init = rospy.get_time()
        self.stop()
        self.new_commands = not self.new_commands
        self.new_commands_recieved_pub.publish(True)

    def compare_commands(self):
        time_init = rospy.get_time()
        # if self.new_commands != self.old_commands:
        #     self.old_commands= self.new_commands
        #     return False
        return True

    def check_visited(self, visited):
        time_init = rospy.get_time()
        self.visited = visited
        # if self.rtabmap.header.seq > 1 and len(self.rtabmap.data) > 1:
        if self.visited.cells.count() > (self.desired_map_completion * self.rtabmap.info.width * self.rtabmap.info.height):
            rospy.wait_for_service('/rtabmap/set_mode_localization')
            try:
                self.rtab_map_srv = rospy.ServiceProxy(
                    '/rtabmap/set_mode_mapping ', std_srvs.srv.Empty)
            except rospy.ServiceException as e:
                print("RTabMap Mappping service call failed: %s" % e)
            self.loc_or_map_mode_pub.publish(True)
            rospy.loginfo("Switching RTabMap to Localization mode!!!")
        else:
            self.loc_or_map_mode_pub.publish(False)

    def update_odometry(self, msg):
        time_init = rospy.get_time()
        # rospy.sleep(self.ctrl_invl)
        # if self.rtabmap.info.resolution > 0.0:
        # rospy.loginfo("Updating odometry.")
        self.c_pose.pose = msg.pose.pose
        self.c_pose.header.stamp = rospy.Time.now()
        self.odom_br.sendTransform(
            (self.c_pose.pose.position.x, self.c_pose.pose.position.y,
             self.c_pose.pose.position.z),
            (self.c_pose.pose.orientation.x, self.c_pose.pose.orientation.y,
             self.c_pose.pose.orientation.z, self.c_pose.pose.orientation.w),
            rospy.Time.now(), "base_footprint", "odom")
        (tr, rot) = self.odom_l.lookupTransform(
            "odom", "base_footprint", rospy.Time(0))
        self.cx = tr[0]
        self.cy = tr[1]
        self.rx = self.cx - ((self.w_base / 2) * math.cos(self.ctheta))
        self.ry = self.cy - ((self.w_base / 2) * math.sin(self.ctheta))
        roll, pitch, self.ctheta = euler_from_quaternion(rot)
        point = Point()
        point.x = int(math.floor(
            (self.cx - self.rtabmap.info.origin.position.x) / 0.05))
        point.y = int(math.floor(
            (self.cy - self.rtabmap.info.origin.position.y) / 0.05))
        self.send_current_cell.publish(point)
        pose2d = Pose2d()
        pose2d.cx = self.cx
        pose2d.cy = self.cy
        pose2d.ctheta = self.ctheta
        self.pose2d_pub.publish(pose2d)
        # rospy.loginfo("Odometry updated.")

    def update_imu(self, msg):
        time_init = rospy.get_time()
        # rospy.loginfo("Updating IMU.")
        self.acc = distance.euclidean(
            msg.linear_acceleration.x, msg.linear_acceleration.y)
        self.vel = distance.euclidean(
            msg.linear_acceleration.x, msg.linear_acceleration.y) * self.ctrl_invl
        # rospy.loginfo("IMU updated.")

    def update_rtabmap(self, map):
        time_init = rospy.get_time()
        self.rtabmap = map

    def update_filtered_frontiers(self, filtered_frontiers):
        time_init = rospy.get_time()
        self.filtered_frontiers = filtered_frontiers

    def send_speed(self, linear_speed, angular_speed):
        time_init = rospy.get_time()
        # rospy.loginfo("Sending velocity targets to the Turtlebot.")
        msg_cmd_vel = Twist()
        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.angular.z = angular_speed
        self.cmd_vel_pub.publish(msg_cmd_vel)
        # rospy.loginfo("Velocity targets published.")

    def turnTo(self, orientation):
        time_init = rospy.get_time()
        # uses a simple PID control loop to turn in place to a desired heading
        self.turningController.reset()
        roll, pitch, yaw = euler_from_quaternion(orientation, 'rxyz')
        reachedHeading = False
        while (not reachedHeading and not rospy.is_shutdown()):
            error = yaw - self.ctheta
            if(abs(error) < 0.1):
                reachedHeading = True
                self.stop()
                break
            else:
                angularSpeed = self.turningController.ComputeEffort(error)
                self.send_speed(0.0, angularSpeed)
            rospy.sleep(self.ctrl_invl)

    def stop(self):
        time_init = rospy.get_time()
        rospy.loginfo("Sending zero-velocity target to the Turtlebot.")
        self.send_speed(0.0, 0.0)
        rospy.loginfo("Zero-velocity sent.")


class PurePersuit(MNCLGlobalController):

    def __init__(self):
        super(PurePersuit, self).__init__()
        rospy.loginfo("Beginning Pure Persuit initialization.")
        self.old_nearest = None
        self.goal_pose = PoseStamped()
        self.kv = rospy.get_param('kv')
        self.ka = rospy.get_param('ka')
        self.lad = rospy.get_param('lad')
        self.lfg = rospy.get_param('lfg')
        self.target_speed = rospy.get_param('target_speed')
        self.target_acc = rospy.get_param('target_acc')
        self.goal_sub = rospy.Subscriber(
            '/move_base_simple/goal', PoseStamped, self.handle_goal_pose, queue_size=1)
        self.smoothed_pth_pub = rospy.Publisher(
            '/pure_persuit/smoothed_path', Path, queue_size=5)
        # rospy.sleep(2)
        # while self.rtabmap.header.seq > 1 and len(self.rtabmap.data) > 1:
        #     rospy.sleep(self.ctrl_invl)
        rospy.loginfo("Pure Persuit ready.")

    def handle_goal_pose(self, msg):
        rospy.loginfo("Handling goal pose.")
        time_init = rospy.get_time()
        rospy.wait_for_service('plan_path')
        self.is_busy_pub.publish(True)
        self.goal_pose = msg
        try:
            path_srv = rospy.ServiceProxy('plan_path', GetPlan)
            start_pose = PoseStamped()
            start_pose.pose.position.x = self.cx
            start_pose.pose.position.y = self.cy
            goal_pose = msg
            path = path_srv(start_pose, goal_pose, 1.0)
        except rospy.ServiceException as e:
            print("PurePersuit service call failed: %s." % e)
        rospy.loginfo("PurePersuit service call successful.")
        if len(path.plan.poses) - 1 > 1:
            injected_path = self.inject_waypoints(path.plan)
            smoothed_path = self.path_smoothing(injected_path)
        self.smoothed_pth_pub.publish(smoothed_path)
        self.execute_path(smoothed_path)
        self.is_busy_pub.publish(False)
        rospy.loginfo("Pure Persuit has reached its target.")

    def inject_waypoints(self, path):
        rospy.loginfo("Injecting waypoints.")
        time_init = rospy.get_time()
        new_path = Path()
        for i in range(0, len(path.poses) - 1):
            vector = np.array([path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x,
                              path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y])
            mag = math.sqrt(math.pow(vector[0], 2) + math.pow(vector[1], 2))
            num_points = math.ceil(mag / self.spacing)
            vector = np.array(
                [vector[0] / mag, vector[1] / mag]) * self.spacing
            for j in range(int(num_points)):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = path.poses[i].pose.position.x + (
                    vector[0] * j)
                pose.pose.position.y = path.poses[i].pose.position.y + (
                    vector[1] * j)
                new_path.poses.append(pose)
        last_pose = PoseStamped()
        last_pose.header.frame_id = 'map'
        last_pose.header.stamp = rospy.Time.now()
        last_pose.pose.position.x = path.poses[len(
            path.poses) - 1].pose.position.x
        last_pose.pose.position.y = path.poses[len(
            path.poses) - 1].pose.position.y
        new_path.poses.append(last_pose)
        new_path.header.frame_id = 'map'
        new_path.header.stamp = rospy.Time.now()
        print("Injecting waypoints took: ", rospy.get_time() - time_init, ".")
        rospy.loginfo("Finished injecting waypoints.")
        return new_path

    def path_smoothing(self, path):
        time_init = rospy.get_time()
        npath = []
        for pose in path.poses:
            npath.append([pose.pose.position.x, pose.pose.position.y])
        pth = npath
        change = self.rad_tolerance
        while change >= self.rad_tolerance:
            change = 0.0
            for i in range(1, len(npath) - 1):
                for j in range(0, len(npath[i])):
                    aux = npath[i][j]
                    npath[i][j] += self.alpha * (pth[i][j] - npath[i][j]) + self.beta * (
                        npath[i - 1][j] + npath[i + 1][j] - (2.0 * npath[i][j]))
                    change += abs(aux - npath[i][j])
        smoothed_path = Path()
        for arr in npath:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = arr[0]
            pose.pose.position.y = arr[1]
            smoothed_path.poses.append(pose)
        smoothed_path.header.frame_id = 'map'
        smoothed_path.header.stamp = rospy.Time.now()
        print("Smoothing path took: ", rospy.get_time() - time_init, ".")
        return smoothed_path

    # def path_smoothing(self, path):
    #     rospy.loginfo("Smoothing path.")
    #     time_init = rospy.get_time()
    #     npath = path
    #     smoothed_path = Path()
    #     smoothed_path.header.frame_id = 'map'
    #     alpha = self.alpha
    #     beta = self.beta
    #     rad_tolerance = self.rad_tolerance
    #     change = rad_tolerance
    #     while change >= rad_tolerance:
    #         del smoothed_path.poses[0:len(smoothed_path.poses) - 2]
    #         change = 0.0
    #         for i in range(1, len(npath.poses) - 1):
    #             aux = npath.poses[i].pose.position.x
    #             npath.poses[i].pose.position.x += alpha * (path.poses[i].pose.position.x - npath.poses[i].pose.position.x) + beta * (
    #                 npath.poses[i - 1].pose.position.x + npath.poses[i + 1].pose.position.x - (2.0 * npath.poses[i].pose.position.x))
    #             auy = npath.poses[i].pose.position.y
    #             npath.poses[i].pose.position.y += alpha * (path.poses[i].pose.position.y - npath.poses[i].pose.position.y) + beta * (
    #                 npath.poses[i - 1].pose.position.y + npath.poses[i + 1].pose.position.y - (2.0 * npath.poses[i].pose.position.y))
    #             dx = abs(aux - npath.poses[i].pose.position.x)
    #             dy = abs(auy - npath.poses[i].pose.position.y)
    #             change += distance.euclidean(dy, dx)
    #             pose = PoseStamped()
    #             pose.header.frame_id = 'map'
    #             pose.header.stamp = rospy.Time.now()
    #             pose.pose.position.x = npath.poses[i].pose.position.x
    #             pose.pose.position.y = npath.poses[i].pose.position.y
    #             smoothed_path.poses.append(pose)
    #     smoothed_path.header.stamp = rospy.Time.now()
    #     print("Smoothing path took: ", rospy.get_time() - time_init, ".")
    #     rospy.loginfo("Smoothing path.")
    #     return smoothed_path

    def execute_path(self, path):
        rospy.loginfo("Executing Pure Persuit path following.")
        time_init = rospy.get_time()
        dx = path.poses[5].pose.position.x - self.cx
        dy = path.poses[5].pose.position.y - self.cy
        it = math.atan2(dy, dx)
        iq = quaternion_from_euler(0.0, 0.0, it, 'rxyz')
        self.turnTo(iq)
        last_index = len(path.poses) - 1
        t_index, lf = self.search_target(path)
        # print("t_index is: ", t_index, " and last_index is: ", last_index)
        next_path_time = time.time()
        while last_index > t_index and self.compare_commands():
            if time.time() > next_path_time:
                lin_v = self.kv * self.target_speed + self.ka * self.target_acc
                ang_v, t_index = self.pp_steering(path, t_index)
                # print("t_index is: ", t_index)
                self.send_speed(lin_v, ang_v)
                next_path_time = time.time() + self.ctrl_invl
        self.old_nearest = None
        self.stop()
        rospy.loginfo("Pure Persuit is stopping the Turtlebot.")

    def search_target(self, path):
        # rospy.loginfo("Searching for target via-point indices.")
        time_init = rospy.get_time()
        if self.old_nearest is None:
            dx = [self.rx - ip.pose.position.x for ip in path.poses]
            dy = [self.ry - ip.pose.position.y for ip in path.poses]
            dist = np.hypot(dx, dy)
            index = np.argmin(dist)
            self.old_nearest = index
        else:
            index = self.old_nearest
            dt_index = math.sqrt(pow(self.rx - path.poses[index].pose.position.x, 2) +
                                 pow(self.ry - path.poses[index].pose.position.y, 2))
            while 1 and self.compare_commands():
                dn_index = math.sqrt(pow(self.rx - path.poses[index + 1].pose.position.x, 2) +
                                     pow(self.ry - path.poses[index + 1].pose.position.y, 2))
                if dt_index < dn_index:
                    break
                index = index + \
                    1 if (index + 1) < len(path.poses) else index
                dt_index = dn_index
            self.old_nearest = index
        lf = self.lfg * self.vel + self.lad
        while self.compare_commands() and lf > math.sqrt(pow(self.rx - path.poses[index].pose.position.x, 2) +
                                                         pow(self.ry - path.poses[index].pose.position.y, 2)):
            if index + 1 >= len(path.poses):
                break
            index += 1
        # rospy.loginfo("Found valid index.")
        return index, lf

    def pp_steering(self, path, prev_index):
        # rospy.loginfo("Steering to index.")
        time_init = rospy.get_time()
        (index, lf) = self.search_target(path)
        if prev_index >= index:
            index = prev_index
        if index < len(path.poses):
            tx = path.poses[index].pose.position.x
            ty = path.poses[index].pose.position.y
        else:
            tx = path.poses[-1].pose.position.x
            ty = path.poses[-1].pose.position.y
            index = len(path.poses) - 1
        sigma = math.atan2(2.0 * self.w_base * math.sin(math.atan2(ty -
                           self.ry, tx - self.rx) - self.ctheta) / lf, 1.0)
        # rospy.loginfo("Index sigma found.")
        return sigma, index

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    PurePersuit().run()
