#!/usr/bin/env python

from tf.transformations import euler_from_quaternion
# from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg._Imu import Imu
from geometry_msgs.msg import Twist
from scipy.spatial import distance
from nav_msgs.srv import GetPlan
import numpy as np
import numpy
import rospy
import time
import math
import tf

lad = 0.25  # [m] look-ahead distance
w_radius = 3.52 / 100.0  # [m] wheel radius
w_base = 23.0 / 100.0  # [m] wheel base
ctrl_invl = 0.0025  # [s] control loop interval
lfg = 0.0375  # look forward gain
kv = 1/0.22
ka = 0.0375
kp = 0.4125  # speed proportional gain


class NavHandler:

    def __init__(self):
        """
        Class constructor
        """
        self.nextTime = time.time()
        # Initialize the node and call it "nav_handler"
        rospy.init_node("nav_handler")
        self.cmd_vel_pub = rospy.Publisher(
            '/cmd_vel', Twist, None, queue_size=5)
        self.goal_sub = rospy.Subscriber(
            '/move_base_simple/goal', PoseStamped, self.handleGoalPose)
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.update_odometry)
        self.odom_sub = rospy.Subscriber(
            '/imu', Imu, self.update_imu)
        self.odom_l = tf.TransformListener()
        self.odom_br = tf.TransformBroadcaster()
        self.injected_path_pub = rospy.Publisher(
            '/path_planner/injected_path', Path, queue_size=1)
        self.smoothed_pth_pub = rospy.Publisher(
            '/path_planner/smoothed_path', Path, queue_size=1)
        self.c_pose = PoseStamped()
        self.c_pose.pose.position = (0.0, 0.0, 0.0)
        self.c_pose.pose.orientation = (0.0, 0.0, 0.0, 1.0)
        self.c_pose.header.stamp = rospy.Time.now()
        self.odom_br.sendTransform(
            self.c_pose.pose.position, self.c_pose.pose.orientation, self.c_pose.header.stamp, "base_footprint", "odom")
        self.cx, self.cy, self.ctheta = 0.0, 0.0, 0.0  # current 2D pose
        self.px, self.py, self.ptheta = 0.0, 0.0, 0.0  # previous 2D pose
        self.rx = self.cx - ((w_base / 2) * math.cos(self.ctheta))
        self.ry = self.cy - ((w_base / 2) * math.sin(self.ctheta))
        self.vel, self.acc = 0.0, 0.0
        self.target_speed = 0.0625
        self.target_acc = 0.05
        self.old_nearest = None
        # Sleep to allow roscore to do some housekeeping
        rospy.sleep(2.0)
        rospy.loginfo("Nav Handler node ready")

    def send_speed(self, linear_speed, angular_speed):
        timeInit = rospy.get_time()
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        # Make a new Twist message
        msg_cmd_vel = Twist()
        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.angular.z = angular_speed
        # Publish the message
        # print(msg_cmd_vel)
        self.cmd_vel_pub.publish(msg_cmd_vel)
        # print("Calculating send_speed took: ", rospy.get_time() - timeInit)

    def handleGoalPose(self, msg):
        timeInit = rospy.get_time()
        rospy.wait_for_service('plan_path')
        try:
            path_srv = rospy.ServiceProxy('plan_path', GetPlan)
            startPose = PoseStamped()
            startPose.pose.position.x = self.cx
            startPose.pose.position.y = self.cy
            goalPose = msg
            path = path_srv(startPose, goalPose, 0.5)
            print("Planning path!")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        # higherResPath = self.inject_waypoints(path.plan, 0.15)
        b = 0.375
        a = 1.0 - b
        smoothedPath = self.path_smoothing(path.plan, a, b, 0.75)
        self.execute_path(smoothedPath)
        # print("Calculating handleGoalPose took: ", rospy.get_time() - timeInit)

    def update_odometry(self, msg):
        timeInit = rospy.get_time()
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
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
        self.odom_br.sendTransform(
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
            rospy.Time.now(), "map", "odom")
        # not sure why but the map moves with the robot when amcl is on if this command is not included
        self.cx = tr[0]
        self.cy = tr[1]
        self.rx = self.cx - ((w_base / 2) * math.cos(self.ctheta))
        self.ry = self.cy - ((w_base / 2) * math.sin(self.ctheta))
        roll, pitch, self.ctheta = euler_from_quaternion(rot)
        # print("Calculating update_odometry took: ", rospy.get_time() - timeInit)

    def update_imu(self, msg):
        timeInit = rospy.get_time()
        if time.time > self.nextTime:
            self.acc = distance.euclidean(msg.linear_acceleration.x, msg.linear_acceleration.y)
            self.vel = distance.euclidean(msg.linear_acceleration.x, msg.linear_acceleration.y) * ctrl_invl
            self.nextTime = time.time() + ctrl_invl
            # print("Calculating update_imu took: ", rospy.get_time() - timeInit)

    def execute_path(self, path):
        timeInit = rospy.get_time()
        # print("Correct? ", path)
        last_index = len(path.poses) - 1
        t_index, lf = self.search_target(path)
        # print(t_index)
        nextPthTime = time.time()
        while last_index > t_index:
            if time.time() > nextPthTime:
                linVel = kv * self.target_speed + ka * self.target_acc
                angVel, t_index = self.pp_steering(path, t_index)
                self.send_speed(linVel, angVel)
                nextPthTime = time.time() + ctrl_invl
        self.old_nearest = None
        self.stop()
        # print("Calculating execute_path took: ", rospy.get_time() - timeInit)

    # def inject_waypoints(self, path, spacing):
    #     newPath = Path()
    #     # newPath.poses.append(path.poses[0])
    #     for i in range(len(path.poses) - 1):
    #         vec = np.array([path.poses[i+1].pose.position.x - path.poses[i].pose.position.x,
    #                        path.poses[i+1].pose.position.y - path.poses[i].pose.position.y])
    #         numPoints = int(math.ceil(distance.euclidean(vec[0], vec[1]) / spacing))
    #         vecN = vec / distance.euclidean(vec[0], vec[1])
    #         for j in range(numPoints - 1):
    #             pose = PoseStamped()
    #             pose.pose.position.x = path.poses[0].pose.position.x + vecN[0] * j
    #             pose.pose.position.y = path.poses[1].pose.position.y + vecN[1] * j
    #             newPath.poses.append(pose)
    #     newPath.poses.append(path.poses[len(path.poses) - 1])
    #     newPath.poses.reverse()
    #     newPath.header.frame_id = 'map'
    #     newPath.header.stamp = rospy.Time.now()
    #     self.injected_path_pub.publish(newPath)
    #     return newPath

    def path_smoothing(self, path, a, b, tolerance):
        newPath = path
        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(path.poses) - 1):
                aux = newPath.poses[i].pose.position.x
                newPath.poses[i].pose.position.x += a * (path.poses[i].pose.position.x - newPath.poses[i].pose.position.x) + b * (
                    newPath.poses[i - 1].pose.position.x + newPath.poses[i + 1].pose.position.x - (2.0 * newPath.poses[i].pose.position.x))
                auy = newPath.poses[i].pose.position.y
                newPath.poses[i].pose.position.y += a * (path.poses[i].pose.position.y - newPath.poses[i].pose.position.y) + b * (
                    newPath.poses[i - 1].pose.position.y + newPath.poses[i + 1].pose.position.y - (2.0 * newPath.poses[i].pose.position.y))
        change += distance.euclidean(
            aux - newPath.poses[i].pose.position.x, auy - newPath.poses[i].pose.position.y)
        newPath.header.stamp = rospy.Time.now()
        self.smoothed_pth_pub.publish(newPath)
        return newPath

    def search_target(self, path):
        timeInit = rospy.get_time()
        if self.old_nearest is None:
            dx = [self.rx - ip.pose.position.x for ip in path.poses]
            dy = [self.ry - ip.pose.position.y for ip in path.poses]
            dist = numpy.hypot(dx, dy)
            index = numpy.argmin(dist)
            self.old_nearest = index
        else:
            index = self.old_nearest
            dt_index = math.sqrt(pow(self.rx - path.poses[index].pose.position.x, 2) +
                                 pow(self.ry - path.poses[index].pose.position.y, 2))
            while 1:
                dn_index = math.sqrt(pow(self.rx - path.poses[index + 1].pose.position.x, 2) +
                                     pow(self.ry - path.poses[index + 1].pose.position.y, 2))
                if dt_index < dn_index:
                    break
                index = index + \
                    1 if (index + 1) < len(path.poses) else index
                dt_index = dn_index
            self.old_nearest = index
        lf = lfg * self.vel + lad
        while lf > math.sqrt(pow(self.rx - path.poses[index].pose.position.x, 2) +
                             pow(self.ry - path.poses[index].pose.position.y, 2)):
            if index + 1 >= len(path.poses):
                break
            index += 1
        # print("Calculating search_target took: ", rospy.get_time() - timeInit)
        return index, lf

    def pp_steering(self, path, prev_index):  # don't laugh
        timeInit = rospy.get_time()
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
        sigma = math.atan2(2.0 * w_base * math.sin(math.atan2(ty -
                           self.ry, tx - self.rx) - self.ctheta) / lf, 1.0)
        # print("Calculating pp_steering took: ", rospy.get_time() - timeInit)
        return sigma, index

    def stop(self):
        self.send_speed(0, 0)

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


if __name__ == '__main__':
    NavHandler().run()
