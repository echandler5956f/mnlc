#!/usr/bin/env python

from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg._Imu import Imu
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from cmath import pi
import numpy
import rospy
import math
import tf

lad = 0.375  # [m] look-ahead distance
w_radius = 3.52 / 100.0  # [m] wheel radius
w_base = 23.0 / 100.0  # [m] wheel base
ctrl_invl = 0.005  # [s] control loop interval
lfg = 0.00475 # look forward gain
kv = 1/0.22
ka = 0.02
kp = 0.625  # speed proportional gain


class NavHandler:

    def __init__(self):
        """
        Class constructor
        """
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
        self.target_speed = 0.0875
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
        self.execute_path(path.plan)
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


    def execute_path(self, path):
        timeInit = rospy.get_time()
        # print("Correct? ", path)
        last_index = len(path.poses) - 1
        t_index, lf  = self.search_target(path)
        # print(t_index)
        while last_index > t_index:
            linVel = kp * (self.target_speed - self.vel)
            angVel, t_index = self.pp_steering(path, t_index)
            self.send_speed(linVel, angVel)
        self.old_nearest = None
        self.stop()
        # print("Calculating execute_path took: ", rospy.get_time() - timeInit)

    def update_imu(self, msg):
        timeInit = rospy.get_time()
        self.acc = msg.linear_acceleration.x
        self.vel += self.acc * ctrl_invl
        # print("Calculating update_imu took: ", rospy.get_time() - timeInit)

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

    def pp_steering(self, path, prev_index): # don't laugh
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
