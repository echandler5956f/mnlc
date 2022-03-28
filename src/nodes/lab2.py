#!/usr/bin/env python2

from time import sleep

from numpy import empty
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from numpy.linalg import norm
import rospy
import math
import tf


class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        # REQUIRED CREDIT
        # Initialize node, name it 'lab2'
        rospy.init_node('lab2')
        # robot intrinsics
        self.w_radius = 3.52 / 100.0  # cm
        self.w_base = 23.0 / 100.0  # cm
        # communication rate:
        self.ctrl_invl = 0.1  # s
        # Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                           Twist, None, queue_size=10)
        # Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        # When a message is received, call self.update_odometry
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.update_odometry)
        # Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        # When a message is received, call self.go_to
        self.goal_sub = rospy.Subscriber(
            'move_base_simple/goal', PoseStamped, self.go_to)
        self.odom_l = tf.TransformListener()
        self.odom_br = tf.TransformBroadcaster()
        self.c_pose = PoseStamped()
        self.c_pose.pose.position = (0, 0, 0)
        self.c_pose.pose.orientation = (0, 0, 0, 1)
        self.c_pose.header.stamp = rospy.Time.now()
        self.odom_br.sendTransform(
            self.c_pose.pose.position, self.c_pose.pose.orientation, self.c_pose.header.stamp, "base_footprint", "odom")
        self.cx, self.cy, self.ctheta = 0.0, 0.0, 0.0  # current 2D pose
        self.px, self.py, self.ptheta = 0.0, 0.0, 0.0  # previous 2D pose
        rospy.sleep(2)

    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        # REQUIRED CREDIT
        # Make a new Twist message
        msg_cmd_vel = Twist()
        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.angular.z = angular_speed
        # Publish the message
        self.cmd_vel_pub.publish(msg_cmd_vel)

    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        # REQUIRED
        # past = rospy.Time.now()
        # (self.px, self.py, self.ptheta) = (self.cx, self.cy, self.ctheta)
        # while math.sqrt(math.pow((self.cx - self.px), 2) + math.pow((self.cy - self.py), 2)) < distance:
        #     self.send_speed(linear_speed, 0) if (
        #         (rospy.Time.now().nsecs - past.nsecs) % self.ctrl_invl) else rospy.spin()

        (self.px, self.py, self.ptheta) = (self.cx, self.cy, self.ctheta)
        reachedT = False
        while (not reachedT and not rospy.is_shutdown()):
            currentDistance = math.sqrt(
                math.pow((self.cx - self.px), 2) + math.pow((self.cy - self.py), 2))
            if (currentDistance >= distance):
                reachedT = True
                self.send_speed(0, 0)
            else:
                self.send_speed(linear_speed, 0)
                rospy.sleep(self.ctrl_invl)

    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        # REQUIRED CREDIT
        # past = rospy.Time.now()
        # (self.px, self.py, self.ptheta) = (self.cx, self.cy, self.ctheta)
        # while self.ctheta - self.ptheta < angle:
        #     self.send_speed(0, aspeed) if (
        #         (rospy.Time.now().nsecs - past.nsecs) % self.ctrl_invl) else rospy.spin()
        
        (self.px, self.py, self.ptheta) = (self.cx, self.cy, self.ctheta)
        reachedT = False
        while (not reachedT and not rospy.is_shutdown()):
            angularDistance = self.ctheta - self.ptheta
            if (angularDistance >= angle):
                reachedT = True
                self.send_speed(0, 0)
            else:
                self.send_speed(0, aspeed)
                rospy.sleep(self.ctrl_invl)

    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        # REQUIRED CREDIT
        fx = msg.pose.position.x  # final x
        fy = msg.pose.position.y  # final y
        quat = msg.pose.orientation  # the final angle to turn to
        q = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, ft = euler_from_quaternion(q)
        d = math.sqrt(math.pow((fx - self.cx), 2) +
                      math.pow((fy - self.cy), 2))  # Euclidean distance
        it = (math.atan2(fy - self.cy, fx - self.cx)) - \
            self.ctheta  # the first angle to turn to
        self.rotate(it, -1)  # first turn at constant velocity
        self.drive(d, 1)  # drive distance 'd' at constant velocity
        self.rotate(ft, 1)  # rotate to final angle at constant velocity

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        # REQUIRED CREDIT
        try:
            self.c_pose.pose = msg.pose
            self.c_pose.header.stamp = rospy.Time.now
            self.odom_br.sendTransform(
                self.c_pose.pose.position, self.c_pose.pose.orientation, self.c_pose.header.stamp, "base_footprint", "odom")
            self.cx = self.c_pose.pose.position.x
            self.cy = self.c_pose.pose.position.y
            roll, pitch, self.ctheta = (euler_from_quaternion(
                self.c_pose.pose.orientation))
        except:
            print("Waiting")

    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        # EXTRA CREDIT
        # TODO

    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        # EXTRA CREDIT
        # TODO

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    Lab2().run()
