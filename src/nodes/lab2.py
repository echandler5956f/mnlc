#!/usr/bin/env python2

from cmath import pi
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import math
import tf


class PIDController():
    # basic PID controller borrowed from my RBE2002 final project
    sumError = 0
    errorBound = 0
    (Kp, Ki, Kd) = (0, 0, 0)
    prevError = 0
    currError = 0
    currEffort = 0
    effortCap = 0

    def PIDController(self, Kp, Ki, Kd, errorBound):
        """
        Class constructor
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.errorBound = errorBound
        self.effortCap = 0

    def ComputeEffort(self, error):
        self.currError = error  # store in case we want it later
        self.sumError += self.currError
        if(self.errorBound > 0):  # cap error; errorBound == 0 means don't cap
            # you could multiply sumError by Ki to make it scale
            if(abs(self.sumError) > self.errorBound):
                # if we exceeded the limit, just subtract it off again
                self.sumError -= self.currError
        derivError = self.currError - self.prevError
        self.prevError = self.currError
        self.currEffort = self.Kp * self.currError + \
            self.Ki * self.sumError + self.Kd * derivError
        if(self.effortCap > 0 and self.currEffort > self.effortCap):
            self.currEffort = self.effortCap
        return self.currEffort

    def reset(self):
        self.currError = 0
        self.sumError = 0
        self.prevError = 0
        self.currEffort = 0


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
        self.ctrl_invl = 0.01  # s
        # Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel_pub = rospy.Publisher(
            '/cmd_vel', Twist, None, queue_size=10)
        # Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        # When a message is received, call self.update_odometry
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.update_odometry)
        # Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        # When a message is received, call self.go_to
        self.goal_sub = rospy.Subscriber(
            '/move_base_simple/goal', PoseStamped, self.driveToPose)
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

        ######################### PID Stuff #############################

        self.positionController = PIDController()
        self.positionController.PIDController(0.1, 0.25, 0.02, 0.0875)

        self.headingController = PIDController()
        self.headingController.PIDController(0.0875, 0.0, 0.075, 0.0)

        self.turningController = PIDController()
        self.turningController.PIDController(0.25, 0.0, 0.0, 0.0)

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

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        # REQUIRED CREDIT
        try:
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
            roll, pitch, self.ctheta = euler_from_quaternion(rot)
        except:
            pass

    def driveToPoint(self, position):
        # uses two PID loops-
        # one for correcting heading so that the robot is always facing the point vector,
        # and one to move forward
        self.positionController.reset()
        self.headingController.reset()
        fx = position.x  # final x
        fy = position.y  # final y
        reachedPosition = False
        while (not reachedPosition and not rospy.is_shutdown()):
            dx = fx - self.cx
            dy = fy - self.cy
            thetaError = self.ctheta - math.atan2(dy, dx)
            # atan2 is funky, im not sure why this works
            distError = pow(pow(abs(dx), 2) + pow(abs(dy), 2), 0.5)
            if (abs(distError) < 0.00625):
                reachedPosition = True
                self.stop()
                break
            else:
                if(thetaError > pi + 0.2 or thetaError < - pi - 0.2):
                    # override the arc controller if the theta error is too large
                    # this is a rare case and may take one or two tries to get it right,
                    # but the system converges on the desired point eventually
                    # 99% of the time the robot will never enter this section of the control algorithm
                    tmp = PoseStamped()
                    orientation = tmp.pose.orientation
                    (orientation.x, orientation.y, orientation.z, orientation.w) = quaternion_from_euler(
                        0, 0, math.atan2(dy, dx), 'rxyz')
                    # turn in place to corrected orientation
                    self.turnTo(orientation)
                    # update error calculations after the emergency turnTo
                    self.headingController.reset()
                    self.positionController.reset()
                    # try again but at the easier angle
                else:
                    # default case is to drive to a point via arcs
                    IKLeftSpeed = self.positionController.ComputeEffort(
                        distError) + self.headingController.ComputeEffort(thetaError)  # cm/s
                    IKRightSpeed = self.positionController.ComputeEffort(
                        distError) - self.headingController.ComputeEffort(thetaError)  # cm/s
                    IKDriveBaseVelocity = (IKRightSpeed + IKLeftSpeed) / 2.0
                    IKDriveBaseOmega = (
                        IKRightSpeed - IKLeftSpeed) / self.w_base
                    self.send_speed(IKDriveBaseVelocity, IKDriveBaseOmega)
                    rospy.sleep(self.ctrl_invl)

    def turnTo(self, orientation):
        # uses a simple PID control loop to turn in place to a desired heading
        self.turningController.reset()
        quat = orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, tHeading = euler_from_quaternion(q)
        reachedHeading = False
        while (not reachedHeading and not rospy.is_shutdown()):
            error = tHeading - self.ctheta
            if(abs(error) < 0.075):
                reachedHeading = True
                self.stop()
                break
            else:
                angularSpeed = self.turningController.ComputeEffort(error)
                self.send_speed(0.0, angularSpeed)
            rospy.sleep(self.ctrl_invl)

    def smooth_drive(self, distance, max_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        # EXTRA CREDIT
        self.positionController.reset()
        self.positionController.effortCap = max_speed
        (self.px, self.py, self.ptheta) = (self.cx, self.cy, self.ctheta)
        reachedP = False
        while (not reachedP and not rospy.is_shutdown()):
            currentDistance = math.sqrt(
                math.pow((self.cx - self.px), 2) + math.pow((self.cy - self.py), 2))
            error = distance - currentDistance
            if (abs(error) < 0.1):
                reachedP = True
                self.stop()
                break
            else:
                speed = self.positionController.ComputeEffort(error)
                self.send_speed(speed, 0)
            rospy.sleep(self.ctrl_invl)

    def driveToPose(self, poseStamped):
        """
        Drives to a given pose in an arc followed by a turn to heading.
        :param msg [PoseStamped] The target pose.
        """
        # EXTRA CREDIT
        self.driveToPoint(poseStamped.pose.position)
        # drives to a point via arcs
        self.turnTo(poseStamped.pose.orientation)
        # turns in place to a certain orientation

    def stop(self):
        self.send_speed(0, 0)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

####################################################################################
############### The three drive functions below have been deprecated ###############
###################### in favor of superior motion algorithms ######################
####################################################################################

    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        # REQUIRED
        (self.px, self.py, self.ptheta) = (self.cx, self.cy, self.ctheta)
        reachedP = False
        while (not reachedP and not rospy.is_shutdown()):
            currentDistance = math.sqrt(
                math.pow((self.cx - self.px), 2) + math.pow((self.cy - self.py), 2))
            if (abs(currentDistance - distance) < 0.1):
                reachedP = True
                self.stop()
                break
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
        reachedT = False
        magAspeed = abs(aspeed)
        while (not reachedT and not rospy.is_shutdown()):
            error = angle - self.ctheta
            if (abs(error) < 0.075):
                reachedT = True
                self.stop()
                break
            else:
                if error < 0:
                    self.send_speed(0, -magAspeed)
                if error > 0:
                    self.send_speed(0, magAspeed)
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
        it = (math.atan2(fy - self.cy, fx - self.cx)) - self.ctheta
        self.rotate(it, 0.15)  # first turn at constant velocity
        self.drive(d, 0.2)  # drive distance 'd' at constant velocity
        self.rotate(ft, 0.15)  # rotate to final angle at constant velocity


if __name__ == '__main__':
    Lab2().run()
