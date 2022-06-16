#!/usr/bin/env python

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg._Imu import Imu
from scipy.spatial import distance
import move_base_msgs.msg as mb
from nav_msgs.srv import GetMap
import rbe3002.msg as rbem
import std_msgs.msg
import std_srvs.srv
import numpy as np
import actionlib
import roslib
import rospy
import math
import tf

roslib.load_manifest('rbe3002')


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

class averagingFilter():
    filter_size = 5
    data = []
    index = 0

    def averagingFilter(self, size):
        self.filter_size = size
        self.data = np.zeros((self.filter_size, 1), dtype=np.float)
    
    def average(self, vel):
        self.data[self.index] = vel
        self.index = 0 if (self.index >= self.filter_size - 1) else (self.index + 1)
        average = 0.0
        for i in range(0, self.filter_size):
            average = average + self.data[i]
        return average / self.filter_size

class mnlc_pure_pursuit():

    def __init__(self):
        self.error = False
        rospy.loginfo("Initializing mnlc_pure_pursuit.")
        rospy.init_node("mnlc_pure_pursuit")
        self.initialize_params()
        rospy.sleep(self.start_time)
        self.safe_start()
        # give gazebo a chance to warm up so rtabmap doesnt raise an error about not having a map
        rospy.loginfo("mnlc_pure_pursuit node ready.")

    def initialize_params(self):
        self.t = rospy.get_time()
        self.acc = std_msgs.msg.Float32()
        self.acc.data = 0.0
        self.start_time = rospy.get_param('/pure_pursuit/start_time')
        self.ctrl_invl = rospy.get_param(
            '/controller/ctrl_invl')  # [s] control loop interval
        self.ctrl_rate = rospy.Rate(1/self.ctrl_invl)
        # [s] standard service timeout limit
        self.timeout = rospy.get_param('/controller/timeout')
        # radius fidelity of the smoothing
        self.rad_tolerance = rospy.get_param('/pure_pursuit/rad_tolerance')
        self.tolerance = rospy.get_param(
            '/pure_pursuit/tolerance')  # [m] tolerance
        self.alpha = rospy.get_param('/pure_pursuit/alpha')  # curve constant
        self.beta = rospy.get_param('/pure_pursuit/beta')  # curve constant
        self.w_base = rospy.get_param('/pure_pursuit/w_base')  # [m] wheel base
        # [m] space between injected points
        self.spacing = rospy.get_param('/pure_pursuit/spacing')
        self.old_nearest = None
        # [m] look-ahead distance
        self.lad = rospy.get_param('/pure_pursuit/lad')
        self.lfg = rospy.get_param('/pure_pursuit/lfg')  # look forward gain
        self.target_speed = rospy.get_param(
            '/pure_pursuit/target_speed')  # [m/s] velocity target
        self.filter_size = rospy.get_param(
            '/pure_pursuit/filter_size')  # size of the velocity filter
        kv = rospy.get_param('/pure_pursuit/kv')  # velocity constant
        ka = rospy.get_param('/pure_pursuit/ka')  # acceleration constant
        kp = rospy.get_param('/pure_pursuit/kp')  # pid proportional gain
        ki = rospy.get_param('/pure_pursuit/ki')  # integral gain
        kd = rospy.get_param('/pure_pursuit/kd')  # derivative gain
        # a hard cap on the pid error. if 0.0, no cap
        error_bound = rospy.get_param('/pure_pursuit/error_bound')
        self.map_metadata = OccupancyGrid()
        self.listener = tf.TransformListener()
        self.velocity_controller = PIDController()
        self.turningController = PIDController()
        self.lin_vel_filter = averagingFilter()
        self.ang_vel_filter = averagingFilter()
        self.velocity_controller.PIDController(kv, 0, ka, 0)
        self.turningController.PIDController(kp, ki, kd, error_bound)
        self.lin_vel_filter.averagingFilter(self.filter_size)
        self.ang_vel_filter.averagingFilter(self.filter_size)

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
        now = rospy.Time.now()
        self.listener.waitForTransform('/map', '/base_footprint', now, rospy.Duration(0, 100000000))
        cond = 0
        while cond == 0:
            try:
                rospy.loginfo('Waiting for the robot transform')
                (trans, rot) = self.listener.lookupTransform(
                    '/map', '/base_footprint', now)
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("Python Pure Pursuit Controller (Init) TF EXCEPTION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                cond = 0
                # rospy.sleep(0.1)
                self.error_handler()
                return
        roll, pitch, self.ctheta = euler_from_quaternion(rot)
        self.cx = trans[0]
        self.cy = trans[1]
        self.imu_sub = rospy.Subscriber(
            '/imu', Imu, self.update_imu)
        rospy.Timer(rospy.Duration(self.ctrl_invl), self.odometry)
        self.cmd_vel_pub = rospy.Publisher(
            '/cmd_vel', Twist, None, queue_size=1)
        self.acc_pub = rospy.Publisher('/acc', std_msgs.msg.Float32, None, queue_size=1)
        self.smoothed_pth_pub = rospy.Publisher(
            '/pure_persuit/smoothed_path', Path, queue_size=1)
        self.phase1_server = actionlib.SimpleActionServer(
            '/phase1', rbem.explorationAction, execute_cb=self.execute_path, auto_start=False)
        self.phase1_server.start()

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
        self.phase2_server = actionlib.SimpleActionServer(
            '/mnlc/navigate_to_origin', mb.MoveBaseAction)
        timeout_s = rospy.get_time()
        while not self.phase2_server.is_new_goal_available():
            if timeout_s + self.timeout < rospy.get_time():
                self.error_handler()
                break
            self.ctrl_rate.sleep()

        goal = self.phase2_server.current_goal.get_goal()
        self.phase2_server.start()
        self.phase2_server.register_goal_callback(self.final)

    def final(self):
        self.phase3_client = actionlib.SimpleActionClient(
            '/mnlc/navigation', mb.MoveBaseAction)
        self.phase3_client.wait_for_server(
            timeout=rospy.Duration(self.timeout))

    def execute_path(self, goal):
        start_time = rospy.get_time()
        poses = goal.path
        if len(poses) - 1 > 1:
            injected_path = self.inject_waypoints(poses)
            smoothed_path = self.path_smoothing(injected_path)
            self.smoothed_pth_pub.publish(smoothed_path)
            i = Path()
        rospy.loginfo("Executing Pure Persuit path following.")
        time_init = rospy.get_time()
        dx = smoothed_path.poses[7].pose.position.x - self.cx
        dy = smoothed_path.poses[7].pose.position.y - self.cy
        it = math.atan2(dy, dx)
        iq = quaternion_from_euler(0.0, 0.0, it, 'rxyz')
        self.turnTo(iq)
        last_index = len(smoothed_path.poses) - 1
        t_index, lf = self.search_target(smoothed_path.poses)
        # print("t_index is: ", t_index, " and last_index is: ", last_index)
        next_path_time = rospy.get_time()
        while last_index > t_index:
            if rospy.get_time() > start_time + 25.0 and distance.euclidean(self.vel[0], self.vel[1]) <= 0.000625:
                self.old_nearest = None
                self.stop()
                result = rbem.explorationResult()
                result.reached_frontier = False
                self.phase1_server.set_succeeded(result=result)
                return
            if rospy.get_time() > next_path_time:
                lin_v = self.velocity_controller.ComputeEffort(
                    (self.target_speed * 2) - distance.euclidean(self.vel[0], self.vel[1]))
                ang_v, t_index = self.pp_steering(smoothed_path.poses, t_index)
                self.send_speed(self.lin_vel_filter.average(lin_v), self.ang_vel_filter.average(ang_v))
                # print(rospy.get_time() - self.t)
                self.t = rospy.get_time()
                next_path_time = rospy.get_time() + self.ctrl_invl
                feedback = rbem.explorationFeedback()
                feedback.poses_left = len(smoothed_path.poses) - t_index
                self.phase1_server.publish_feedback(feedback=feedback)
        self.old_nearest = None
        self.stop()
        rospy.loginfo("Pure Persuit is stopping the Turtlebot.")
        result = rbem.explorationResult()
        result.reached_frontier = True
        self.phase1_server.set_succeeded(result=result)

    def inject_waypoints(self, poses):
        new_path = Path()
        for i in range(0, len(poses) - 1):
            vector = np.array([poses[i + 1].pose.position.x - poses[i].pose.position.x,
                              poses[i + 1].pose.position.y - poses[i].pose.position.y])
            mag = math.sqrt(math.pow(vector[0], 2) + math.pow(vector[1], 2))
            num_points = math.ceil(mag / self.spacing)
            vector = np.array(
                [vector[0] / mag, vector[1] / mag]) * self.spacing
            for j in range(int(num_points)):
                pose = PoseStamped()
                pose.header.frame_id = '/map'
                pose.header.stamp = rospy.Time(0)
                pose.pose.position.x = poses[i].pose.position.x + (
                    vector[0] * j)
                pose.pose.position.y = poses[i].pose.position.y + (
                    vector[1] * j)
                new_path.poses.append(pose)
        last_pose = PoseStamped()
        last_pose.header.frame_id = '/map'
        last_pose.header.stamp = rospy.Time(0)
        last_pose.pose.position.x = poses[len(
            poses) - 1].pose.position.x
        last_pose.pose.position.y = poses[len(
            poses) - 1].pose.position.y
        new_path.poses.append(last_pose)
        new_path.header.frame_id = '/map'
        new_path.header.stamp = rospy.Time(0)
        return new_path

    def path_smoothing(self, path):
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
            pose.header.frame_id = '/map'
            pose.header.stamp = rospy.Time(0)
            pose.pose.position.x = arr[0]
            pose.pose.position.y = arr[1]
            smoothed_path.poses.append(pose)
        smoothed_path.header.frame_id = '/map'
        smoothed_path.header.stamp = rospy.Time(0)
        return smoothed_path

    def search_target(self, poses):
        if self.old_nearest is None:
            dx = [self.rx - ip.pose.position.x for ip in poses]
            dy = [self.ry - ip.pose.position.y for ip in poses]
            dist = np.hypot(dx, dy)
            index = np.argmin(dist)
            self.old_nearest = index
        else:
            index = self.old_nearest
            dt_index = math.sqrt(pow(self.rx - poses[index].pose.position.x, 2) +
                                 pow(self.ry - poses[index].pose.position.y, 2))
            while 1:
                dn_index = math.sqrt(pow(self.rx - poses[index + 1].pose.position.x, 2) +
                                     pow(self.ry - poses[index + 1].pose.position.y, 2))
                if dt_index < dn_index:
                    break
                index = index + \
                    1 if (index + 1) < len(poses) else index
                dt_index = dn_index
            self.old_nearest = index
        lf = self.lfg * distance.euclidean(self.vel[0], self.vel[1]) + self.lad
        while lf > math.sqrt(pow(self.rx - poses[index].pose.position.x, 2) +
                             pow(self.ry - poses[index].pose.position.y, 2)):
            if index + 1 >= len(poses):
                break
            index += 1
        return index, lf

    def pp_steering(self, poses, prev_index):
        (index, lf) = self.search_target(poses)
        if prev_index >= index:
            index = prev_index
        if index < len(poses):
            tx = poses[index].pose.position.x
            ty = poses[index].pose.position.y
        else:
            tx = poses[-1].pose.position.x
            ty = poses[-1].pose.position.y
            index = len(poses) - 1
        sigma = math.atan2(2.0 * self.w_base * math.sin(math.atan2(ty -
                           self.ry, tx - self.rx) - self.ctheta) / lf, 1.0)
        return sigma, index

    def turnTo(self, orientation):
        # uses a simple PID control loop to turn in place to a desired heading
        self.turningController.reset()
        roll, pitch, yaw = euler_from_quaternion(orientation, 'rxyz')
        reachedHeading = False
        while (not reachedHeading and not rospy.is_shutdown()):
            error = yaw - self.ctheta
            while error > math.pi:
                error -= 2 * math.pi
            while error < - math.pi:
                error += 2 * math.pi
            if(abs(error) < 0.1):
                reachedHeading = True
                self.stop()
                break
            else:
                angularSpeed = self.turningController.ComputeEffort(error)
                self.send_speed(0.0, angularSpeed)
            self.ctrl_rate.sleep()

    def send_speed(self, linear_speed, angular_speed):
        msg_cmd_vel = Twist()
        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.angular.z = angular_speed
        self.cmd_vel_pub.publish(msg_cmd_vel)

    def odometry(self, tmp):
        now = rospy.Time.now()
        self.listener.waitForTransform('/odom', '/base_footprint', now, rospy.Duration(0, 100000000))
        cond = 0
        while cond == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(
                    '/map', '/base_footprint', now)
                (self.vel, ang) = self.listener.lookupTwistFull('/base_footprint', '/map',
                                                                '/base_footprint', (0, 0, 0), '/map', now, rospy.Duration(0.1))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("Pure Pursuit Controller (Odom update) TF EXCEPTION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                cond = 0
                # rospy.sleep(0.1)
        self.cx = trans[0]
        self.cy = trans[1]
        roll, pitch, self.ctheta = euler_from_quaternion(rot)
        self.rx = self.cx - ((self.w_base / 2) * math.cos(self.ctheta))
        self.ry = self.cy - ((self.w_base / 2) * math.sin(self.ctheta))

    def update_imu(self, msg):
        rospy.sleep(0.01)
        self.acc.data = distance.euclidean(
            msg.linear_acceleration.x, msg.linear_acceleration.y)
        self.acc_pub.publish(self.acc)        

    def stop(self):
        rospy.loginfo("Sending zero-velocity target to the Turtlebot.")
        self.send_speed(0.0, 0.0)

    def error_handler(self):
        self.error = True

    def run(self):
        while not rospy.is_shutdown() and self.error == False:
            rospy.spin()


if __name__ == '__main__':
    try:
        mnlc_pure_pursuit().run()
    except rospy.ROSInterruptException:
        pass
