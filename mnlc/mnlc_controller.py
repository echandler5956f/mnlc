#!/usr/bin/env python

from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap, GetPlan
import transitions.extensions as sme
from rbe3002.msg import PointArray
import move_base_msgs.msg as mb
import rbe3002.msg as rbem
import std_srvs.srv
import std_msgs.msg
import actionlib
import roslib
import rospy
import tf

roslib.load_manifest('rbe3002')


class mnlc_controller(sme.GraphMachine):
    states = ['Start', 'phase_1', 'phase_2', 'phase_3']
    transitions = [{'trigger': 'exploring_state', 'source': 'Start', 'dest': 'phase_1'},
                   {'trigger': 'return_to_origin_state',
                       'source': 'phase_1', 'dest': 'phase_2'},
                   {'trigger': 'go_to_point_state', 'source': 'phase_2', 'dest': 'phase_3'}]

    def __init__(self):
        self.error = False
        rospy.loginfo("Initializing mnlc_controller.")
        rospy.init_node("mnlc_controller")
        self.initialize_params()
        rospy.sleep(self.start_time)
        # give gazebo a chance to warm up so rtabmap doesnt raise an error about not having a map
        rospy.loginfo("mnlc_controller node ready.")
        self.safe_start_phase_1()

    def initialize_params(self):
        self.recieved_first_frontier = False
        self.start_time = rospy.get_param('/controller/start_time')
        self.ctrl_invl = rospy.get_param('/controller/ctrl_invl')  # in seconds
        self.ctrl_rate = rospy.Rate(1/self.ctrl_invl)
        self.obstacle_cost = rospy.get_param('/controller/obstacle_cost')
        self.nodes_launched = rospy.get_param(
            '/controller/nodes_launched')  # dont count this node
        self.non_exploring_nodes = rospy.get_param(
            '/controller/non_exploring_nodes')  # again don't count this node
        self.timeout = rospy.get_param('/controller/timeout')  # seconds
        self.exploration_scale_factor = rospy.get_param(
            '/controller/exploration_scale_factor')
        self.machine = sme.GraphMachine.__init__(self, states=self.states, transitions=self.transitions,
                                                 initial=self.states[0], send_event=True, queued=True, ignore_invalid_triggers=True)
        self.initial_map_metadata = OccupancyGrid()
        self.local_costmap = OccupancyGrid()
        self.global_costmap = OccupancyGrid()
        self.odom_br = tf.TransformBroadcaster()
        self.poseL = tf.TransformListener()
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.update_odom_tf)
        self.state_machine = rospy.Publisher(
            '/mnlc_state_machine', std_msgs.msg.Int8, queue_size=1)
        self.bounding_points_pub = rospy.Publisher(
            name='/bounding_points', data_class=PointStamped, latch=False, queue_size=5)
        self.goal_subscriber = rospy.Subscriber(
            '/move_base_simple/goal', PoseStamped, self.update_goal, queue_size=1)
        self.nodes_responded = 0

    def safe_start_phase_1(self):
        rospy.wait_for_service(
            service='/rtabmap/set_mode_mapping', timeout=rospy.Duration(self.timeout))
        try:
            rospy.ServiceProxy(name='/rtabmap/set_mode_mapping',
                               service_class=std_srvs.srv.Empty)
        except rospy.ServiceException as e:
            rospy.logerr("RTabMap Mappping service call failed: %s" % e)
            self.error_handler()
            return
        rospy.wait_for_service('/rtabmap/get_map')
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
        self.initial_map_metadata = map.map
        self.local_c_space_sub = rospy.Subscriber(
            '/mnlc_local_costmap_opencv/cspace', OccupancyGrid, self.update_local_costmap, queue_size=1)
        self.global_c_space_sub = rospy.Subscriber(
            '/mnlc_global_costmap_opencv/cspace', OccupancyGrid, self.update_global_costmap, queue_size=1)
        rospy.Service('/begin_phase1', std_srvs.srv.Empty,
                      self.count_operational_nodes)
        self.exploring_state()
        self.state_machine.publish(1)
        rospy.loginfo("Error checks complete. Beginning Phase 1.")
        self.phase_1()

    def phase_1(self):
        self.phase1_client = actionlib.SimpleActionClient(
            '/phase1', rbem.explorationAction)
        self.phase1_client.wait_for_server()
        rest = Twist()
        map = self.initial_map_metadata
        p0, p1, p2, p3, p4 = PointStamped(), PointStamped(
        ), PointStamped(), PointStamped(), PointStamped()
        p0.header.frame_id = p1.header.frame_id = p2.header.frame_id = p3.header.frame_id = p4.header.frame_id = '/map'
        p0.header.stamp = p1.header.stamp = p2.header.stamp = p3.header.stamp = p4.header.stamp = rospy.Time.now()
        p0.point.x = -(((map.info.width) * map.info.resolution) + map.info.origin.position.x +
                       (map.info.resolution/2)) * self.exploration_scale_factor
        p0.point.y = (((map.info.height) * map.info.resolution) + map.info.origin.position.y +
                      (map.info.resolution/2)) * self.exploration_scale_factor
        p1.point.x = (((map.info.width) * map.info.resolution) + map.info.origin.position.x +
                      (map.info.resolution/2)) * self.exploration_scale_factor
        p1.point.y = (((map.info.height) * map.info.resolution) + map.info.origin.position.y +
                      (map.info.resolution/2)) * self.exploration_scale_factor
        p2.point.x = (((map.info.width) * map.info.resolution) + map.info.origin.position.x +
                      (map.info.resolution/2)) * self.exploration_scale_factor
        p2.point.y = -(((map.info.height) * map.info.resolution) + map.info.origin.position.y +
                       (map.info.resolution/2)) * self.exploration_scale_factor
        p3.point.x = -(((map.info.width) * map.info.resolution) + map.info.origin.position.x +
                       (map.info.resolution/2)) * self.exploration_scale_factor
        p3.point.y = -(((map.info.height) * map.info.resolution) + map.info.origin.position.y +
                       (map.info.resolution/2)) * self.exploration_scale_factor
        p4.point.x = self.cx
        p4.point.y = self.cy
        self.bounding_points_pub.publish(p0)
        self.bounding_points_pub.publish(p1)
        self.bounding_points_pub.publish(p2)
        self.bounding_points_pub.publish(p3)
        self.bounding_points_pub.publish(p4)
        recieved_first_frontier = self.recieved_first_frontier
        while recieved_first_frontier == False:
            print("Waiting for first frontier")
            rospy.sleep(1)
            recieved_first_frontier = self.recieved_first_frontier
        while 1:
            print("Navigating to frontier")
            frontier_goal = self.goal_pose
            rospy.wait_for_service('/plan_path')
            try:
                path_srv = rospy.ServiceProxy('/plan_path', GetPlan)
                start_pose = PoseStamped()
                start_pose.pose.position.x = self.cx
                start_pose.pose.position.y = self.cy
                path = path_srv(start_pose, frontier_goal, 1.0)
            except rospy.ServiceException as e:
                rospy.logerr("Path Planning service call failed: %s." % e)
            if len(path.plan.poses) <= 1:
                rospy.logwarn(
                    "Controller recieved information indicating that the planner cannot plan a path. Sending new frontier.")
                pass
            else:
                goal = rbem.explorationGoal()
                goal.path = path.plan.poses
                self.phase1_client.send_goal(
                    goal=goal, feedback_cb=self.feedback)
                self.phase1_client.wait_for_result()
                result = self.phase1_client.get_result()
                print(result)
                rospy.sleep(0.1)

    def feedback(self, feedback):
        print("Feedback: ", feedback)

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

    def update_odom_tf(self, msg):
        self.odom_br.sendTransform(
            (msg.pose.pose.position.x, msg.pose.pose.position.y,
             msg.pose.pose.position.z),
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
            rospy.Time.now(), "/base_footprint", "/odom")
        (tr, rot) = self.poseL.lookupTransform(
            "/odom", "/base_footprint", rospy.Time(0))
        self.cx = tr[0]
        self.cy = tr[1]

    def re_init(self):
        rospy.loginfo(
            "Reinitializing mnlc_controller for navigation and localization.")
        self.reinitialize_params()
        self.safe_start_phase_2()
        self.set_phase_2_goal()
        rospy.loginfo("mnlc_controller reinitialized.")

    def reinitialize_params(self):
        rospy.set_param('/rtabmap/Mem/IncrementalMemory', "false")
        rospy.set_param('/rtabmap/map_always_update', "false")
        rospy.set_param('/rtabmap/map_empty_ray_tracing', "false")

    def safe_start_phase_2(self):
        rospy.wait_for_service(
            service='/rtabmap/backup', timeout=rospy.Duration(self.timeout))
        try:
            rospy.ServiceProxy(name='/rtabmap/backup',
                               service_class=std_srvs.srv.Empty)
        except rospy.ServiceException as e:
            rospy.logerr("RTabMap map backup service call failed: %s" % e)
            self.error_handler()
            return
        rospy.wait_for_service(
            service='/rtabmap/set_mode_localization ', timeout=rospy.Duration(self.timeout))
        try:
            rospy.ServiceProxy(name='/rtabmap/set_mode_localization ',
                               service_class=std_srvs.srv.Empty)
        except rospy.ServiceException as e:
            rospy.logerr("RTabMap Localization service call failed: %s" % e)
            self.error_handler()
            return
        rospy.wait_for_service(
            service='/rtabmap/update_parameters ', timeout=rospy.Duration(self.timeout))
        try:
            rospy.ServiceProxy(name='/rtabmap/update_parameters ',
                               service_class=std_srvs.srv.Empty)
        except rospy.ServiceException as e:
            rospy.logerr(
                "RTabMap update_parameters service call failed: %s" % e)
            self.error_handler()
            return
        self.nodes_responded = 0
        # make sure we didn't lose any nodes over exploration
        rospy.Service('/round_up_nodes', std_srvs.srv.Empty,
                      self.count_operational_nodes, error_handler=self.error_handler)
        timeout_s = rospy.get_time()
        while self.nodes_responded != self.nodes_launched:
            if timeout_s + self.timeout < rospy.get_time():
                self.error_handler()
                rospy.logerr(
                    "We do not have all the nodes we started with! Detecting  nodes when we should have .")
                break
            self.ctrl_rate.sleep()
        self.nodes_responded = 0
        rospy.Service('/begin_phase2', std_srvs.srv.Empty,
                      self.count_operational_nodes, error_handler=self.error_handler)
        # all unecessary nodes are now being shut down
        timeout_t = rospy.get_time()
        while self.nodes_responded != self.non_exploring_nodes:
            if timeout_t + self.timeout < rospy.get_time():
                self.error_handler()
                rospy.logerr(
                    "We should not have any more nodes than what is essential to navigate, but it appears we do. Detectingnodes when we should have .")
                break
            self.ctrl_rate.sleep()
        self.phase2_client = actionlib.SimpleActionClient(
            '/mnlc/navigate_to_origin', mb.MoveBaseAction)
        self.phase2_client.wait_for_server(
            timeout=rospy.Duration(self.timeout))
        self.return_to_origin_state()
        self.state_machine.publish(2)
        rospy.loginfo("Error checks complete. Getting goal for Phase 2.")

    def set_phase_2_goal(self):
        self.phase2_goal = mb.MoveBaseGoal()
        self.phase2_goal.target_pose.pose.position.x = 0.0
        self.phase2_goal.target_pose.pose.position.y = 0.0
        self.phase2_goal.target_pose.pose.orientation.z = 0.0
        self.phase2_goal.target_pose.pose.orientation.w = 1.0
        self.phase2_goal.target_pose.header.frame_id = '/map'
        self.phase2_goal.target_pose.header.stamp = rospy.Time.now()
        self.phase2_client.send_goal(
            goal=self.phase2_goal, done_cb=self.final)

    def final(self):
        self.phase3_client = actionlib.SimpleActionClient(
            '/mnlc/navigation', mb.MoveBaseAction)
        self.phase3_client.wait_for_server(
            timeout=rospy.Duration(self.timeout))
        self.phase3_goal = mb.MoveBaseGoal()
        self.simple_goal = rospy.Subscriber(
            '/move_base_simple/goal', PoseStamped, self.convert_goal_topic_to_action, queue_size=1)
        self.go_to_point_state()
        self.state_machine.publish(2)
        rospy.loginfo(
            "Remapping '/move_base_simple/goal' topic to '/mnlc/navigation' action client")

    def update_goal(self, pose_stamped_msg):
        self.recieved_first_frontier = True
        self.goal_pose = pose_stamped_msg

    def count_operational_nodes(self, tmp):
        self.nodes_responded += 1
        return std_srvs.srv.EmptyResponse()

    def error_handler(self):
        self.error = True

    def run(self):
        while not rospy.is_shutdown() and self.error == False:
            rospy.spin()

if __name__ == '__main__':
    try:
        mnlc_controller().run()
    except rospy.ROSInterruptException:
        pass
