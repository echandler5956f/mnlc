#!/usr/bin/env python

from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import transitions.extensions as sme
from rbe3002.msg import PointArray
from nav_msgs.srv import GetMap
import move_base_msgs.msg as mb
import rbe3002.msg as rbem
import std_srvs.srv
import std_msgs.msg
import numpy as np
import actionlib
import roslib
import random
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
        rospy.sleep(self.timeout * 5)
        # give gazebo a chance to warm up so rtabmap doesnt raise an error about not having a map
        rospy.loginfo("mnlc_controller node ready.")
        self.safe_start_phase_1()
        self.set_phase_1_goal()
        self.explore_map()

    def initialize_params(self):
        self.ctrl_invl = rospy.get_param('ctrl_invl', 0.01)  # in seconds
        self.ctrl_rate = rospy.Rate(1/self.ctrl_invl)
        self.obstacle_cost = rospy.get_param('obstacle_cost', 90)
        self.nodes_launched = rospy.get_param(
            'nodes_launched', 9) - 1  # dont count this node
        self.non_exploring_nodes = rospy.get_param(
            'non_exploring_nodes', 5) - 1  # again don't count this node
        self.timeout = rospy.get_param('timeout', 1.0)  # seconds
        self.exploration_scale_factor = rospy.get_param(
            'exploration_scale_factor', 1.0)
        self.machine = sme.GraphMachine.__init__(self, states=self.states, transitions=self.transitions,
                                                 initial=self.states[0], send_event=True, queued=True, ignore_invalid_triggers=True)
        self.initial_map_metadata = OccupancyGrid()
        self.local_costmap = OccupancyGrid()
        self.global_costmap = OccupancyGrid()
        self.odom_br = tf.TransformBroadcaster()
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.update_odom_tf)
        self.state_machine = rospy.Publisher(
            '/mnlc_state_machine', std_msgs.msg.Int8, queue_size=1)
        self.bounding_points_pub = rospy.Publisher(
            name='/bounding_points', data_class=PointStamped, latch=False, queue_size=5)
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
        # cmap = self.initial_map_metadata  # placeholder
        # if cmap.info.resolution == 0:
        #     rospy.logerr(
        #         "Local CSpace map has zero resolution. Exiting...")
        #     self.error_handler()
        #     return
        # cdata = cmap
        # np_data = np.array(cdata.data)
        # lowest_cost_indices = filter(lambda data: (
        #     data != -1 and data < self.obstacle_cost / 2.5), np_data)
        # if len(lowest_cost_indices) > 1:
        #     lowest_cost_index = random.choice(lowest_cost_indices)
        # # randomly choose a point that is within a certain range away from the obstacle cost to start our RRT search from
        # else:
        #     rospy.logerr(
        #         "The lowest cost grid in the local costmap is greater than the obstacle cost! Exiting...")
        #     self.error_handler()
        #     return
        # y = (int)(lowest_cost_index / cdata.info.width)
        # x = lowest_cost_index - (y * cdata.info.width)
        rospy.Service('/begin_phase1', std_srvs.srv.Empty,
                      self.count_operational_nodes)
        timeout_s = rospy.get_time()
        while self.nodes_responded != self.nodes_launched:
            # this doesn't work properly yet, so I am intentionally doing a boolean not equal to
            # which will return true as long as nodes_responded is not 0
            # with that said, the service does seem to properly transition the other nodes into the proper state
            if timeout_s + self.timeout < rospy.get_time():
                self.error_handler()
                break
            self.ctrl_rate.sleep()
        self.exploring_state()
        self.state_machine.publish(1)
        rospy.loginfo("Error checks complete. Getting goal for Phase 1.")
        # return (x, y)

    def set_phase_1_goal(self):
        self.phase1_client = actionlib.SimpleActionClient(
            '/phase1', rbem.explorationAction)
        self.phase1_client.wait_for_server(
            timeout=rospy.Duration(self.timeout))
        rest = Twist()
        map = self.initial_map_metadata
        # print(map.info)
        p0, p1, p2, p3, p4 = PointStamped(), PointStamped(
        ), PointStamped(), PointStamped(), PointStamped()
        p0.header.frame_id = p1.header.frame_id = p2.header.frame_id = p3.header.frame_id = p4.header.frame_id = '/map'
        p0.header.stamp = p1.header.stamp = p2.header.stamp = p3.header.stamp = p4.header.stamp = rospy.Time.now()
        p0.point.x = -(((map.info.width) * map.info.resolution) + map.info.origin.position.x + (map.info.resolution/2)) * self.exploration_scale_factor
        p0.point.y = (((map.info.height) * map.info.resolution) + map.info.origin.position.y + (map.info.resolution/2)) * self.exploration_scale_factor
        p1.point.x = (((map.info.width) * map.info.resolution) + map.info.origin.position.x + (map.info.resolution/2)) * self.exploration_scale_factor
        p1.point.y = (((map.info.height) * map.info.resolution) + map.info.origin.position.y + (map.info.resolution/2)) * self.exploration_scale_factor
        p2.point.x = (((map.info.width) * map.info.resolution) + map.info.origin.position.x + (map.info.resolution/2)) * self.exploration_scale_factor
        p2.point.y = -(((map.info.height) * map.info.resolution) + map.info.origin.position.y + (map.info.resolution/2)) * self.exploration_scale_factor
        p3.point.x = -(((map.info.width) * map.info.resolution) + map.info.origin.position.x + (map.info.resolution/2)) * self.exploration_scale_factor
        p3.point.y = -(((map.info.height) * map.info.resolution) + map.info.origin.position.y + (map.info.resolution/2)) * self.exploration_scale_factor
        p4.point.x = self.cx
        p4.point.y = self.cy
        bounding_points = PointArray()
        bounding_points.points.append(p4)
        bounding_points.points.append(p3)
        bounding_points.points.append(p2)
        bounding_points.points.append(p1)
        bounding_points.points.append(p0)
        self.phase1_goal = rbem.explorationGoal()
        self.phase1_goal.velocity = rest
        self.phase1_goal.points = bounding_points
        self.bounding_points_pub.publish(p0)
        # print(p4)
        self.bounding_points_pub.publish(p1)
        # print(p3)
        self.bounding_points_pub.publish(p2)
        # print(p2)
        self.bounding_points_pub.publish(p3)
        # print(p1)
        self.bounding_points_pub.publish(p4)
        # print(p0)
        self.phase1_client.send_goal(
            goal=self.phase1_goal, done_cb=self.re_init, active_cb=self.recovery_alert)

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
        self.cx = msg.pose.pose.position.x
        self.cy = msg.pose.pose.position.y
        self.odom_br.sendTransform(
            (msg.pose.pose.position.x, msg.pose.pose.position.y,
             msg.pose.pose.position.z),
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
            rospy.Time.now(), "/base_footprint", "/odom")

    def explore_map(self):
        while 1:
            rospy.sleep(0.1)

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
            goal=self.phase2_goal, done_cb=self.final, active_cb=self.recovery_alert)

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

    def convert_goal_topic_to_action(self, pose_stamped_msg):
        self.phase3_client.cancel_all_goals()
        self.phase3_goal.target_pose = pose_stamped_msg
        self.phase3_client.send_goal()

    def count_operational_nodes(self, tmp):
        self.nodes_responded += 1
        return std_srvs.srv.EmptyResponse()

    def recovery_alert(self):
        pass

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
