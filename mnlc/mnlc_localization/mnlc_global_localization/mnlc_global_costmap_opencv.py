#!/usr/bin/env python
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import move_base_msgs.msg as mb
import rbe3002.msg as rbe
import std_srvs.srv
import numpy as np
import actionlib
import roslib
import rospy
import cv2

roslib.load_manifest('rbe3002')

class mnlc_global_costmap_opencv():

    def __init__(self):
        self.error = False
        rospy.loginfo("Initializing mnlc_global_costmap_opencv.")
        rospy.init_node("mnlc_global_costmap_opencv")
        self.initialize_params()
        rospy.sleep(self.timeout * 5) 
        # give gazebo a chance to warm up so rtabmap doesnt raise an error about not having a map        self.safe_start()
        rospy.loginfo("mnlc_global_costmap_opencv node ready.")

    def initialize_params(self):
        self.obstacle_cost = rospy.get_param('obstacle_cost', 90) # grid cost to be considered an obstacle
        self.ctrl_invl = rospy.get_param('ctrl_invl', 0.01)  # [s] control loop interval
        self.ctrl_rate = rospy.Rate(1/self.ctrl_invl)
        self.timeout = rospy.get_param('timeout', 1.0)  # [s] standard service timeout limit
        self.padding = rospy.get_param('padding', 4) # number of grid cells to pad the c_scpace with
        self.map = OccupancyGrid()
        
    def safe_start(self):
        rospy.wait_for_service('/rtabmap/get_map', timeout=rospy.Duration(self.timeout))
        try:
            m = rospy.ServiceProxy('/rtabmap/get_map', GetMap)
        except rospy.ServiceException as e:
            rospy.logerr("RTabMap Mappping service call failed: %s" % e)
            self.error_handler()
            return
        rospy.loginfo("RTabMap Mappping service call successful.")
        map = m()
        if map.map.info.resolution == 0 or map.map.header.seq <= 0:
            rospy.logerr(
                "RTabMap has zero resolution or incorrect sequence ID. Exiting...")
            self.error_handler()
            return
        self.initial_map_metadata = map.map
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
        self.c_space_pub = rospy.Publisher(
            '/mnlc_global_costmap_opencv/cspace', OccupancyGrid, queue_size=1)
        rospy.Timer(rospy.Duration(self.ctrl_invl), self.request_map)

    def request_map(self):
        rospy.wait_for_service('/rtabmap/get_map')
        try:
            m = rospy.ServiceProxy('/rtabmap/get_map', GetMap)
            map = m()
        except rospy.ServiceException as e:
            print("Global Costmap service call failed: %s" % e)
        self.old_height = self.new_height
        self.old_width = self.new_width
        self.new_height = map.map.info.height
        self.new_width = map.map.info.width
        current_mapdata = map.map
        mapdata_asarray = np.array(current_mapdata.data)
        img = bytearray(np.uint8(mapdata_asarray).tolist())
        with open('rtabmap.pgm', 'wb') as f:
            f.write('P5' + '\n')
            f.write(str(self.new_width) + ' ' + str(self.new_height) + '\n')
            f.write(str(255) + '\n')
            f.write(img)
            f.close()

    def calculate_global_costmap(self):
        cspace = OccupancyGrid()
        cspace.header.frame_id = 'map'
        kernel1 = np.ones((self.padding, self.padding), np.float)
        path = r'/home/quant/.ros/rtabmap.pgm'
        while 1:
            img = cv2.imread(path, -1)
            img_dilate = cv2.dilate(img, kernel=kernel1, iterations=1)
            gaussian_blur = cv2.GaussianBlur(
                src=img_dilate, ksize=(9, 9), sigmaX=2, sigmaY=2)
            norm_image = cv2.normalize(
                gaussian_blur, None, alpha=0, beta=100, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
            dataFromGridC = norm_image.flatten('C')
            dataC = tuple(np.array(dataFromGridC, dtype=int))
            cspace.header.stamp = rospy.Time.now()
            cspace.data = dataC
            self.c_space_pub.publish(cspace)

    def error_handler(self):
        self.error = True

    def run(self):
        while not rospy.is_shutdown() and self.error == False:
            rospy.spin()

if __name__ == '__main__':
    try:
        mnlc_global_costmap_opencv().run()
    except rospy.ROSInterruptException:
        pass