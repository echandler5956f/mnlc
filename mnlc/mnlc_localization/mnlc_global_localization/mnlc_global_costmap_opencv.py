#!/usr/bin/env python

from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import std_srvs.srv
import numpy as np
import roslib
import rospy
import cv2
import tf

roslib.load_manifest('rbe3002')

class mnlc_global_costmap_opencv():

    def __init__(self):
        self.error = False
        self.flag = 0
        rospy.loginfo("Initializing mnlc_global_costmap_opencv.")
        rospy.init_node("mnlc_global_costmap_opencv")
        self.initialize_params()
        rospy.sleep(self.start_time)
        # give gazebo a chance to warm up so rtabmap doesnt raise an error about not having a map
        rospy.loginfo("mnlc_global_costmap_opencv node ready.")
        self.safe_start()

    def initialize_params(self):
        self.start_time = rospy.get_param('/global_costmap/start_time')
        # grid cost to be considered an obstacle
        # [s] standard service timeout limit
        self.timeout = rospy.get_param('/controller/timeout')
        # number of grid cells to pad the c_scpace with
        self.padding = rospy.get_param('/global_costmap/padding')
        self.first_map = True
        self.map = OccupancyGrid()
        self.listener = tf.TransformListener()

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
        initial_map_metadata = map.map
        self.width = initial_map_metadata.info.width
        self.height = initial_map_metadata.info.height
        self.resolution = initial_map_metadata.info.resolution
        self.gox = initial_map_metadata.info.origin.position.x
        self.goy = initial_map_metadata.info.origin.position.y
        self.rtab_map_pub = rospy.Publisher(
            '/latest_map', OccupancyGrid, queue_size=1)
        self.c_space_pub = rospy.Publisher(
            '/mnlc_global_costmap_opencv/cspace', OccupancyGrid, queue_size=1)
        self.c_space_pub.publish(initial_map_metadata)
        self.rtab_map_sub = rospy.Subscriber(
            '/map', OccupancyGrid, self.update_map, queue_size=1)
        rospy.sleep(self.timeout)
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
        self.listener.waitForTransform(
            '/odom', '/base_footprint', rospy.Time(0), timeout=rospy.Duration(self.timeout))
        # allocate a thread for publishing
        self.calculate_global_costmap()

    def calculate_global_costmap(self):
        cspace = OccupancyGrid()
        cspace.header.frame_id = '/map'
        cspace.info.origin.position.x = self.gox
        cspace.info.origin.position.y = self.goy
        cspace.info.resolution = self.resolution
        cspace.info.height = self.height
        cspace.info.width = self.width
        kernel1 = np.ones((self.padding, self.padding), np.float)
        while not rospy.is_shutdown():
            # time_init = rospy.get_time()
            img = self.img
            unkown_indices = self.unkown_indices
            img_dilate = cv2.dilate(img, kernel=kernel1, iterations=1)
            gaussian_blur = cv2.GaussianBlur(
                src=img_dilate, ksize=(7, 7), sigmaX=2, sigmaY=2)
            norm_image = cv2.normalize(
                gaussian_blur, None, alpha=0, beta=100, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
            dataFromGridC = norm_image.flatten('C')
            dataFromGridC[unkown_indices] = -1
            cspace.header.stamp = rospy.Time.now()
            cspace.data = dataFromGridC
            self.c_space_pub.publish(cspace)
            # time_end = rospy.get_time()
            # print("Calculating Global CSpace took: ", time_end - time_init, ".")

    def update_map(self, map):
        path = r'/home/quant/.ros/global_costmap.pgm'
        self.rtab_map_pub.publish(map)
        mapdata_asarray = np.array(object=map.data, copy=False)
        self.unkown_indices = np.where(mapdata_asarray == -1)
        mapdata_asarray[mapdata_asarray == -1] = 0
        byte_array = bytearray(np.uint8(mapdata_asarray).tolist())
        with open('global_costmap.pgm', 'wb') as f:
            f.write('P5' + '\n')
            f.write(str(self.width) + ' ' +
                    str(self.height) + '\n')
            f.write(str(255) + '\n')
            f.write(byte_array)
            f.close()
        self.img = cv2.imread(path, -1)

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