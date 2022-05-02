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
import math
import cv2
import tf

roslib.load_manifest('rbe3002')


class mnlc_local_costmap_opencv():

    def __init__(self):
        self.error = False
        rospy.loginfo("Initializing mnlc_local_costmap_opencv.")
        rospy.init_node("mnlc_local_costmap_opencv")
        self.initialize_params()
        rospy.sleep(self.start_time)
        # give gazebo a chance to warm up so rtabmap doesnt raise an error about not having a map
        rospy.loginfo("mnlc_local_costmap_opencv node ready.")
        self.safe_start()

    def initialize_params(self):
        self.start_time = rospy.get_param('/local_costmap/start_time')
        # [s] standard service timeout limit
        self.timeout = rospy.get_param('/controller/timeout')
        # number of grid cells to pad the c_scpace with
        self.padding = rospy.get_param('/local_costmap/padding', 4)
        # [m] the width of the local costmap
        self.local_width = rospy.get_param('/local_costmap/local_width', 1)
        # [m] the height of the local costmap
        self.local_height = rospy.get_param('/local_costmap/local_height', 1)
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
        self.initial_map_metadata = map.map
        self.c_space_pub = rospy.Publisher(
            '/mnlc_local_costmap_opencv/cspace', OccupancyGrid, queue_size=1)
        self.c_space_pub.publish(self.initial_map_metadata)
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
        self.calculate_local_costmap()

    def calculate_local_costmap(self):
        path = r'/home/quant/.ros/local_costmap.pgm'
        kernel1 = np.ones((self.padding, self.padding), np.float)
        global_resolution = self.initial_map_metadata.info.resolution
        global_grid_height = self.initial_map_metadata.info.height
        global_grid_width = self.initial_map_metadata.info.width
        local_grid_height = int(
            math.ceil(self.local_height / global_resolution))
        local_grid_width = int(math.ceil(self.local_width / global_resolution))
        gox = self.initial_map_metadata.info.origin.position.x
        goy = self.initial_map_metadata.info.origin.position.y
        cspace = OccupancyGrid()
        cspace.header.frame_id = '/map'
        cspace.info.resolution = global_resolution
        cspace.info.height = local_grid_height
        cspace.info.width = local_grid_width
        rtab_map_pub = rospy.Publisher(
            '/latest_map', OccupancyGrid, queue_size=1)
        rtab_map_pub.publish(self.initial_map_metadata)
        while 1:
            time_init = rospy.get_time()
            rospy.wait_for_service('/rtabmap/get_map')
            try:
                m = rospy.ServiceProxy('/rtabmap/get_map', GetMap)
                map = m()
            except rospy.ServiceException as e:
                rospy.logerr("Local Costmap service call failed: %s" % e)
                self.error_handler()
                return
            current_mapdata = map.map
            rtab_map_pub.publish(current_mapdata)
            mapdata_asarray = np.array(current_mapdata.data)
            flag = 0
            while flag == 0:
                try:
                    (trans, rot) = self.listener.lookupTransform(
                        "odom", "base_footprint", rospy.Time(0))
                    flag = 1
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr("Tf exception")
                    self.error_handler()
                    return
            time_init = rospy.get_time()
            x = int(math.floor((trans[0] - gox) / global_resolution))
            y = int(math.floor((trans[1] - goy) / global_resolution))
            immediate_vincinity = []
            unkown_indices = []
            for i in range(int(x) - int(math.floor(local_grid_width/2)), int(x) + int(math.floor(local_grid_width/2))):
                for j in range(int(y) - int(math.floor(local_grid_height/2)), int(y) + int(math.floor(local_grid_height/2))):
                    index = j + (i * global_grid_width)
                    immediate_vincinity.append(current_mapdata.data[index])
                    if mapdata_asarray.data[index] == -1:
                        unkown_indices.append(index)
            ba = bytearray(np.uint8(immediate_vincinity).tolist())
            with open('local_costmap.pgm', 'wb') as f:
                f.write('P5' + '\n')
                f.write(str(local_grid_width) + ' ' +
                        str(local_grid_height) + '\n')
                f.write(str(255) + '\n')
                f.write(ba)
                f.close()
            img = cv2.imread(path, -1)
            img_dilate = cv2.dilate(img, kernel=kernel1, iterations=1)
            gaussian_blur = cv2.GaussianBlur(
                src=img_dilate, ksize=(9, 9), sigmaX=2, sigmaY=2)
            norm_image = cv2.normalize(
                gaussian_blur, None, alpha=0, beta=100, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
            dataFromGridC = norm_image.flatten('C')
            for k in range(len(unkown_indices) - 1):
                row = int(unkown_indices[j] % global_grid_width)
                column = int(math.floor(
                    unkown_indices[k] / global_grid_width))
                dataFromGridC[int(math.floor(
                    row * global_grid_height) + column)] = -1
            dataC = tuple(np.array(dataFromGridC, dtype=int))
            cspace.info.origin.position.x = trans[0] - \
                (local_grid_width * global_resolution/2)
            cspace.info.origin.position.y = trans[1] - \
                (local_grid_height * global_resolution/2)
            cspace.header.stamp = rospy.Time.now()
            cspace.data = dataC
            self.c_space_pub.publish(cspace)
            time_end = rospy.get_time()
            # print("Calculating Local CSpace took: ", time_end - time_init, ".")

    def error_handler(self):
        self.error = True

    def run(self):
        while not rospy.is_shutdown() and self.error == False:
            rospy.spin()


if __name__ == '__main__':
    try:
        mnlc_local_costmap_opencv().run()
    except rospy.ROSInterruptException:
        pass
