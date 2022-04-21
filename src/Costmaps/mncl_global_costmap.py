#!/usr/bin/env python

from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import std_msgs.msg
import numpy as np
import rospy
import time
import cv2


class MNCLGlobalCostmap(object):
# always generate costmap, regardless of whether we are mapping or navigating
    def __init__(self):
        rospy.init_node("MNCLGlobalCostmap")
        rospy.loginfo("Initializing Global Costmap Compute node.")
        self.sim = rospy.get_param('sim')
        self.desired_map_rate = rospy.get_param('desired_map_rate')
        self.padding = rospy.get_param('padding')
        self.current_mapdata = OccupancyGrid()
        self.c_space_pub = rospy.Publisher(
            '/MNCLGlobalCostmap/cspace', OccupancyGrid, queue_size=1)
        self.rtab_map_pub = rospy.Publisher(
            '/MNCLGlobalCostmap/map', OccupancyGrid, queue_size=1)
        self.new_map_written_pub = rospy.Publisher(
            '/MNCLGlobalCostmap/new_map_written', std_msgs.msg.String, queue_size=1)
        self.old_height, self.old_width, self.new_height, self.new_width = 0, 0, 0, 0
        self.first_map = True
        self.point_matrix = None
        self.counter = 0
        self.new_commands = False
        self.old_commands = self.new_commands
        rospy.sleep(1)
        rospy.loginfo("Global Costmap Compute node ready.")

    def pgmbwrite(self, img, filename, width, height, maxVal=255, magicNum='P5'):
        """  This function writes a numpy array to a Portable GrayMap Binary (PGMB) 
        image file. By default, header number P5 and max gray level 255 are 
        written. Width and height are same as the size of the given list.
        Line1 : MagicNum
        Line2 : Width Height
        Line3 : Max Gray level
        Image Row 1
        Image Row 2 etc. """
        # print(np.uint8(img).tolist())
        img = bytearray(np.uint8(img).tolist())
        with open(filename, 'wb') as f:
            f.write(magicNum + '\n')
            f.write(str(width) + ' ' + str(height) + '\n')
            f.write(str(maxVal) + '\n')
            f.write(img)
            f.close()
        self.new_map_written_pub.publish(filename)

    # def pgmawrite(self, img, filename, maxVal=255, magicNum='P2'):
    #       """  This function writes a numpy array to a Portable GrayMap (PGM)
    #       image file. By default, header number P2 and max gray level 255 are
    #       written. Width and height are same as the size of the given list.
    #       Line1 : MagicNum
    #       Line2 : Width Height
    #       Line3 : Max Gray level
    #       Image Row 1
    #       Image Row 2 etc. """
    #       img = np.int32(img).tolist()
    #       f = open(filename,'w')
    #       width = 0
    #       height = 0
    #       for row in img:
    #         height = height + 1
    #         width = len(row)
    #       f.write(magicNum + '\n')
    #       f.write(str(width) + ' ' + str(height) + '\n')
    #       f.write(str(maxVal) + '\n')
    #       for i in range(height):
    #         count = 1
    #         for j in range(width):
    #           f.write(str(img[i][j]) + ' ')
    #           if count >= 17:
    #             # No line should contain gt 70 chars (17*4=68)
    #             # Max three chars for pixel plus one space
    #             count = 1
    #             f.write('\n')
    #           else:
    #             count = count + 1
    #         f.write('\n')
    #       f.close()

    def request_map(self):
        # rospy.loginfo(
        #     "Requesting the latest map from RTabMap")
        rospy.wait_for_service('/rtabmap/get_map')
        # time_init = rospy.get_time()
        try:
            m = rospy.ServiceProxy('/rtabmap/get_map', GetMap)
            map = m()
        except rospy.ServiceException as e:
            print("Global Costmap service call failed: %s" % e)
        self.old_height = self.new_height
        self.old_width = self.new_width
        self.new_height = map.map.info.height
        self.new_width = map.map.info.width
        # rospy.loginfo("Map retrieved.")
        current_mapdata = map.map
        self.rtab_map_pub.publish(current_mapdata)
        mapdata_asarray = np.array(current_mapdata.data)
        self.pgmbwrite(mapdata_asarray, 'rtabmap.pgm',
                       self.new_width, self.new_height)
        # self.pgmawrite(mapdata_asarray, 'rtabmapAAA.pgm')
        # print("Requesting the map took: ",
        #       rospy.get_time() - time_init)
        return current_mapdata

    def commands(self, flag):
        self.new_commands = not self.new_commands

    def compare_commands(self):
        # if self.new_commands != self.old_commands:
        #     self.old_commands= self.new_commands
        #     return False
        return True


class OpenCVCostmap(MNCLGlobalCostmap):

    def __init__(self):
        super(OpenCVCostmap, self).__init__()
        self.new_commands_sub = rospy.Subscriber(
            "new_commands", std_msgs.msg.Bool, self.commands, queue_size=3)
        rospy.loginfo("Initializing OpenCVCostmap cspace compute")
        self.mncl_global_costmap()

    def mncl_global_costmap(self):
        next_time = time.time()
        while 1:
            if time.time() > next_time:
                # rospy.loginfo("Beginning OpenCV cspace compute.")
                current_mapdata = self.request_map()
                # current_mapdata = self.current_mapdata
                cspace = current_mapdata
                time_init = rospy.get_time()
                path = r'/home/quant/.ros/rtabmap.pgm'
                img = cv2.imread(path, -1)
                kernel1 = np.ones((self.padding, self.padding), np.float)
                # img_erosion = cv2.erode(img, kernel=kernel1, iterations=1)
                img_dilate = cv2.dilate(img, kernel=kernel1, iterations=1)
                # cv2.imwrite('/home/quant/.ros/dilated.pgm', img_dilate)
                gaussian_blur = cv2.GaussianBlur(
                    src=img_dilate, ksize=(9, 9), sigmaX=2, sigmaY=2)
                # flipped = np.flipud(gaussian_blur)
                # inverted = cv2.bitwise_not(gaussian_blur)
                norm_image = cv2.normalize(
                    gaussian_blur, None, alpha=0, beta=100, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
                # cv2.imshow("Final Map", norm_image)
                # cv2.waitKey()
                dataFromGridC = norm_image.flatten('C')
                dataC = tuple(np.array(dataFromGridC, dtype=int))
                cspace.header.stamp = rospy.Time.now()
                cspace.header.frame_id = 'map'
                cspace.data = dataC
                self.c_space_pub.publish(cspace)
                # rospy.loginfo("Finished OpenCV cspace compute.")
                # print("Calculating MNCLGlobalCostmap took: ",
                #       rospy.get_time() - time_init)
                next_time = time.time() + self.desired_map_rate

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    OpenCVCostmap().run()
