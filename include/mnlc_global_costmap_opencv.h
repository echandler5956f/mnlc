#ifndef mnlc_global_costmap_opencv_H
#define mnlc_global_costmap_opencv_H
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include "rbe3002/PointArray.h"
#include <opencv2/opencv.hpp>
#include <nav_msgs/GetMap.h>
#include <boost/foreach.hpp>
#include <std_srvs/Empty.h>
#include "ros/ros.h"
#include <map>

using namespace rbe3002;

std::unordered_map<int, int> unknown_indices;
std::vector<cv::Point2f> frontiers;
nav_msgs::OccupancyGrid mapdata;
cv::Mat image;
#endif