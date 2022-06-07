#ifndef mnlc_local_rrt_detector_H
#define mnlc_local_rrt_detector_H
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include "mersenne_twister.h"
#include <boost/foreach.hpp>
#include <nav_msgs/GetMap.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>
#include <functional>
#include <ros/ros.h>
#include <algorithm>
#include "ros/ros.h"
#include <math.h>

std::vector<geometry_msgs::Point> bounding_points;
nav_msgs::OccupancyGrid mapdata;
_Float32 controller_start_time;
_Float32 start_time;
int obstacle_cost;
_Float32 timeout;
_Float32 eta;
int state = 1;

#endif