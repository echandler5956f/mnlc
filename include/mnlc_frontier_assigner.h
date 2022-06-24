#ifndef mnlc_frontier_assigner_H
#define mnlc_frontier_assigner_H
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include "rbe3002/PointArray.h"
#include <opencv2/opencv.hpp>
#include <boost/foreach.hpp>
#include <nav_msgs/GetMap.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>
#include <functional>
#include <ros/ros.h>
#include <algorithm>
#include "ros/ros.h"
#include <iostream>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <math.h>
#include <map>

using namespace rbe3002;

std::vector<std::vector<double>> filtered_frontiers;
std::unordered_map<int, int> visited_indices;
nav_msgs::OccupancyGrid mapdata;
_Float32 controller_start_time;
_Float32 hysteresis_radius;
_Float32 hysteresis_gain;
_Float32 next_time = 0.0;
_Float32 info_multiplier;
_Float32 frontier_clear;
bool first_map = true;
_Float32 info_radius;
_Float32 start_time;
int obstacle_cost;
_Float32 timeout;
int state = 1;
long length;
double res;
double gox;
double goy;
int height;
int width;
double cx;
double cy;

#endif