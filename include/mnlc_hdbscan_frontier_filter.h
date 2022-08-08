#ifndef mnlc_hdbscan_frontier_filter_H
#define mnlc_hdbscan_frontier_filter_H
#include "../src/hdbscan/Hdbscan/hdbscan.hpp"
#include <boost/range/algorithm/copy.hpp>
#include <geometry_msgs/PointStamped.h>
#include <boost/range/adaptor/map.hpp>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include "rbe3002/PointArray.h"
#include <boost/foreach.hpp>
#include <nav_msgs/GetMap.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>
#include <Eigen/Dense>
#include <functional>
#include <ros/ros.h>
#include <algorithm>
#include "ros/ros.h"
#include <iostream>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <math.h>

using namespace rbe3002;

class ClusterPoint
{
public:
    int group;
    double x;
    double y;
    double probability;

    ClusterPoint(int group, double x, double y, double probability)
    {
        this->group = group;
        this->x = x;
        this->y = y;
        this->probability = probability;
    }
    
        ClusterPoint()
    {
    }
};

std::vector<std::vector<double>> centroids;
nav_msgs::OccupancyGrid mapdata;
_Float32 controller_start_time;
_Float32 next_time = 0.0;
_Float32 frontier_clear;
bool first_map = true;
_Float32 info_radius;
_Float32 start_time;
int obstacle_cost;
_Float32 timeout;
Hdbscan hdbscan;
int state = 1;
double res;
double gox;
double goy;
int height;
int width;

#endif