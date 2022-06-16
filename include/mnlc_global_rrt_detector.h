#ifndef mnlc_global_rrt_detector_H
#define mnlc_global_rrt_detector_H
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

// rdm class, for gentaring random flot numbers
class rdm{
int i;
public:
rdm();
float randomize();
};

rdm::rdm() { i = time(0); }
float rdm::randomize()
{
  i = i + 1;
  srand(i);
  return float(rand()) / float(RAND_MAX);
}

nav_msgs::OccupancyGrid mapdata;
_Float32 start_time;
_Float32 controller_start_time;
_Float32 timeout;
_Float32 eta;
int obstacle_cost;
int state = 1;
// std::vector<geometry_msgs::Point> bounding_points;
visualization_msgs::Marker points;
visualization_msgs::Marker line;
#endif