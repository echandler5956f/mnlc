#ifndef mnlc_meanshift_frontier_filter_H
#define mnlc_meanshift_frontier_filter_H
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include "rbe3002/PointArray.h"
#include "pybind11/pybind11.h"
#include <boost/foreach.hpp>
#include <nav_msgs/GetMap.h>
#include <pybind11/embed.h>
#include <std_srvs/Empty.h>
#include "pybind11/numpy.h"
#include <std_msgs/Int8.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <functional>
#include <ros/ros.h>
#include <algorithm>
#include "ros/ros.h"
#include <math.h>

using namespace rbe3002;
namespace py = pybind11;
using namespace py::literals;
using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

nav_msgs::OccupancyGrid mapdata;
_Float32 controller_start_time;
_Float32 next_time = 0.0;
_Float32 frontier_clear;
RowMatrixXd frontiers;
bool first_map = true;
_Float32 info_radius;
_Float32 start_time;
int obstacle_cost;
_Float32 timeout;
int state = 1;

#endif