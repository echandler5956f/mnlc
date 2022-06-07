#ifndef mnlc_hdbscan_frontier_filter_H
#define mnlc_hdbscan_frontier_filter_H
#include"../../hdbscan-cpp/HDBSCAN-CPP/Hdbscan/hdbscan.hpp"
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
#include <Eigen/Dense>
#include <functional>
#include <ros/ros.h>
#include <algorithm>
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <math.h>

using namespace rbe3002;
// using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

// // define the format you want, you only need one instance of this...
// // see https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
// const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

// // writing functions taking Eigen types as parameters, 
// // see https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
// template <typename Derived>
// void writeToCSVfile(std::string name, const Eigen::MatrixBase<Derived>& matrix)
// {
//     std::ofstream file(name.c_str());
//     file << matrix.format(CSVFormat);
//     // file.close() is not necessary, 
//     // desctructur closes file, see https://en.cppreference.com/w/cpp/io/basic_ofstream
// }

// std::string csvName = "Frontiers.csv";
std::vector<std::vector<double>> frontiers;
nav_msgs::OccupancyGrid mapdata;
_Float32 controller_start_time;
_Float32 next_time = 0.0;
_Float32 frontier_clear;
// RowMatrixXd frontiers;
bool first_map = true;
_Float32 info_radius;
_Float32 start_time;
int obstacle_cost;
_Float32 timeout;
int state = 1;

#endif