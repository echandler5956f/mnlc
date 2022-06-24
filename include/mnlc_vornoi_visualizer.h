#ifndef mnlc_vornoi_visualizer_H
#define mnlc_vornoi_visualizer_H
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include "mersenne_twister.h"
#include <boost/foreach.hpp>
#include <nav_msgs/GetMap.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>
#include "ros/ros.h"
#include <map>

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

std::vector<cv::Point> rrt_detector_points;
nav_msgs::OccupancyGrid mapdata;
bool first_map = true;
_Float32 start_time;
_Float32 timeout;
cv::Mat map_img;
int state = 1;
cv::Mat image;
double res;
double gox;
double goy;
int height;
int width;
#endif