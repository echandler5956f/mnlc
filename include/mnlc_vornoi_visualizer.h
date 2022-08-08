#ifndef mnlc_vornoi_visualizer_H
#define mnlc_vornoi_visualizer_H
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <boost/foreach.hpp>
#include <nav_msgs/GetMap.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>
#include "ros/ros.h"
#include <map>

static void draw_voronoi(cv::Mat &img, cv::Subdiv2D &subdiv)
{
  std::vector<std::vector<cv::Point2f>> facets;
  std::vector<cv::Point2f> centers;
  subdiv.getVoronoiFacetList(std::vector<int>(), facets, centers);
  std::vector<cv::Point> ifacet;
  std::vector<std::vector<cv::Point>> ifacets(1);
  for (size_t i = 0; i < facets.size(); i++)
  {
    ifacet.resize(facets[i].size());
    for (size_t j = 0; j < facets[i].size(); j++)
      ifacet[j] = facets[i][j];
    cv::Scalar color;
    color[0] = rand() & 255;
    color[1] = rand() & 255;
    color[2] = rand() & 255;
    fillConvexPoly(img, ifacet, color, 8, 0);
    ifacets[0] = ifacet;
    polylines(img, ifacets, true, cv::Scalar(), 1, CV_AA, 0);
    circle(img, centers[i], 1, cv::Scalar(), CV_FILLED, CV_AA, 0);
  }
}

nav_msgs::OccupancyGrid mapdata;
cv::Rect rect(0, 0, 261, 261);
cv::Subdiv2D subdiv(rect);
bool first_map = true;
_Float32 start_time;
_Float32 timeout;
cv::Mat map_img;
cv::Mat image;
int state = 1;
double res;
double gox;
double goy;
int height;
int width;
#endif