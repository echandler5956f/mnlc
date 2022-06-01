#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/GetMap.h>
#include <boost/foreach.hpp>
#include <std_srvs/Empty.h>
#include "ros/ros.h"
#include <fstream>
#include <map>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <sstream>
#include "nav_msgs/MapMetaData.h"
#include <iostream>
#include <bits/stdc++.h>

std::unordered_map<int, int> unknown_indices;
nav_msgs::OccupancyGrid mapdata;
cv::Mat image;

void update_map(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
  ros::Time time_init = ros::Time::now();
  std::unordered_map<int, int> uk;
  double ti = time_init.toSec();
  nav_msgs::OccupancyGrid m = *map;
  mapdata = m;
  int index = 0;
  int row = 0;
  int col = 0;
  BOOST_FOREACH (int8_t &datapoint, m.data)
  {
    if (datapoint == -1)
    {

      uk.emplace(index, 0);
      m.data[index] = 0;
    }
    row = index / m.info.width;
    col = index % m.info.width;
    // printf("row: %d \t", row);
    // printf("col: %d \n", col);
    unsigned char ch = (unsigned char)m.data[index];
    image.ptr<unsigned char>(col)[row] = ch;
    index++;
  }
  // for (auto itr = uk.begin(); itr != uk.end(); itr++)
  //       std::cout << itr->first << "\t" << itr->second << std::endl;
  unknown_indices = uk;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mnlc_global_costmap_opencv");
  ros::NodeHandle n;
  double start_time;
  n.getParam("/global_costmap/start_time", start_time);
  double timeout;
  n.getParam("/controller/timeout", timeout);
  int padding;
  n.getParam("/global_costmap/padding", padding);
  bool first_map = true;
  nav_msgs::OccupancyGrid map;
  tf::TransformListener listener;
  ros::Time st;
  st.fromSec(start_time);
  ros::Time::sleepUntil(st);
  ros::service::waitForService("/rtabmap/get_map", ros::Duration(timeout));
  ros::ServiceClient client1 = n.serviceClient<const nav_msgs::GetMap>("/rtabmap/get_map");
  nav_msgs::GetMap srv1;
  if (client1.call(srv1))
  {
    ROS_INFO("RTabMap Mapping service call successful.");
  }
  else
  {
    ROS_ERROR("RTabMap Mapping service call failed.");
    exit(0);
  }
  if (srv1.response.map.info.resolution == 0)
  {
    ROS_ERROR("RTabMap has zero resolution. Exiting...");
    exit(0);
  }
  nav_msgs::OccupancyGrid initial_map_metadata = srv1.response.map;
  int width = initial_map_metadata.info.width;
  int height = initial_map_metadata.info.height;
  double resolution = initial_map_metadata.info.resolution;
  double gox = initial_map_metadata.info.origin.position.x;
  double goy = initial_map_metadata.info.origin.position.y;
  ros::Publisher rtab_map_pub = n.advertise<nav_msgs::OccupancyGrid>("/latest_map", 1);
  ros::Publisher cspace_pub = n.advertise<nav_msgs::OccupancyGrid>("/mnlc_global_costmap_opencv/cspace", 1);
  cspace_pub.publish(initial_map_metadata);
  image = cv::Mat::ones(width, height, CV_8UC1);
  ros::Subscriber rtabmap_sub = n.subscribe("/map", 10, update_map);
  ros::Time sr;
  sr.fromSec(timeout);
  ros::Time::sleepUntil(sr);
  ros::spinOnce();
  ros::service::waitForService("/begin_phase1", ros::Duration(timeout));
  ros::ServiceClient client2 = n.serviceClient<const std_srvs::Empty>("/begin_phase1");
  std_srvs::Empty srv2;
  if (client2.call(srv2))
  {
    ROS_INFO("Begin phase1 service call successful.");
  }
  else
  {
    ROS_ERROR("Begin phase1 service call failed:");
    exit(0);
  }
  listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(timeout));
  nav_msgs::OccupancyGrid cspace;
  cspace.header.frame_id = "/map";
  cspace.header.stamp = ros::Time::now();
  cspace.info.width = width;
  cspace.info.height = height;
  cspace.info.resolution = resolution;
  cspace.info.origin.position.x = gox;
  cspace.info.origin.position.y = goy;
  int dilation_type = cv::MORPH_RECT;
  int dilation_size = padding;
  cv::Mat kernel = cv::getStructuringElement(dilation_type,
                                             cv::Size(dilation_size, dilation_size),
                                             cv::Point((int)(dilation_size / 2), (int)(dilation_size / 2)));
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    std::unordered_map<int, int> uk = unknown_indices;
    cv::Mat img_dilate;
    cv::Mat img = image;
    cv::dilate(img, img_dilate, kernel);
    cv::Mat gaussian_bur;
    cv::Size size;
    size.width, size.height = 7;
    cv::GaussianBlur(img_dilate, gaussian_bur, size, 1.0, 1.0);
    cv::Mat norm_image;
    cv::normalize(gaussian_bur, norm_image, 0, 100, cv::NormTypes::NORM_MINMAX);
    std::vector<int8_t, std::allocator<int8_t>> data_from_grid_c;
    for (int i = 0; i < height; i++)
    {
      for (int j = 0; j < width; j++)
      {
        auto it = uk.find(i * width + j);
        if (it != uk.end())
        {
          data_from_grid_c.push_back(-1);
        }
        else
        {
          data_from_grid_c.push_back(norm_image.ptr<unsigned char>(j)[i]);
        }
      }
    }
    cspace.header.stamp = ros::Time::now();
    mapdata.header.stamp = ros::Time::now();
    cspace.data = data_from_grid_c;
    cspace_pub.publish(cspace);
    rtab_map_pub.publish(mapdata);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}