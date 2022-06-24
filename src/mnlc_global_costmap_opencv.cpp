#include "mnlc_global_costmap_opencv.h"

void update_map(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
  ros::Time time_init = ros::Time::now();
  double ti = time_init.toSec();
  nav_msgs::OccupancyGrid m = *map;
  mapdata = m;
  frontiers.clear();
  cv::Mat frontier_image = cv::Mat::ones(m.info.width, m.info.height, CV_8UC1);
  std::unordered_map<int, int> uk;
  int index = 0;
  int row = 0;
  int col = 0;
  std::vector<int8_t, std::allocator<int8_t>> temp = m.data;
  BOOST_FOREACH (int8_t &datapoint, temp)
  {
    row = index / m.info.width;
    col = index % m.info.width;
    // printf("row: %d \t", row);
    // printf("col: %d \n", col);
    if (datapoint == 100)
    {
      frontier_image.ptr<unsigned char>(col)[row] = 0;
    }
    else if (datapoint == 0)
    {
      frontier_image.ptr<unsigned char>(col)[row] = 255;
    }
    else if (datapoint == -1)
    {
      uk.emplace(index, 0);
      m.data[index] = 0;
      frontier_image.ptr<unsigned char>(col)[row] = 205;
    }
    unsigned char ch = (unsigned char)m.data[index];
    image.ptr<unsigned char>(col)[row] = ch;
    index++;
  }
  // for (auto itr = uk.begin(); itr != uk.end(); itr++)
  //       std::cout << itr->first << "\t" << itr->second << std::endl;
  unknown_indices = uk;
  cv::Mat original;
  cv::inRange(frontier_image, 0, 1, original);
  cv::Mat edges;
  const double threshold1 = 0.0;
  const double threshold2 = 255.0;
  const int kernel_size = 3;
  cv::Canny(frontier_image, edges, threshold1, threshold2, kernel_size, true);
  std::vector<std::vector<cv::Point>> contours1;
  cv::findContours(original, contours1, cv::RETR_LIST, cv::CHAIN_APPROX_TC89_L1);
  cv::drawContours(original, contours1, -1, 255, 5);
  cv::Mat tmp;
  cv::bitwise_not(original, tmp);
  cv::Mat frontier;
  cv::bitwise_and(tmp, edges, frontier);
  std::vector<std::vector<cv::Point>> contours2;
  cv::findContours(frontier, contours2, cv::RETR_LIST, cv::CHAIN_APPROX_TC89_L1);
  cv::drawContours(frontier, contours2, -1, 255, 2);
  std::vector<std::vector<cv::Point>> contours3;
  cv::findContours(frontier, contours3, cv::RETR_LIST, cv::CHAIN_APPROX_TC89_L1);
  float res = m.info.resolution;
  float gox = m.info.origin.position.x;
  float goy = m.info.origin.position.y;
  cv::Point2f p;
  cv::Moments moment;
  if (sizeof(contours3) > 0)
  {
    BOOST_FOREACH (std::vector<cv::Point> &contour, contours3)
    {
      moment = cv::moments(contour, true);
      int cx = (int)(moment.m10 / moment.m00);
      int cy = (int)(moment.m01 / moment.m00);
      float xr = cx * res + gox;
      float yr = cy * res + goy;
      // printf("xr: %f\t yr: %f\n", xr, yr);
      p.x = xr;
      p.y = yr;
      frontiers.push_back(p);
    }
  }
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
  int detector_padding = (int)(padding / 2);
  visualization_msgs::Marker points;
  points.header.frame_id = "/map";
  points.header.stamp = ros::Time::now();
  points.ns = "markers";
  points.id = 9;
  points.type = visualization_msgs::Marker::POINTS;
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.scale.x = points.scale.y = 0.05;
  points.color.r = 255.0 / 255.0;
  points.color.g = 0.0 / 255.0;
  points.color.b = 0.0 / 255.0;
  points.color.a = 1;
  points.lifetime = ros::Duration();
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
  ros::Publisher detected_opencv_pub = n.advertise<geometry_msgs::PointStamped>("/opencv_points", 10);
  ros::Publisher detected_opencv_arr_pub = n.advertise<PointArray>("/opencv_points_arr", 1);
  ros::Publisher shapes_pub = n.advertise<visualization_msgs::Marker>("/OpenCVFrontierDetector/shapes", 10);
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
  geometry_msgs::PointStamped exploration_goal;
  exploration_goal.header.frame_id = "/map";
  exploration_goal.point.z = 0;
  PointArray point_array;
  ros::Rate loop_rate(60);
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
    point_array.points.clear();
    BOOST_FOREACH (cv::Point2f &frontier, frontiers)
    {
      exploration_goal.header.stamp = ros::Time(0);
      exploration_goal.point.x = frontier.y;
      exploration_goal.point.y = frontier.x;
      detected_opencv_pub.publish(exploration_goal);
      point_array.points.push_back(exploration_goal);
      geometry_msgs::Point p;
      p.x = exploration_goal.point.x;
      p.y = exploration_goal.point.y;
      points.points.push_back(p);
    }
    detected_opencv_arr_pub.publish(point_array);
    shapes_pub.publish(points);
    points.points.clear();
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