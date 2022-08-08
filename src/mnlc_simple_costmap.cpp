#include "mnlc_simple_costmap.h"

void update_map(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
  mapdata = *map;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mnlc_simple_costmap");
  ros::NodeHandle n;
  double start_time;
  n.getParam("/simple_costmap/start_time", start_time);
  double timeout;
  n.getParam("/controller/timeout", timeout);
  int padding;
  n.getParam("/simple_costmap/padding", padding);
  bool first_map = true;
  int detector_padding = (int)(padding / 2);
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
  listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(timeout));
  nav_msgs::OccupancyGrid cspace = srv1.response.map;
  int width = cspace.info.width;
  int height = cspace.info.height;
  double res = cspace.info.resolution;
  double gox = cspace.info.origin.position.x;
  double goy = cspace.info.origin.position.y;
  ros::Publisher cspace_pub = n.advertise<nav_msgs::OccupancyGrid>("/mnlc_simple_costmap/cspace", 1);
  ros::Subscriber rtabmap_sub = n.subscribe("/map", 1, update_map);
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
  cspace.header.frame_id = "/map";
  cspace.header.stamp = ros::Time(0);
  ros::Rate loop_rate(60);
  while (ros::ok())
  {
    nav_msgs::OccupancyGrid tmp = mapdata;
    cspace = mapdata;
    for (int i = 0; i < padding + 1; i++)
    {
      tmp = cspace;
      for (int index = 0; index < mapdata.data.size(); index++)
      {
        int x = index / width;
        int y = index % width;
        if ((tmp.data[index] == 100) && (x < width - 1) && (x > 0 + 1) && (y < height - 1) && (y > 0 + 1))
        {
          cspace.data[index] = 100;
          cspace.data[(x - 1) * width + y - 1] = 100;
          cspace.data[(x - 1) * width + y] = 100;
          cspace.data[(x - 1) * width + y + 1] = 100;
          cspace.data[x * width + y - 1] = 100;
          cspace.data[(x + 1) * width + y - 1] = 100;
          cspace.data[(x + 1) * width + y] = 100;
          cspace.data[(x + 1) * width + y + 1] = 100;
          cspace.data[x * width + y + 1] = 100;
        }
      }
    }
    cspace.header.stamp = ros::Time(0);
    cspace_pub.publish(cspace);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}