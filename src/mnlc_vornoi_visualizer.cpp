#include "mnlc_vornoi_visualizer.h"

void update_global_tree_map(const geometry_msgs::Point::ConstPtr &point)
{
    if (first_map == false)
    {
        int x = (int)(std::floor((point->x - gox) / res));
        int y = (int)(std::floor((point->y - goy) / res));
        cv::Point2f p((float)y, (float)x);
        subdiv.insert(p);
    }
}

void update_state_machine(const std_msgs::Int8::ConstPtr &s)
{
    state = s->data;
}

void update_map(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    mapdata = *map;
    if (first_map)
    {
        first_map = false;
    }
    int row = 0;
    int col = 0;
    int cost;
    for (int i = 0; i < mapdata.data.size(); i++)
    {
        row = i / mapdata.info.width;
        col = i % mapdata.info.width;
        cost = mapdata.data[i];
        if (cost == 100)
        {
            map_img.ptr<unsigned char>(col)[row] = 0;
        }
        else
        {
            map_img.ptr<unsigned char>(col)[row] = 255;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mnlc_vornoi_visualizer");
    ros::NodeHandle n;
    n.getParam("/global_rrt_detector/start_time", start_time);
    n.getParam("/controller/timeout", timeout);
    tf::TransformListener listener;
    ros::Subscriber cspace_sub = n.subscribe("/mnlc_simple_costmap/cspace", 1, update_map);
    ros::Subscriber state_machine = n.subscribe("/mnlc_state_machine", 1, update_state_machine);
    ros::Time st1;
    st1.fromSec(start_time);
    ros::Time::sleepUntil(st1);
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
    ros::Time sr;
    sr.fromSec(timeout);
    ros::Time::sleepUntil(sr);
    map_img = cv::Mat::zeros(261, 261, CV_8UC1);
    image = cv::Mat::zeros(261, 261, CV_8UC3);
    cv::Mat flipped = cv::Mat::zeros(261, 261, CV_8UC3);
    cv::Mat masked = cv::Mat::zeros(261, 261, CV_8UC3);
    cv::Mat resized = cv::Mat::zeros(1305, 1305, CV_8UC3);
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
    res = mapdata.info.resolution;
    gox = mapdata.info.origin.position.x;
    goy = mapdata.info.origin.position.y;
    width = mapdata.info.width;
    height = mapdata.info.height;
    ros::Subscriber detected_tree_sub = n.subscribe("/detected_global_tree", 250, update_global_tree_map);
    ros::spinOnce();
    ros::Rate loop_rate(500);
    while (ros::ok())
    {
        draw_voronoi(image, subdiv);
        cv::bitwise_and(image, image, masked, map_img);
        cv::flip(masked, flipped, -1);
        cv::resize(flipped, resized, resized.size(), 0, 0, CV_INTER_CUBIC);
        cv::imshow("Vornoi Diagram", resized);
        cv::waitKey(2);
        ros::spinOnce();
        // loop_rate.sleep();
    }
    return 0;
}