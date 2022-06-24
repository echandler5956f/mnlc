#include "mnlc_vornoi_visualizer.h"

rdm r; // for genrating random numbers

void update_global_tree_map(const geometry_msgs::Point::ConstPtr &point)
{
    if (first_map == false)
    {
        int x = (int)(std::floor((point->x - gox) / res));
        int y = (int)(std::floor((point->y - goy) / res));
        cv::Point p((int)(x * (263/261)), (int)(y * (263/261)));
        rrt_detector_points.push_back(p);
        image.ptr<unsigned char>(x)[y] = 255;
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
        if (cost == 100 || cost == -1)
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
    unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
    MTRand_int32 irand(init, length);
    MTRand drand;
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
    image = cv::Mat::zeros(261, 261, CV_8UC1);
    cv::Mat inverse_image = cv::Mat::zeros(261, 261, CV_8UC1);
    cv::Mat distance_transform_output = cv::Mat::zeros(261, 261, CV_32FC1);
    cv::Mat vornoi_labels = cv::Mat::zeros(261, 261, CV_32SC1);
    cv::Mat vornoi_corrected = cv::Mat::zeros(261, 261, CV_32FC1);
    cv::Mat vornoi_colorspaced = cv::Mat::zeros(261, 261, CV_32FC3);
    cv::Mat vornoi_float_normalized = cv::Mat::zeros(261, 261, CV_32FC3);
    cv::Mat vornoi_masked = cv::Mat::zeros(261, 261, CV_32FC3);
    cv::Mat vornoi_resized = cv::Mat::zeros(1827, 1827, CV_32FC3);
    cv::Mat flipped_image = cv::Mat::zeros(1827, 1827, CV_32FC3);
    ros::Subscriber detected_tree_sub = n.subscribe("/detected_global_tree", 250, update_global_tree_map);
    ros::spinOnce();
    ros::Rate loop_rate(500);
    while (ros::ok())
    {
        cv::bitwise_not(image, inverse_image);
        cv::distanceTransform(inverse_image, distance_transform_output, vornoi_labels, CV_DIST_L2, CV_DIST_MASK_5, cv::DistanceTransformLabelTypes::DIST_LABEL_CCOMP);
        vornoi_labels.convertTo(vornoi_corrected, CV_32FC1);
        cv::cvtColor(vornoi_corrected, vornoi_colorspaced, cv::COLOR_GRAY2BGR);
        cv::normalize(vornoi_colorspaced, vornoi_float_normalized, 0.0, 1.0, cv::NORM_MINMAX);
        cv::Mat flood_mask = cv::Mat::zeros(263, 263, CV_8UC1);
        for (int i = 0; i < rrt_detector_points.size(); i++) {
            cv::floodFill(vornoi_float_normalized, flood_mask, rrt_detector_points[i], cv::Scalar(drand(), drand(), drand()), (cv::Rect *)0, cv::Scalar(0.0025, 0.0025, 0.0025), cv::Scalar(0.0025, 0.0025, 0.0025), 8 | cv::FLOODFILL_FIXED_RANGE);
        }
        cv::bitwise_and(vornoi_float_normalized, vornoi_float_normalized, vornoi_masked, map_img);
        cv::resize(vornoi_masked, vornoi_resized, vornoi_resized.size(), 0, 0, CV_INTER_CUBIC);
        cv::flip(vornoi_resized, flipped_image, -1);
        cv::imshow("Vornoi Diagram", flipped_image);
        cv::waitKey(3);
        ros::spinOnce();
        // loop_rate.sleep();
    }
    return 0;
}