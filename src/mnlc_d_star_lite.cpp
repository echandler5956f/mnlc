#include "d_star_lite_ros.h"

void update_map(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    mapdata = *map;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mnlc_d_star_lite");
    ros::NodeHandle n;
    n.getParam("/d_star_lite/start_time", start_time);
    n.getParam("/controller/timeout", timeout);
    n.getParam("/d_star_lite/obstacle_cost", obstacle_cost);
    n.getParam("/d_star_lite/unknown_cost", unknown_cost);
    n.getParam("/d_star_lite/scan_radius", scan_radius);
    n.getParam("/d_star_lite/heuristic_weight", heuristic_weight);
    n.getParam("/d_star_lite/obstacle_tolerance", obstacle_tolerance);
    n.getParam("/d_star_lite/search_tolerance", search_tolerance);
    n.getParam("/d_star_lite/max_its", max_its);
    nav_msgs::OccupancyGrid global_costmap;
    tf::TransformListener listener;
    ros::Subscriber cspace_sub = n.subscribe("/mnlc_global_costmap_opencv/cspace", 1, update_map);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/Field_D_Star", 1);
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
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = mapdata.header.frame_id;
    ps.pose.orientation.w = 1.0;
    path_p.header.frame_id = mapdata.header.frame_id;
    pair<unsigned int, unsigned int> start = std::make_pair(138, 138);
    pair<unsigned int, unsigned int> goal = std::make_pair(198, 161);
    DStarLite::DStarLiteROS::Config config(mapdata, start, goal, obstacle_cost, unknown_cost, scan_radius,
                                           true, heuristic_weight, obstacle_tolerance, search_tolerance, max_its);
    DStarLite::DStarLiteROS ds(config);
    uint64_t timer_start, timer_end;
    ros::Rate loop_rate(60);
    while (ros::ok())
    {
        // timer_start = ros::Time::now().toNSec();
        vector<pair<double, double>> path = ds.execute(start);
        // timer_end = ros::Time::now().toNSec();
        // printf("Execute Field D* took: %" PRIu64 "\n", timer_end - timer_start);
        path_p.header.stamp = ros::Time::now();
        path_p.poses.clear();
        for (unsigned int i = 0; i < path.size(); i++)
        {
            ps.pose.position.x = (path[i].first * mapdata.info.resolution) + mapdata.info.origin.position.x;
            ps.pose.position.y = (path[i].second * mapdata.info.resolution) + mapdata.info.origin.position.y;
            ps.header.stamp = ros::Time::now();
            path_p.poses.push_back(ps);
        }
        path_pub.publish(path_p);
        // timer_start = ros::Time::now().toNSec();
        ds.update_map(mapdata.data);
        // timer_end = ros::Time::now().toNSec();
        // printf("Update map of Field D* took: %" PRIu64 "\n", timer_end - timer_start);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ds.~DStarLiteROS();
    return 0;
}