#include "mnlc_frontier_assigner.h"

void update_filtered_frontiers(const PointArray::ConstPtr &frontiers)
{
    if (first_map == false)
    {
        for (int i = 0; i < frontiers->points.size(); i++)
        {
            filtered_frontiers.push_back({frontiers->points[i].point.x, frontiers->points[i].point.y});
        }
    }
    else
    {
        first_map = false;
    }
}

void update_opencv_frontiers(const PointArray::ConstPtr &frontiers)
{
    if (first_map == false)
    {
        for (int i = 0; i < frontiers->points.size(); i++)
        {
            filtered_frontiers.push_back({frontiers->points[i].point.x, frontiers->points[i].point.y});
        }
    }
    else
    {
        first_map = false;
    }
}

void update_visited(const geometry_msgs::Point::ConstPtr &visited)
{
    if (first_map == false)
    {
        double x = visited->x;
        double y = visited->y;
        int visited_width = 25;
        int visited_height = 25;
        int xmin = (int)(x) - (int)(std::floor(visited_width / 2));
        int xmax = (int)(x) + (int)(std::floor(visited_width / 2));
        int ymin = (int)(y) - (int)(std::floor(visited_height / 2));
        int ymax = (int)(y) + (int)(std::floor(visited_height / 2));
        for (int i = xmin; i < xmax; i++)
        {
            for (int j = ymin; j < ymax; j++)
            {
                visited_indices.emplace(i + width * j, 0);
            }
        }
        cx = (x * res) + gox + (res / 2);
        cy = (y * res) + goy + (res / 2);
    }
    else
    {
        first_map = false;
    }
}

void update_state_machine(const std_msgs::Int8::ConstPtr &s)
{
    state = s->data;
}

void update_map(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    mapdata = *map;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mnlc_frontier_assigner");
    ros::NodeHandle n;
    n.getParam("/frontier_assigner/start_time", start_time);
    n.getParam("/frontier_assigner/obstacle_cost", obstacle_cost);
    n.getParam("/controller/timeout", timeout);
    n.getParam("/frontier_assigner/info_radius", info_radius);
    n.getParam("/frontier_assigner/info_radius", info_multiplier);
    n.getParam("/frontier_assigner/hysteresis_radius", hysteresis_radius);
    n.getParam("/frontier_assigner/hysteresis_radius", hysteresis_gain);
    n.getParam("/frontier_assigner/frontier_clear", frontier_clear);
    tf::TransformListener listener;
    ros::Subscriber cspace_sub = n.subscribe("/mnlc_global_costmap_opencv/cspace", 1, update_map);
    ros::Subscriber state_machine = n.subscribe("/mnlc_state_machine", 1, update_state_machine);
    ros::Subscriber filtered_points = n.subscribe("/frontier_filter/filtered_points", 10, update_filtered_frontiers);
    ros::Subscriber get_current_cell = n.subscribe("/mnlc_controller/current_cell", 1, update_visited);
    ros::Publisher shapes = n.advertise<visualization_msgs::Marker>("/assigned_frontiers", 10);
    ros::Publisher goal_publisher = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
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
    visualization_msgs::Marker marker;
    marker.header.frame_id = mapdata.header.frame_id;
    marker.header.stamp = ros::Time(0);
    marker.ns = "goal_markers";
    marker.id = 4;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = 0.05;
    marker.color.r = 0.0 / 255.0;
    marker.color.g = 255.0 / 255.0;
    marker.color.b = 136.0 / 255.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    res = mapdata.info.resolution;
    gox = mapdata.info.origin.position.x;
    goy = mapdata.info.origin.position.y;
    width = mapdata.info.width;
    height = mapdata.info.height;
    length = mapdata.data.size();
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "/map";
    goal_pose.pose.orientation.w = 1;
    geometry_msgs::Point assigned_goal;
    std::vector<double> info_gain_arr;
    ros::Rate loop_rate(500);
    while (ros::ok())
    {
        for (int i = 0; i < filtered_frontiers.size(); i++)
        {
            double info_gain = 0;
            int index = (int)((std::floor((filtered_frontiers[i][1] - goy) / res) * width) + (std::floor((filtered_frontiers[i][0] - gox) / res)));
            int r_region = (int)(info_radius / res);
            int init_index = index - r_region * (width + 1);
            for (int j = 0; j < 2 * r_region + 1; j++)
            {
                int start = j * width + init_index;
                int end = start + 2 * r_region;
                int limit = ((start / width) + 2) * width;
                for (int k = start; k < end + 1; k++)
                {
                    double r = std::sqrt(std::pow(filtered_frontiers[i][0] - (gox + (k - (k / width) * width) * res), 2) +
                                         std::pow(filtered_frontiers[i][1] - (goy + (k / width) * res), 2));
                    if (k >= 0 && k < limit && k < length && mapdata.data[k] == -1 && r <= info_radius)
                    {
                        info_gain += 1.0;
                    }
                }
            }
            info_gain *= (res * res);
            info_gain_arr.push_back(info_gain);
        }
        int cindex = (int)((std::floor((cy - goy) / res) * width) + (std::floor((cx - gox) / res)));
        int r_region = (int)(info_radius / res);
        int init_index = cindex - r_region * (width + 1);
        for (int i = 0; i < 2 * r_region + 1; i++)
        {
            int start = i * width + init_index;
            int end = start + 2 * r_region;
            int limit = ((start / width) + 2) * width;
            for (int j = start; j < end + 1; j++)
            {
                if (j >= 0 && j < limit && j < length)
                {
                    for (int k = 0; k < filtered_frontiers.size(); k++)
                    {
                        double r = std::sqrt(std::pow((gox + (j - (j / width) * width) * res) - filtered_frontiers[k][0], 2) +
                                             std::pow((goy + (j / width) * res) - filtered_frontiers[k][1], 2));
                        double distance = std::sqrt(std::pow((gox + (j - (j / width) * width) * res) - cx, 2) +
                                                    std::pow((goy + (j / width) * res) - cy, 2));
                        if (mapdata.data[j] == -1 && r <= info_radius && distance <= info_radius)
                        {
                            info_gain_arr[k] -= (res * res);
                        }
                        int index_check = (int)((std::floor((filtered_frontiers[k][1] - goy) / res) * width) + (std::floor((filtered_frontiers[k][0] - gox) / res)));
                        auto it = visited_indices.find(index_check);
                        if (it != visited_indices.end())
                        {
                            info_gain_arr[k] -= 2.0;
                        }
                        int cost = mapdata.data[k];
                        if (cost != -1)
                        {
                            info_gain_arr[k] -= cost * 0.075;
                        }
                    }
                }
            }
        }
        std::vector<double> rev_rec;
        std::vector<std::vector<double>> centroids_rec;
        for (int i = 0; i < filtered_frontiers.size(); i++)
        {
            double distance = std::sqrt(std::pow(cx - filtered_frontiers[i][0], 2) + std::pow(cy - filtered_frontiers[i][1], 2));
            double information_gain = info_gain_arr[i];
            if (distance <= hysteresis_radius)
            {
                information_gain *= hysteresis_gain;
            }
            double rev = information_gain * info_multiplier - distance;
            rev_rec.push_back(rev);
            centroids_rec.push_back({filtered_frontiers[i][0], filtered_frontiers[i][1]});
        }
        if (rev_rec.size() > 0)
        {
            std::vector<double> best_frontier = centroids_rec[std::max_element(rev_rec.begin(), rev_rec.end()) - rev_rec.begin()];
            goal_pose.header.stamp = ros::Time::now();
            goal_pose.pose.position.x = assigned_goal.x = best_frontier[0];
            goal_pose.pose.position.y = assigned_goal.y = best_frontier[1];
            goal_publisher.publish(goal_pose);
            marker.points.clear();
            // assigned_goal.x = (198 * mapdata.info.resolution) + \
            // mapdata.info.origin.position.x + (mapdata.info.resolution/2);
            // assigned_goal.y = (161 * mapdata.info.resolution) + \
            // mapdata.info.origin.position.y + (mapdata.info.resolution/2);
            marker.points.push_back(assigned_goal);
            shapes.publish(marker);
        }
        info_gain_arr.clear();
        if (ros::Time::now().toSec() > next_time)
        {
            filtered_frontiers.clear();
            next_time = ros::Time::now().toSec() + frontier_clear;
        }
        ros::spinOnce();
        // loop_rate.sleep();
    }
    return 0;
}