#include "mnlc_hdbscan_frontier_filter.h"

void update_frontiers(const geometry_msgs::PointStamped::ConstPtr &frontier)
{
    if (first_map == false)
    {
        hdbscan.dataset.push_back({frontier->point.x, frontier->point.y});
    }
    else
    {
        first_map = false;
    }
}

void update_centroids(const PointArray::ConstPtr &c)
{
    if (first_map == false)
    {
        centroids.clear();
        for (int i = 0; i < c->points.size(); i++)
        {
            centroids.push_back({c->points[i].point.x, c->points[i].point.y});
        }
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
    ros::init(argc, argv, "mnlc_hdbscan_frontier_filter");
    ros::NodeHandle n;
    n.getParam("/hdbscan_frontier_filter/start_time", start_time);
    n.getParam("/hdbscan_frontier_filter/obstacle_cost", obstacle_cost);
    n.getParam("/controller/timeout", timeout);
    n.getParam("/hdbscan_frontier_filter/info_radius", info_radius);
    n.getParam("/hdbscan_frontier_filter/frontier_clear", frontier_clear);
    tf::TransformListener listener;
    ros::Subscriber cspace_sub = n.subscribe("/mnlc_global_costmap_opencv/cspace", 1, update_map);
    ros::Subscriber state_machine = n.subscribe("/mnlc_state_machine", 1, update_state_machine);
    ros::Subscriber opencv_points_arr = n.subscribe("/opencv_points_arr", 1, update_centroids);
    ros::Publisher shapes = n.advertise<visualization_msgs::Marker>("/frontier_filter/filtered_points_markers", 1);
    ros::Publisher filtered_points = n.advertise<PointArray>("/frontier_filter/filtered_points", 1);
    ros::Publisher filtered_points_single = n.advertise<geometry_msgs::PointStamped>("/frontier_filter/filtered_points_single", 10);
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
    marker.ns = "filtered_markers";
    marker.id = 3;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = 0.05;
    marker.color.r = 255.0 / 255.0;
    marker.color.g = 255.0 / 255.0;
    marker.color.b = 0.0 / 255.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    res = mapdata.info.resolution;
    gox = mapdata.info.origin.position.x;
    goy = mapdata.info.origin.position.y;
    width = mapdata.info.width;
    height = mapdata.info.height;
    ros::Subscriber frontier_sub = n.subscribe("/detected_points", 250, update_frontiers);
    PointArray point_array;
    geometry_msgs::PointStamped point_stamped;
    point_stamped.header.frame_id = "/map";
    ros::Rate loop_rate(500);
    while (ros::ok())
    {
        if (hdbscan.dataset.size() > 250)
        {
            std::vector<int8_t, std::allocator<int8_t>> data = mapdata.data;
            hdbscan.execute(25, 25, "Euclidean");
            // hdbscan.displayResult();
            int num_clusters = hdbscan.numClusters_;
            std::vector<ClusterPoint> cluster_groups;
            for (int i = 0; i < hdbscan.normalizedLabels_.size(); i++)
            {
                ClusterPoint cp(hdbscan.normalizedLabels_[i], hdbscan.dataset[i][0], hdbscan.dataset[i][1], hdbscan.membershipProbabilities_[i]);
                cluster_groups.push_back(cp);
            }
            std::sort(begin(cluster_groups), end(cluster_groups), [](ClusterPoint lhs, ClusterPoint rhs)
                      { return rhs.group > lhs.group; });
            // for (int i = 0; i < cluster_groups.size(); i++)
            // {
            //     printf("Group: %d\tX: %f\tY: %f\tProbability: %f\n", cluster_groups[i].group, cluster_groups[i].x, cluster_groups[i].y, cluster_groups[i].probability);
            // }
            int group = cluster_groups[0].group;
            int i = 0;
            bool passed_through = (group == -1);
            while (group == -1)
            {
                group = cluster_groups[i].group;
                i++;
            }
            if (passed_through)
            {
                i--;
            }
            if (cluster_groups.size() - i > 5)
            {
                while (i < cluster_groups.size())
                {
                    std::vector<double> centroid = {0.0, 0.0};
                    int start = i;
                    group = cluster_groups[start].group;
                    while (group == cluster_groups[i].group)
                    {
                        centroid[0] += cluster_groups[i].x;
                        centroid[1] += cluster_groups[i].y;
                        i++;
                        if (i >= cluster_groups.size())
                        {
                            break;
                        }
                    }
                    centroid[0] /= (i - start);
                    centroid[1] /= (i - start);
                    // printf("Centroid %d located at (%f, %f).\n", i, centroid[0], centroid[1]);
                    centroids.push_back(centroid);
                }
                hdbscan.dataset = centroids;
                i = 0;
                while (i < centroids.size())
                {
                    Eigen::Vector2i grid((int)(std::floor(centroids[i][0] - gox) / res), (int)(std::floor(centroids[i][1] - goy) / res));
                    int cost = data[grid(0) + (grid(1) * width)];
                    bool cond = (cost >= obstacle_cost);
                    float info_gain = 0.0;
                    int index = (int)((std::floor((centroids[i][1] - goy) / res) * width) + (std::floor((centroids[i][0] - gox) / res)));
                    int r_region = (int)(info_radius / res);
                    int init_index = index - r_region * (width + 1);
                    for (int n = 0; n < 2 * r_region + 1; n++)
                    {
                        int start = n * width + init_index;
                        int end = start + 2 * r_region;
                        int limit = ((start / width) + 2) * width;
                        for (int j = start; j < end + 1; j++)
                        {
                            if ((j >= 0) && (j < limit) && (j < data.size()))
                            {
                                Eigen::Vector2d p(gox + (j - (j / width) * width) * res, goy + (j / width) * res);
                                float distance = std::sqrt(std::pow(centroids[i][0] - p(0), 2) + std::pow(centroids[i][1] - p(1), 2));
                                if (distance <= info_radius)
                                {
                                    float distance_modifier = (std::pow(2, ((-11 * distance) / (2 * info_radius))));
                                    float obstacle_modifier = 0.5 * ((std::pow(9, (data[j] / 100))) - 1) * distance_modifier;
                                    if (data[j] == -1)
                                    {
                                        info_gain += distance_modifier;
                                    }
                                    else
                                    {
                                        info_gain -= obstacle_modifier;
                                    }
                                }
                            }
                        }
                    }
                    info_gain *= 10 * (std::pow(res, 2));
                    // printf("Info gain: %f\n", info_gain);
                    if (cond || (info_gain < 0.425) || (info_gain > 1.375))
                    {
                        centroids[i][0] = centroids[centroids.size() - 1][0];
                        centroids[i][1] = centroids[centroids.size() - 1][1];
                        centroids.pop_back();
                        i = i - 1;
                    }
                    i = i + 1;
                }
                point_array.points.clear();
                marker.points.clear();
                if (centroids.size() > 1)
                {
                    for (int k = 0; k < centroids.size(); k++)
                    {
                        point_stamped.point.x = centroids[k][0];
                        point_stamped.point.y = centroids[k][1];
                        point_stamped.header.stamp = ros::Time(0);
                        point_array.points.push_back(point_stamped);
                        marker.points.push_back(point_stamped.point);
                        filtered_points_single.publish(point_stamped);
                    }
                    filtered_points.publish(point_array);
                    shapes.publish(marker);
                }
            }
        }
        ros::spinOnce();
        // loop_rate.sleep();
    }
    return 0;
}