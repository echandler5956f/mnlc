#include "mnlc_hdbscan_frontier_filter.h"

void update_frontiers(const geometry_msgs::PointStamped::ConstPtr &frontier)
{
    if (first_map == false)
    {
        double x = frontier->point.x;
        double y = frontier->point.y;
        if ((x * x) + (y * y) >= info_radius)
        {
            frontiers.push_back({x, y});
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
    // if (ros::Time(0).toSec() >= next_time)
    // {
    //     frontiers.clear();
    //     next_time = ros::Time(0).toSec() + frontier_clear;
    // }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mnlc_hdbscan_frontier_filter");
    ros::NodeHandle n;
    n.getParam("/frontier_filter/start_time", start_time);
    n.getParam("/frontier_filter/obstacle_cost", obstacle_cost);
    n.getParam("/controller/timeout", timeout);
    n.getParam("/frontier_filter/frontier_clear", frontier_clear);
    n.getParam("/frontier_filter/info_radius", info_radius);
    tf::TransformListener listener;
    ros::Subscriber frontier_sub = n.subscribe("/detected_points", 250, update_frontiers);
    ros::Subscriber cspace_sub = n.subscribe("/mnlc_global_costmap_opencv/cspace", 1, update_map);
    ros::Subscriber state_machine = n.subscribe("/mnlc_state_machine", 1, update_state_machine);
    ros::Publisher shapes = n.advertise<visualization_msgs::Marker>("/frontier_filter/filtered_points_markers", 1);
    ros::Publisher filtered_points = n.advertise<PointArray>("/frontier_filter/filtered_points", 1);
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
    marker.scale.x = marker.scale.y = 0.3;
    marker.color.r = 255.0 / 255.0;
    marker.color.g = 255.0 / 255.0;
    marker.color.b = 0.0 / 255.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    double res = mapdata.info.resolution;
    double gox = mapdata.info.origin.position.x;
    double goy = mapdata.info.origin.position.y;
    int width = mapdata.info.width;
    int height = mapdata.info.height;
    rbe3002::PointArray point_array;
    geometry_msgs::PointStamped point_stamped;
    point_stamped.header.frame_id = "/map";
    ros::Rate loop_rate(120);
    while (ros::ok())
    {
        ros::spinOnce();
        // printf("Frontier inner size: %ld\t Frontier outer size: %ld\n", frontiers.innerSize(), frontiers.outerSize());
        // if (frontiers.outerSize() > 1 && frontiers.innerSize() > 1)
                if (frontiers.size() > 120)
        {
            std::vector<int8_t, std::allocator<int8_t>> data = mapdata.data;
            std::vector<std::vector<double>> f = frontiers;
            Hdbscan hdbscan(f);
            // printf("Initialized Hdbscan!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            hdbscan.execute(7, 7, "Manhattan");
            // printf("Executed Hdbscan!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            int num_clusters = *std::max_element(hdbscan.normalizedLabels_.begin(), hdbscan.normalizedLabels_.end() - 1);
            printf("Number of clusters: %d\t Number of elements: %lu\n", num_clusters, hdbscan.membershipProbabilities_.size());
            std::vector<std::vector<std::vector<double>>> cluster_groups; // cluster_groups[group][i][0: x, 1: y, 2: probability]
            std::vector<std::vector<double>> centroids;
            for (int i = 0; i < num_clusters; i++)
            {
                cluster_groups.push_back({});
            }
            // printf("Entering cluster grouping loop!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            for (int i = 0; i < hdbscan.membershipProbabilities_.size(); i++)
            {
                // printf("Assigning clusters to groups!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                if ((hdbscan.normalizedLabels_[i] != -1) && (hdbscan.normalizedLabels_[i] != 0))
                {
                    cluster_groups[hdbscan.normalizedLabels_[i] - 1].push_back({f[i][0], f[i][1], hdbscan.membershipProbabilities_[i]});
                }
            }
            // printf("Finished grouping clusters!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            for (int i = 0; i < num_clusters - 1; i++)
            { // normalized clusters is not 0 index, so we compensate
                // printf("Calculating centroid of cluster %d.\n", i);
                std::vector<double> c = {0.0, 0.0};
                for (int j = 0; j < cluster_groups[i].size(); j++)
                {
                    // printf("i: %d\t j: %d\n", i, j);
                    c[0] += cluster_groups[i][j][0];
                    c[1] += cluster_groups[i][j][1];
                }
                c[0] /= cluster_groups[i].size();
                c[1] /= cluster_groups[i].size();
                centroids.push_back(c);
                printf("Centroid %d located at (%f, %f).\n", i, c[0], c[1]);
            }
            // printf("Finished calculating centroids!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            int i = 0;
            while (i < centroids.size())
            {
                Eigen::Vector2i grid((int)(std::floor(centroids[i][0] - gox) / res), (int)(std::floor(centroids[i][1] - goy) / res));
                // int gindex = grid(0) + (grid(1) * width);
                // int cost = data[gindex];
                bool cond = (data[grid(0) + (grid(1) * width)] >= obstacle_cost);// || (data[grid(0) + (grid(1) * width)] == -1);
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
                        // printf("J: %d.\t Limit: %d\n", j, limit);
                        if ((j >= 0) && (j < limit) && (j < data.size()))
                        {
                            Eigen::Vector2d p(gox + (j - (j / width) * width) * res, goy + (j / width) * res);
                            // printf("Cost: %d.\t Radius: %f.\n", data[j], std::sqrt(std::pow(centroids(i, 0) - p(0), 2) + std::pow(centroids(i, 1) - p(1), 2)));
                            if ((data[j] == -1) && (std::sqrt(std::pow(centroids[i][0] - p(0), 2) + std::pow(centroids[i][1] - p(1), 2)) <= info_radius))
                            {
                                info_gain = info_gain + 1;
                            }
                        }
                    }
                }
                info_gain = info_gain * (std::pow(res, 2));
                printf("Info gain: %f\n", info_gain);
                // if (centroids.size() - 1 <= 2)
                // {
                //     break;
                // }
                if (cond || (info_gain < 1.0))
                {
                    // printf("Rows before: %ld.\t Collumns before: %ld.\n", centroids.rows(), centroids.cols());
                    centroids[i][0] = centroids[centroids.size() - 1][0];
                    centroids[i][1] = centroids[centroids.size() - 1][1];
                    centroids.pop_back();
                    // printf("Rows after: %ld.\t Collumns after: %ld.\n", centroids.rows(), centroids.cols());
                    // printf("i: %d\n", i);
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
                }
                filtered_points.publish(point_array);
                shapes.publish(marker);
            }
            frontiers.clear();
        }
        // loop_rate.sleep();
    }
    return 0;
}