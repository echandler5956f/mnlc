#include "mnlc_meanshift_frontier_filter.h"

void update_frontiers(const geometry_msgs::PointStamped::ConstPtr &frontier)
{
    if (first_map == false)
    {
        frontiers.conservativeResize(frontiers.rows() + 1, 2);
        frontiers(frontiers.rows() - 1, 0) = frontier->point.x;
        frontiers(frontiers.rows() - 1, 1) = frontier->point.y;
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
    if (ros::Time(0).toSec() >= next_time)
    {
        frontiers.resize(0, 2);
        // frontiers.resize(2, 0);
        next_time = ros::Time(0).toSec() + frontier_clear;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mnlc_meanshift_frontier_filter");
    ros::NodeHandle n;
    py::scoped_interpreter guard{};
    auto cluster = py::module::import("sklearn.cluster");
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
    marker.scale.x = marker.scale.y = 0.05;
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
    ros::Rate loop_rate(500);
    while (ros::ok())
    {
        ros::spinOnce();
        // printf("Frontier inner size: %ld\t Frontier outer size: %ld\n", frontiers.innerSize(), frontiers.outerSize());
        if (frontiers.outerSize() > 1 && frontiers.innerSize() > 1)
        {
            std::vector<int8_t, std::allocator<int8_t>> data = mapdata.data;
            // auto b = cluster.attr("estimate_bandwidth")("X"_a = frontiers, "quantile"_a = 0.3, "n_samples"_a = 25, "random_state"_a = 12 );
            // float bandwidth = b.cast<float>();
            // printf("%f\n", bandwidth);
            // if (bandwidth < 0.001)
            // {
            //     bandwidth = 0.3;
            // }
            auto ms = cluster.attr("MeanShift")("bandwidth"_a = 0.3, "bin_seeding"_a = true);
            ms.attr("fit")("X"_a = frontiers);
            auto c = ms.attr("cluster_centers_");
            RowMatrixXd centroids = c.cast<RowMatrixXd>();
            // printf("Centroids inner size: %ld\t Centroids outer size: %ld\n", centroids.innerSize(), centroids.outerSize());
            int i = 0;
            while (i < centroids.outerSize())
            {
                Eigen::Vector2i grid((int)(std::floor(centroids(i, 0) - gox) / res), (int)(std::floor(centroids(i, 1) - goy) / res));
                int gindex = grid(0) + (grid(1) * width);
                int cost = data[gindex];
                bool cond = (cost >= obstacle_cost) || (cost == -1);
                float info_gain = 0.0;
                int index = (int)((std::floor((centroids(i, 0) - gox) / res)) + (std::floor((centroids(i, 1) - goy) / res)) * width);
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
                            if ((data[j] == -1) && (std::sqrt(std::pow(centroids(i, 0) - p(0), 2) + std::pow(centroids(i, 1) - p(1), 2)) <= info_radius))
                            {
                                info_gain = info_gain + 1;
                            }
                        }
                    }
                }
                info_gain = info_gain * (std::pow(res, 2));
                // if (centroids.rows() - 1 <= 2)
                // {
                //     break;
                // }
                // printf("Info gain: %f\n", info_gain);
                // if (cond || (info_gain < 0.8125))
                // {
                //     // printf("Rows before: %ld.\t Collumns before: %ld.\n", centroids.rows(), centroids.cols());
                //     centroids(i, 0) = centroids(centroids.rows(), 0);
                //     centroids(i, 1) = centroids(centroids.rows(), 1);
                //     centroids.resize(centroids.rows() - 1, 2);
                //     // printf("Rows after: %ld.\t Collumns after: %ld.\n", centroids.rows(), centroids.cols());
                //     // printf("i = %d\n", i);
                //     i = i - 1;
                // }
                i = i + 1;
            }
            point_array.points.clear();
            marker.points.clear();
            if (centroids.outerSize() > 2)
            {
                for (int k = 0; k < centroids.outerSize(); k++)
                {
                    // if (!((centroids(k, 0) == 0.0) && (centroids(k, 1) == 0.0)))
                    // {
                        point_stamped.point.x = centroids(k, 0);
                        point_stamped.point.y = centroids(k, 1);
                        point_stamped.header.stamp = ros::Time(0);
                        point_array.points.push_back(point_stamped);
                        marker.points.push_back(point_stamped.point);
                    // }
                }
                filtered_points.publish(point_array);
                shapes.publish(marker);
            }
        }
        loop_rate.sleep();
    }
    return 0;
}