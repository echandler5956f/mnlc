#include "mnlc_a_star.h"

void a_star(Node start, Node goal, std::vector<geometry_msgs::Point> &path)
{
    double time_init = ros::Time(0).toSec();
    ROS_INFO("Beginning A* algorithm");
    printf("Executing A* from (%d, %d) to (%d, %d).", start.x, start.y, goal.x, goal.y);
    int width = mapdata.info.width;
    int height = mapdata.info.height;
    float res = mapdata.info.resolution;
    float gox = mapdata.info.origin.position.x;
    float goy = mapdata.info.origin.position.y;
    std::vector<int8_t, std::allocator<int8_t>> data = mapdata.data;
    frontier_vis.cells.clear();
    frontier_vis.cell_height = frontier_vis.cell_width = res;
    frontier_vis.header.frame_id = "/map";
    std::priority_queue<Node, std::vector<Node>, std::function<bool(Node, Node)>> frontiers(Compare);
    frontiers.push(start);
    std::unordered_map<int, int> came_from;
    std::unordered_map<int, int> cost_so_far;
    int start_index = start.x + (start.y * width);
    int goal_index = goal.x + (goal.y * width);
    came_from[start_index] = NULL;
    cost_so_far[start_index] = 0;
    //   for (auto itr = came_from.begin(); itr != came_from.end(); itr++)
    //     std::cout << itr->first << "\t" << itr->second << std::endl;
    // goes ccw starting from middle right node
    Node nbr1(goal.x + 1, goal.y + 0);
    Node nbr2(goal.x + 1, goal.y + 1);
    Node nbr3(goal.x + 0, goal.y + 1);
    Node nbr4(goal.x - 1, goal.y + 1);
    Node nbr5(goal.x - 1, goal.y + 0);
    Node nbr6(goal.x - 1, goal.y - 1);
    Node nbr7(goal.x + 0, goal.y - 1);
    Node nbr8(goal.x + 1, goal.y - 1);
    Node gnbrs[9] = {goal, nbr1, nbr2, nbr3, nbr4, nbr5, nbr6, nbr7, nbr8};
    for (int i = 0; i < 9; i++)
    {
        if (data[gnbrs[i].x + (gnbrs[i].y * width)] >= obstacle_cost)
        {
            ROS_WARN("A* has determined that the goal is invalid. Requesting new path...");
            return;
        }
    }
    Node connbrs[8];
    double start_time = ros::Time(0).toSec();
    while (frontiers.empty() == false)
    {
        Node current = frontiers.top();
        frontiers.pop();
        int cindex = current.x + (current.y * width);
        if ((current.x == goal.x) && (current.y == goal.y))
        {
            ROS_INFO("Path has been found.");
            break;
        }
        if (ros::Time(0).toSec() >= start_time + 1.0)
        {
            ROS_WARN("A* has not found a path in less than 1.0 seconds. Requesting new path...");
            return;
        }
        Node nbr1(current.x + 1, current.y + 0);
        connbrs[0] = nbr1;
        Node nbr2(current.x + 1, current.y + 1);
        connbrs[1] = nbr2;
        Node nbr3(current.x + 0, current.y + 1);
        connbrs[2] = nbr3;
        Node nbr4(current.x - 1, current.y + 1);
        connbrs[3] = nbr4;
        Node nbr5(current.x - 1, current.y + 0);
        connbrs[4] = nbr5;
        Node nbr6(current.x - 1, current.y - 1);
        connbrs[5] = nbr6;
        Node nbr7(current.x + 0, current.y - 1);
        connbrs[6] = nbr7;
        Node nbr8(current.x + 1, current.y - 1);
        connbrs[7] = nbr8;
        for (int i = 0; i < 8; i++)
        {
            int cost = data[connbrs[i].x + (connbrs[i].y * width)];
            int mcost;
            if (cost < 0)
            {
                mcost = 20;
            }
            else
            {
                mcost = cost;
            }
            if ((connbrs[i].x > 0) && (connbrs[i].x < width) &&
                (connbrs[i].y > 0) && (connbrs[i].y < height) &&
                (mcost < obstacle_cost))
            {
                int index = connbrs[i].x + (connbrs[i].y * width);
                int new_cost = cost_so_far[cindex] + mcost;
                if ((cost_so_far.find(index) == cost_so_far.end()) || (new_cost < cost_so_far[index]))
                {
                    cost_so_far[index] = new_cost;
                    connbrs[i].priority = new_cost + (0.5 * std::sqrt(std::pow(goal.x - connbrs[i].x, 2) + std::pow(goal.y - connbrs[i].y, 2)));
                    frontiers.push(connbrs[i]);
                    came_from[index] = cindex;
                    geometry_msgs::Point point;
                    point.x = (current.x * res) + gox + (res / 2);
                    point.y = (current.y * res) + goy + (res / 2);
                    frontier_vis.cells.push_back(point);
                }
            }
            // if (frontiers.empty()) {
            //     printf("Empty.\n");
            // } else {
            //     std::size_t size = frontiers.size();
            //     printf("Size: %zu\n", size);
            // }
        }
    }
    frontier_vis.header.stamp = ros::Time(0);
    ROS_INFO("Reconstructing path.");
    int c = goal_index;
    geometry_msgs::Point point;
    while (c != start_index)
    {
        point.x = ((c % width) * res) + gox + (res / 2);
        point.y = ((c / width) * res) + goy + (res / 2);
        path.push_back(point);
        // printf("c: %d\t", c);
        c = came_from[c];
        // printf("came from: %d\n", c);
    }
    point.x = ((start_index % width) * res) + gox + (res / 2);
    point.y = ((start_index / width) * res) + goy + (res / 2);
    path.push_back(point);
    ROS_INFO("Path reconstructed.");
    std::reverse(path.begin(), path.end());
    ROS_INFO("A* has successfully found the optimal path.");
    printf("Calculating A* took: %f\n", ros::Time(0).toSec() - time_init);
}

void update_map(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    mapdata = *map;
}

bool plan_path(nav_msgs::GetPlan::Request &plan_request, nav_msgs::GetPlan::Response &resp)
{
    int width = mapdata.info.width;
    int height = mapdata.info.height;
    float res = mapdata.info.resolution;
    float gox = mapdata.info.origin.position.x;
    float goy = mapdata.info.origin.position.y;
    Node start((int)floor((plan_request.start.pose.position.x - gox) / res),
               (int)floor((plan_request.start.pose.position.y - goy) / res), 0);
    Node goal((int)floor((plan_request.goal.pose.position.x - gox) / res),
              (int)floor((plan_request.goal.pose.position.y - goy) / res));
    std::vector<geometry_msgs::Point> path_of_points;
    frontier_vis.cells.clear();
    a_star(start, goal, path_of_points);
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    path.header.frame_id = pose.header.frame_id = "/map";
    path.header.stamp = pose.header.stamp = ros::Time(0);
    pose.pose.orientation.w = 1.0;
    if (path_of_points.size() <= 3)
    {
        ROS_INFO("A* has not found a valid path within the alotted time. Send a new goal point.");
        path.poses.push_back(pose);
        pose.header.seq = 420;
        resp.plan = path;
        return true;
    }
    BOOST_FOREACH (geometry_msgs::Point &point, path_of_points)
    {
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        path.poses.push_back(pose);
    }
    resp.plan = path;
    ROS_INFO("A* has computed the path");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mnlc_a_star");
    ros::NodeHandle n;
    n.getParam("/a_star/start_time", start_time);
    n.getParam("/controller/timeout", timeout);
    n.getParam("/a_star/obstacle_cost", obstacle_cost);
    nav_msgs::OccupancyGrid global_costmap;
    tf::TransformListener listener;
    ros::Subscriber cspace_sub = n.subscribe("/mnlc_global_costmap_opencv/cspace", 1, update_map);
    ros::Publisher frontier_pub = n.advertise<nav_msgs::GridCells>("/a_star/frontier", 1);
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
    ros::ServiceServer planner = n.advertiseService("/plan_path", plan_path);
    frontier_vis.header.frame_id = "/map";
    frontier_vis.header.stamp = ros::Time(0);
    ros::Rate loop_rate(120);
    while (ros::ok())
    {
        ros::spinOnce();
        frontier_pub.publish(frontier_vis);
        loop_rate.sleep();
    }
    return 0;
}