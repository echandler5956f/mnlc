#include "mnlc_local_rrt_detector.h"

rdm r; // for genrating random numbers

void update_state_machine(const std_msgs::Int8::ConstPtr &s)
{
    state = s->data;
}

void update_map(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    mapdata = *map;
}

void set_bounding_points(const geometry_msgs::PointStamped::ConstPtr &ps)
{
    geometry_msgs::Point p;
    p.x = ps->point.x;
    p.y = ps->point.y;
    points.points.push_back(p);
}

int main(int argc, char **argv)
{
    unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
    MTRand_int32 irand(init, length);
    MTRand drand;
    ros::init(argc, argv, "mnlc_local_rrt_detector", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    n.getParam("/local_rrt_detector/start_time", start_time);
    n.getParam("/controller/start_time", controller_start_time);
    n.getParam("/controller/timeout", timeout);
    n.getParam("/controller/obstacle_cost", obstacle_cost);
    n.getParam("/local_rrt_detector/eta", eta);
    tf::TransformListener listener;
    // ros::Subscriber cspace_sub = n.subscribe("/latest_map", 1, update_map);
    ros::Subscriber cspace_sub = n.subscribe("/mnlc_simple_costmap/cspace", 1, update_map);
    ros::Subscriber state_machine = n.subscribe("/mnlc_state_machine", 1, update_state_machine);
    ros::Subscriber update_bounding_points = n.subscribe("/bounding_points", 5, set_bounding_points);
    ros::Publisher detected_points = n.advertise<geometry_msgs::PointStamped>("/detected_points", 250);
    ros::Publisher shapes = n.advertise<visualization_msgs::Marker>("/mnlc_local_rrt_detector/shapes", 250);
    ros::Time st1;
    st1.fromSec(controller_start_time * 1.5);
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
    points.header.frame_id = line.header.frame_id = "/map";
    points.header.stamp = line.header.stamp = ros::Time(0);
    points.ns = line.ns = "markers";
    points.id = 0;
    line.id = 1;
    points.type = visualization_msgs::Marker::POINTS;
    line.type = visualization_msgs::Marker::LINE_LIST;
    points.action = visualization_msgs::Marker::ADD;
    line.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line.pose.orientation.w = 1.0;
    line.scale.x = line.scale.y = 0.01;
    points.scale.x = points.scale.y = 0.05;
    points.color.r = 0.0 / 255.0;
    points.color.g = 0.0 / 255.0;
    points.color.b = 153.0 / 255.0;
    points.color.a = 1.0;
    line.color.r = 124.0 / 255.0;
    line.color.g = 163.0 / 255.0;
    line.color.b = 193.0 / 255.0;
    line.color.a = 1.0;
    points.lifetime = line.lifetime = ros::Duration();
    geometry_msgs::Point p;
    geometry_msgs::PointStamped frontier;
    // ros::Time st2;
    // st2.fromSec(start_time);
    // ros::Time::sleepUntil(st2);
    while (points.points.size() < 5)
    {
        ros::spinOnce();
        shapes.publish(points);
    }
    double res = mapdata.info.resolution;
    double gox = mapdata.info.origin.position.x;
    double goy = mapdata.info.origin.position.y;
    int width = mapdata.info.width;
    float ix = pow((pow((points.points[0].y - points.points[0].y), 2) + pow((points.points[2].x - points.points[0].x), 2)), 0.5);
    float iy = pow((pow((points.points[2].y - points.points[0].y), 2) + pow((points.points[0].x - points.points[0].x), 2)), 0.5);
    float sx = (points.points[0].x + points.points[2].x) * 0.5;
    float sy = (points.points[0].y + points.points[2].y) * 0.5;
    geometry_msgs::Point tr;
    tr = points.points[4];
    std::vector<std::vector<float>> v;
    std::vector<float> xnew;
    xnew.push_back(tr.x);
    xnew.push_back(tr.y);
    v.push_back(xnew);
    points.points.clear();
    shapes.publish(points);
    float xr, yr;
    std::vector<float> x_rand, x_nearest, x_new;
    ros::Rate loop_rate(1500);
    while (ros::ok())
    {
        x_rand.clear();
        xr = (drand() * ix) - (ix * 0.5) + sx;
        yr = (drand() * iy) - (iy * 0.5) + sx;
        x_rand.push_back(xr);
        x_rand.push_back(yr);
        float min = pow((pow((x_rand[0] - v[0][0]), 2) + pow((x_rand[1] - v[0][1]), 2)), 0.5);
        int min_index;
        float temp;
        for (int i = 0; i < v.size(); i++)
        {
            temp = pow((pow((x_rand[0] - v[i][0]), 2) + pow((x_rand[1] - v[i][1]), 2)), 0.5);
            if (temp <= min)
            {
                min = temp;
                min_index = i;
            }
        }
        x_nearest = v[min_index];
        std::vector<float> x_newer;
        if (pow((pow((x_rand[0] - x_nearest[0]), 2) + pow((x_rand[1] - x_nearest[1]), 2)), 0.5) <= eta)
        {
            x_newer = x_rand;
        }
        else
        {
            float m = (x_rand[1] - x_nearest[1]) / (x_rand[0] - x_nearest[0]);
            float sign = (x_rand[0] - x_nearest[0] < 0.0) ? -1.0 : 1.0;
            x_newer.push_back(sign * (sqrt((pow(eta, 2)) / ((pow(m, 2)) + 1))) + x_nearest[0]);
            x_newer.push_back(m * (x_newer[0] - x_nearest[0]) + x_nearest[1]);
            if (x_rand[0] == x_nearest[0])
            {
                x_newer[0] = x_nearest[0];
                x_newer[1] = x_nearest[1] + eta;
            }
        }
        x_new = x_newer;
        float rez = res * 0.1;
        int stepz = int(ceil(pow((pow((x_nearest[0] - x_new[0]), 2) + pow((x_nearest[1] - x_new[1]), 2)), 0.5)) / rez);
        std::vector<float> xi = x_nearest;
        char obs = 0;
        char unk = 0;
        for (int c = 0; c < stepz; c++)
        {
            std::vector<float> x_newest;

            if (pow((pow((x_new[0] - xi[0]), 2) + pow((x_new[1] - xi[1]), 2)), 0.5) <= rez)
            {
                x_newest = x_new;
            }
            else
            {
                float m = (x_new[1] - xi[1]) / (x_new[0] - xi[0]);
                float sign = (x_new[0] - xi[0]) < 0.0 ? -1.0 : 1.0;
                x_newest.push_back(sign * (sqrt((pow(rez, 2)) / ((pow(m, 2)) + 1))) + xi[0]);
                x_newest.push_back(m * (x_newest[0] - xi[0]) + xi[1]);

                if (x_new[0] == xi[0])
                {
                    x_newest[0] = xi[0];
                    x_newest[1] = xi[1] + rez;
                }
            }
            xi = x_newest;
            int cost = mapdata.data[(floor((xi[1] - goy) / res) * width) + (floor((xi[0] - gox) / res))];
            if (cost == 100)
            {
                obs = 1;
            }

            if (cost == -1)
            {
                unk = 1;
                break;
            }
        }
        char out = 0;
        x_new = xi;
        if (unk == 1)
        {
            out = -1;
        }

        if (obs == 1)
        {
            out = 0;
        }

        if (obs != 1 && unk != 1)
        {
            out = 1;
        }
        if (out == -1)
        {
            frontier.header.stamp = ros::Time(0);
            frontier.header.frame_id = mapdata.header.frame_id;
            frontier.point.x = x_new[0];
            frontier.point.y = x_new[1];
            frontier.point.z = 0.0;
            p.x = x_new[0];
            p.y = x_new[1];
            p.z = 0.0;
            points.points.push_back(p);
            // shapes.publish(points);
            detected_points.publish(frontier);
            points.points.clear();
            v.clear();
            tf::StampedTransform transform;
            int temp = 0;
            while (temp == 0)
            {
                try
                {
                    temp = 1;
                    listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
                }
                catch (tf::TransformException ex)
                {
                    temp = 0;
                    ros::Duration(0.05).sleep();
                }
            }
            x_new[0] = transform.getOrigin().x();
            x_new[1] = transform.getOrigin().y();
            v.push_back(x_new);
            line.points.clear();
        }
        else if (out == 1)
        {
            v.push_back(x_new);
            p.x = x_new[0];
            p.y = x_new[1];
            p.z = 0.0;
            line.points.push_back(p);
            p.x = x_nearest[0];
            p.y = x_nearest[1];
            p.z = 0.0;
            line.points.push_back(p);
        }
        // shapes.publish(line);
        ros::spinOnce();
        // loop_rate.sleep();
    }
    return 0;
}