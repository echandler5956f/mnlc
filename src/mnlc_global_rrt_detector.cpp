#include "mnlc_global_rrt_detector.h"

void update_state_machine(const std_msgs::Int8::ConstPtr &s)
{
    state = s->data;
}

void update_map(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    mapdata = *map;
}

void set_bounding_points(const geometry_msgs::PointStamped::ConstPtr &p)
{
    bounding_points.push_back(p->point);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mnlc_global_rrt_detector", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    n.getParam("/global_rrt_detector/start_time", start_time);
    n.getParam("/controller/start_time", controller_start_time);
    n.getParam("/controller/timeout", timeout);
    n.getParam("/controller/obstacle_cost", obstacle_cost);
    n.getParam("/global_rrt_detector/eta", eta);
    tf::TransformListener listener;
    ros::Subscriber cspace_sub = n.subscribe("/latest_map", 1, update_map);
    ros::Subscriber state_machine = n.subscribe("/mnlc_state_machine", 1, update_state_machine);
    ros::Subscriber update_bounding_points = n.subscribe("/bounding_points", 10, set_bounding_points);
    ros::Publisher detected_points = n.advertise<geometry_msgs::PointStamped>("/detected_points", 250);
    ros::Publisher shapes = n.advertise<visualization_msgs::Marker>("/mnlc_global_rrt_detector/shapes", 1);
    ros::Time st1;
    st1.fromSec(controller_start_time);
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
    ros::Time st2;
    st2.fromSec(start_time * 1.5);
    ros::Time::sleepUntil(st2);
    ros::spinOnce();
    visualization_msgs::Marker points;
    visualization_msgs::Marker line;
    points.points = bounding_points;
    points.header.frame_id = line.header.frame_id = mapdata.header.frame_id;
    points.header.stamp = line.header.stamp = ros::Time::now();
    points.ns = line.ns = "markers";
    points.id = 0;
    line.id = 1;
    points.type = visualization_msgs::Marker::POINTS;
    line.type = visualization_msgs::Marker::LINE_LIST;
    points.action = visualization_msgs::Marker::ADD;
    line.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line.pose.orientation.w = 1.0;
    line.scale.x = line.scale.y = 0.01;
    points.scale.x = points.scale.y = 0.1;
    points.color.r = 153.0 / 255.0;
    points.color.g = 0.0 / 255.0;
    points.color.b = 0.0 / 255.0;
    points.color.a = 1.0;
    line.color.r = 163.0 / 255.0;
    line.color.g = 124.0 / 255.0;
    line.color.b = 193.0 / 255.0;
    line.color.a = 1.0;
    points.lifetime = line.lifetime = ros::Duration();
    double ix = std::sqrt(std::pow(points.points[0].x - points.points[2].x, 2)
              + std::pow(points.points[0].y - points.points[0].y, 2));
    double iy = std::sqrt(std::pow(points.points[0].x - points.points[0].x, 2)
              + std::pow(points.points[0].y - points.points[2].y, 2));
    double sx = (points.points[0].x + points.points[2].x) * 0.5;
    double sy = (points.points[0].y + points.points[2].y) * 0.5;
    geometry_msgs::Point tr;
    tr = points.points[4];
    std::vector<std::vector<double>> v;
    std::vector<double> rnew;
    rnew.push_back(tr.y);
    rnew.push_back(tr.x);
    v.push_back(rnew);
    shapes.publish(points);
    points.points.clear();
    rnew.clear();
    double res = mapdata.info.resolution;
    double gox = mapdata.info.origin.position.x;
    double goy = mapdata.info.origin.position.y;
    int width = mapdata.info.width;
    unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
    MTRand_int32 irand(init, length);
    MTRand drand;
    geometry_msgs::PointStamped point;
    geometry_msgs::Point p;
    point.header.frame_id = "/map";
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        std::vector<int8_t, std::allocator<int8_t>> data = mapdata.data;
        double rand[2] = {(drand() * ix) - (ix * 0.5) + sx, (drand() * iy) - (iy * 0.5) + sy};
        std::vector<double> voi = v[0];
        double min = std::sqrt(std::pow(voi[1] - rand[1], 2) + std::pow(voi[0] - rand[0], 2));
        int min_index = 0;
        double temp = 0.0;
        std::vector<double> vi;
        for (int i = 0; i < v.size(); i++)
        {
            vi = v[i];
            temp = std::sqrt(std::pow(vi[1] - rand[1], 2) + std::pow(vi[0] - rand[0], 2));
            if (temp <= min)
            {
                min = temp;
                min_index = i;
            }
        }
        std::vector<double> near = v[min_index];
        std::vector<double> n;
        if (std::sqrt(std::pow(near[1] - rand[1], 2) + std::pow(near[0] - rand[0], 2)) <= eta)
        {
            n.push_back(rand[1]);
            n.push_back(rand[0]);
        }
        else
        {
            double m = (rand[1] - near[1]) / (rand[0] - near[0]);
            int sign = 1;
            if (rand[0] - near[0] < 0.0)
            {
                sign = -1;
            }
            n.push_back((sign * (std::sqrt(std::pow(eta, 2)) / ((std::pow(m, 2)) + 1))) + near[0]);
            n.push_back(m * (n[0] - near[0]) + near[1]);
            if (std::fabs(rand[0] - near[0]) < 1E-3)
            {
                n[0] = near[0];
                n[1] = near[1] + eta;
            }
        }
        rnew = n;
        double rez = res * 0.1;
        double norm = std::sqrt(std::pow(rnew[1] - near[1], 2) + std::pow(rnew[0] - near[0], 2));
        int steps = (int)(std::ceil(norm) / rez);
        std::vector<double> xi = near;
        int obstacle = 0;
        int unkown = 0;
        for (int i = 0; i < steps; i++)
        {
            std::vector<double> xn = rnew;
            n.clear();
            if (std::sqrt(std::pow(xi[1] - xn[1], 2) + std::pow(xi[0] - xn[0], 2)) <= rez)
            {
                n = xn;
            }
            else
            {
                double m = (xn[1] - xi[1]) / (xn[0] - xi[0]);
                int sign = 1;
                if (xn[0] - xi[0] < 0.0)
                {
                    sign = -1;
                }
                n.push_back((sign * (std::sqrt(std::pow(rez, 2)) / ((std::pow(m, 2)) + 1))) + xi[0]);
                n.push_back(m * (n[0] - xi[0]) + xi[1]);
                if (std::fabs(xn[0] - xi[0]) < 1E-3)
                {
                    n[0] = xi[0];
                    n[1] = xi[1] + rez;
                }
            }
            xi = n;
            int c_data = data[(int)((std::floor((xi[1] - goy) / res) * width) + (std::floor((xi[0] - gox) / res)))];
            if (c_data >= obstacle_cost)
            {
                obstacle = 1;
            }
            if (c_data == -1)
            {
                unkown = 1;
                break;
            }
        }
        rnew = xi;
        int out = 0;
        if (unkown == 1)
        {
            out = -1;
        }
        if (obstacle == 1)
        {
            out = 0;
        }
        if (obstacle != 1 && unkown != 1)
        {
            out = 1;
        }
        if (out == -1)
        {
            point.header.stamp = ros::Time(0);
            point.point.x = rnew[0];
            point.point.y = rnew[1];
            points.points.push_back(point.point);
            shapes.publish(points);
            detected_points.publish(point);
            points.points.clear();
        }
        else if (out == 1)
        {
            v.push_back(rnew);
            p.x = rnew[0];
            p.y = rnew[1];
            line.points.push_back(p);
            p.x = near[0];
            p.y = near[1];
            line.points.push_back(p);
        }
        shapes.publish(line);
        loop_rate.sleep();
    }
    return 0;
}