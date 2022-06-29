#ifndef mnlc_d_star_lite_H
#define mnlc_d_star_lite_H
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/GetPlan.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/GetMap.h>
#include <boost/foreach.hpp>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <functional>
#include <algorithm>
#include "ros/ros.h"
#include <math.h>
#include <map>

#define FTOL 5e-4F

class Node
{

public:
    std::pair<int, int> coords;
    float priority[2] = {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()};
    int index;

    Node()
    {
    }

    void grid_to_index(const int width)
    {
        index = coords.first + width * coords.second;
    }

    void index_to_grid(const int index, const int width)
    {
        this->index = index;
        coords = std::make_pair(index % width, index / width);
    }

    void compute_priority(const float g, const float rhs, const float h, const float km)
    {
        float min = std::min(g, rhs);
        priority[0] = min + h + km;
        priority[1] = min;
    }

    bool operator==(const Node &s) const { return coords.first == s.coords.first && coords.second == s.coords.second; }
    bool operator!=(const Node &s) const { return coords.first != s.coords.first || coords.second != s.coords.second; }
    bool operator<(const Node &s) const { return priority[0] < s.priority[0] || (std::fabs(priority[0] - s.priority[0]) < FTOL && priority[1] < s.priority[1]); }
    bool operator<=(const Node &s) const { return priority[0] < s.priority[0] || (std::fabs(priority[0] - s.priority[0]) < FTOL && priority[1] < s.priority[1]); }
    bool operator>(const Node &s) const { return priority[0] > s.priority[0] || (std::fabs(priority[0] - s.priority[0]) < FTOL && priority[1] > s.priority[1]); }
    bool operator>=(const Node &s) const { return priority[0] > s.priority[0] || (std::fabs(priority[0] - s.priority[0]) < FTOL && priority[1] > s.priority[1]); }
};

class Compare
{
public:
    // Note that the Compare parameter is defined such that it returns true
    // if its first argument comes before its second argument in a weak ordering.
    // But because the priority queue outputs largest elements first,
    // the elements that "come before" are actually output last. That is,
    // the front of the queue contains the "last" element according to the
    // weak ordering imposed by Compare. This means that if the first argument
    // has lower cost, we want Compare to return False, and if the second argument
    // has lower cost, we will return True.
    bool operator()(const Node &s1, const Node &s2) const
    {
        return s1.priority[0] > s2.priority[0] || (s1.priority[0] == s2.priority[0] && s1.priority[1] > s2.priority[1]);
    }
};

class DStarLite
{
private:
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> rhs;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> g;
    std::vector<int8_t, std::allocator<int8_t>> data;
    std::__cxx11::string frame_id;
    std::multiset<Node> queue;
    float heuristic_weight;
    int obstacle_tolerance;
    int search_tolerance;
    nav_msgs::Path path;
    float scan_radius;
    int obstacle_cost;
    int unknown_cost;
    bool verbose;
    int max_its;
    double res;
    double gox;
    double goy;
    int height;
    int width;
    int step;
    float km;

public:
    Node last_update_start;
    Node original_start;
    Node original_goal;
    Node start;
    Node last;
    Node goal;

    DStarLite(const nav_msgs::OccupancyGrid &mapdata, const Node &start, const Node &goal, const int obstacle_cost, const int unknown_cost, const float scan_radius, const bool verbose = true, const float heuristic_weight = 0.5, const int obstacle_tolerance = 0, const int search_tolerance = 0, const int max_its = INT_MAX)
    {
        frame_id = mapdata.header.frame_id;
        res = mapdata.info.resolution;
        gox = mapdata.info.origin.position.x;
        goy = mapdata.info.origin.position.y;
        height = mapdata.info.height;
        width = mapdata.info.width;
        data = mapdata.data;
        this->goal = goal;
        this->start = start;
        this->obstacle_tolerance = obstacle_tolerance;
        this->search_tolerance = search_tolerance;
        this->obstacle_cost = obstacle_cost;
        this->unknown_cost = unknown_cost;
        this->scan_radius = scan_radius;
        this->verbose = verbose;
        this->heuristic_weight = heuristic_weight;
        this->max_its = max_its;
        path.header.frame_id = frame_id;
        last = start;
        original_start = start;
        original_goal = goal;
        last_update_start = start;
        g.resize(height, width);
        rhs.resize(height, width);
        step = 0;
    }

    void init(bool restart = false)
    {
        goal.grid_to_index(width);
        start.grid_to_index(width);
        if (data[start.index] > obstacle_cost)
        {
            ROS_ERROR("Starting point is on an obstacle.");
        }
        if (unfeasible())
        {
            move_goal();
        }
        queue.clear();
        km = 0.0;
        g.setConstant(std::numeric_limits<float>::infinity());
        rhs = g;
        rhs(goal.coords.first, goal.coords.second) = 0.0;
        goal.compute_priority(g(goal.coords.first, goal.coords.second), rhs(goal.coords.first, goal.coords.second), euclidean_distance(goal.coords, start.coords, heuristic_weight), km);
        queue.emplace(goal);
        if (restart)
        {
            original_goal = goal;
            last = start;
            last_update_start = start;
        }
        std::cout << "Successfully initialized D* Lite Planner.\n";
    }

    float euclidean_distance(const std::pair<int, int> &u, const std::pair<int, int> &v, const double weight = 1.0) const
    {
        return weight * std::sqrt(std::pow(u.first - v.first, 2) + std::pow(u.second - v.second, 2));
    }

    float manhattan_distance(const std::pair<int, int> &u, const std::pair<int, int> &v, const double weight = 1.0) const
    {
        return weight * (fabs(u.first - v.first) + fabs(u.second - v.second));
    }

    bool near_obstacle(const std::pair<int, int> &u) const
    {
        for (int i = std::max(0, u.first - obstacle_tolerance); i < std::min(height, u.first + obstacle_tolerance + 1); i++)
        {
            for (int j = std::max(0, u.second - obstacle_tolerance); j < std::min(width, u.second + obstacle_tolerance + 1); j++)
            {
                if (data[grid_to_index(i, j)] >= obstacle_cost)
                {
                    return true;
                }
            }
        }
    }

    std::pair<int, int> available_goal(const std::pair<int, int> &u, const std::pair<int, int> &v) const
    {
        int l = std::max(height - u.first, width - u.second);
        std::map<float, std::pair<int, int>> vps;
        std::pair<int, int> temp;
        for (int rad = 1; rad < l - 1; rad++)
        {
            int u_r[2] = {std::max(0, u.first - rad), std::min(height, u.first + rad)};
            int u_c[2] = {std::max(0, u.second - rad), std::min(width, u.second + rad)};
            for (int col = u_c[0]; col < u_c[1] + 1; col++)
            {
                for (int i = 0; i < 2; i++)
                {
                    temp.first = u_r[i];
                    temp.second = col;
                    if (!near_obstacle(temp))
                    {
                        vps[std::abs(u_r[i] - v.first) + std::abs(col - v.second)] = std::make_pair(u_r[i], col);
                    }
                }
            }
            for (int row = u_r[0] + 1; row < u_r[1]; row++)
            {
                for (int i = 0; i < 2; i++)
                {
                    temp.first = row;
                    temp.second = u_c[i];
                    if (!near_obstacle(temp))
                    {
                        vps[std::abs(row - v.first) + std::abs(u_c[i] - v.second)] = std::make_pair(row, u_c[i]);
                    }
                }
            }
            if (!vps.empty())
            {
                auto point = *(vps.begin());
                return point.second;
            }
        }
        return std::make_pair(-1, -1);
    }

    void move_goal()
    {
        std::cout << "Unfeasible goal, relocating...\n";
        std::pair<int, int> new_goal = available_goal(goal.coords, last_update_start.coords);
        if (new_goal.first < 0)
        {
            ROS_ERROR("Failed to relocate goal to a valid point.");
        }
        else
        {
            std::cout << "Goal moved from [" << goal.coords.first << ", " << goal.coords.second << "] to [" << new_goal.first << ", " << new_goal.second << "].\n";
            goal.coords = new_goal;
            goal.grid_to_index(width);
        }
    }

    bool unfeasible() const
    {
        std::vector<Node> nbrs;
        int x = goal.coords.first;
        int y = goal.coords.second;
        Node nbr;
        for (int i = std::max(0, x - 1); i < std::min(height, x + 2); i++)
        {
            for (int j = std::max(0, y - 1); j < std::min(width, y + 2); j++)
            {
                if (i != x || j != y)
                {
                    nbr.coords = std::make_pair(i, j);
                    nbr.grid_to_index(width);
                    if (data[nbr.index] <= obstacle_cost)
                    {
                        nbrs.push_back(nbr);
                    }
                }
            }
        }
        for (int i = 0; i < nbrs.size(); i++)
        {
            for (int j = 0; j < nbrs.size(); j++)
            {
                if (i != j && (int)manhattan_distance(nbrs[i].coords, nbrs[j].coords) == 1)
                {
                    return false;
                }
            }
        }
        return true;
    }

    bool within_margin(const std::pair<int, int> &u, const std::pair<int, int> &v, int margin = -1) const
    {
        margin = margin >= 0 ? margin : search_tolerance;
        return std::sqrt(std::pow(u.first - v.first, 2) + std::pow(u.second - v.second, 2)) <= margin;
    }

    float compute_cost(const std::pair<int, int> &u) const
    {
        int cost = data[grid_to_index(u.first, u.second)];
        if (cost > obstacle_cost && !within_margin(goal.coords, u))
        {
            return std::numeric_limits<float>::infinity();
        }
        else if (cost == -1)
        {
            return (float)unknown_cost;
        }
        else
        {
            return (float)cost;
        }
    }

    bool no_alternative(const std::pair<int, int> &u, const std::pair<int, int> &v) const
    {
        std::pair<int, int> diff(u.first - v.first, u.second - v.second);
        std::vector<std::pair<int, int>> diffs;
        if (diff.first == 0)
        {
            diffs.emplace_back(std::max(0, v.first - 1), v.second);
            diffs.emplace_back(std::min(height - 1, v.first + 1), v.second);
        }
        if (diff.second == 0)
        {
            diffs.emplace_back(v.first, std::max(0, v.second - 1));
            diffs.emplace_back(v.first, std::min(width - 1, v.second + 1));
        }
        bool out = true;
        for (int i = 0; out && i < diffs.size(); i++)
        {
            out &= near_obstacle(diffs[i]);
        }
        return out;
    }

    void update_vertex(Node &u)
    {
        float gs = g(u.coords.first, u.coords.second);
        float rhss = rhs(u.coords.first, u.coords.second);
        auto it = queue.find(u);
        if (it != queue.end())
        {
            queue.erase(it);
        }
        if (gs != rhss)
        {
            u.compute_priority(gs, rhss, euclidean_distance(u.coords, start.coords, heuristic_weight), km);
            queue.emplace(u);
        }
    }

    float get_g(const std::pair<int, int> &u) const
    {
        return g(u.first, u.second);
    }

    float get_rhs(const std::pair<int, int> &u) const
    {
        return rhs(u.first, u.second);
    }

    void shortest_path()
    {
        printf("Computing shortest path.\n");
        int i = 0;
        Node tmp(*(queue.begin()));
        Node fake_start(start);
        fake_start.compute_priority(get_g(fake_start.coords), get_rhs(fake_start.coords), 0.0, km);
        printf("Top node priority: [%f, %f]\n", tmp.priority[0], tmp.priority[1]);
        printf("Top node has grid cell (%d, %d)\trhs: %f\tg: %f\n",
               tmp.coords.first, tmp.coords.second, get_rhs(tmp.coords), get_g(tmp.coords));
        printf("Start node priority: [%f, %f]\n", fake_start.priority[0], fake_start.priority[1]);
        printf("Start node has grid cell (%d, %d)\trhs: %f\tg: %f\n",
               fake_start.coords.first, fake_start.coords.second, get_rhs(fake_start.coords), get_g(fake_start.coords));
        std::cout << std::boolalpha << (fake_start < tmp) << std::endl; // should return true
        while (1)
        {
            printf("Priority queue size is: %ld!!!!!!!!!!!!!!!!!\n", queue.size());
            fake_start.compute_priority(g(start.coords.first, start.coords.second), rhs(start.coords.first, start.coords.second), 0.0, km);
            auto it = queue.begin();
            Node k_old(*it);
            Node k_new(*it);
            k_new.compute_priority(get_g(k_new.coords), get_rhs(k_new.coords),
                                   euclidean_distance(k_new.coords, start.coords, heuristic_weight), km);
            printf("Start node is grid cell (%d, %d)\trhs: %f\tg: %f\n",
                   start.coords.first, start.coords.second, get_rhs(start.coords), get_g(start.coords));
            printf("Priority of Start is: [%f, %f].\n", start.priority[0], start.priority[1]);
            printf("k_new node is grid cell (%d, %d)\trhs: %f\tg: %f\n",
                   k_new.coords.first, k_new.coords.second, get_rhs(k_new.coords), get_g(k_new.coords));
            printf("Priority of k_new is: [%f, %f], priority of k_old is: [%f, %f].\n", k_new.priority[0], k_new.priority[1], k_old.priority[0], k_old.priority[1]);
            if (!(fake_start > k_old || (get_rhs(start.coords) != get_g(start.coords))))
            {
                break;
            }
            if (k_new > k_old)
            {
                printf("k_new is greater than k_old!!");
                queue.erase(it);
                queue.emplace(k_new);
            }
            else if (get_g(k_new.coords) > get_rhs(k_new.coords))
            {
                printf("g(k_new) > rhs(k_new)\n");
                g(k_new.coords.first, k_new.coords.second) = get_rhs(k_new.coords);
                queue.erase(it);
                int x = k_new.coords.first;
                int y = k_new.coords.second;
                Node nbr;
                for (int i = std::max(0, x - 1); i < std::min(height, x + 2); i++)
                {
                    for (int j = std::max(0, y - 1); j < std::min(width, y + 2); j++)
                    {
                        if (i != x || j != y)
                        {
                            nbr.coords = std::make_pair(i, j);
                            nbr.grid_to_index(width);
                            if (nbr.index != goal.index)
                            {
                                rhs(nbr.coords.first, nbr.coords.second) = std::min(get_rhs(nbr.coords), compute_cost(k_new.coords) + get_g(k_new.coords));
                            }
                            update_vertex(nbr);
                        }
                    }
                }
            }
            // c(s, u) is the cost of the second argument
            else
            {
                printf("IM IN THE ELSE BLOCK!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                float g_old = get_g(k_new.coords);
                g(k_new.coords.first, k_new.coords.second) = std::numeric_limits<float>::infinity();
                int x1 = k_new.coords.first;
                int y1 = k_new.coords.second;
                Node nbr1;
                for (int i = std::max(0, x1 - 1); i < std::min(height, x1 + 2); i++)
                {
                    for (int j = std::max(0, y1 - 1); j < std::min(width, y1 + 2); j++)
                    {
                        nbr1.coords = std::make_pair(i, j);
                        nbr1.grid_to_index(width);
                        if (std::fabs(get_rhs(nbr1.coords) - (compute_cost(k_new.coords) + g_old)) < FTOL)
                        {
                            if (i != goal.coords.first || j != goal.coords.second)
                            {
                                int x2 = nbr1.coords.first;
                                int y2 = nbr1.coords.second;
                                Node nbr2;
                                float min = std::numeric_limits<float>::infinity();
                                float cost;
                                for (int m = std::max(0, x2 - 1); m < std::min(height, x2 + 2); m++)
                                {
                                    for (int n = std::max(0, y2 - 1); n < std::min(width, y2 + 2); n++)
                                    {
                                        if ((m != x2 || n != y2))
                                        {
                                            nbr2.coords = std::make_pair(m, n);
                                            nbr2.grid_to_index(width);
                                            cost = compute_cost(nbr2.coords) + get_g(nbr2.coords);
                                            // printf("Cost of grid cell (%d, %d) at index %d is: %f\n", nbr.coords.first, nbr.coords.second, nbr.index, cost);
                                            if (cost < min)
                                            {
                                                min = cost;
                                            }
                                        }
                                    }
                                }
                                rhs(nbr1.coords.first, nbr1.coords.second) = min;
                            }
                        }
                        update_vertex(nbr1);
                    }
                }
            }
            i++;
            if (verbose)
            {
                std::cout << "Shortest path step: " << i << " completed.\n";
            }
        }
    }

    void scan(Node &_last, const std::vector<int8_t, std::allocator<int8_t>> new_data)
    {
        std::vector<int> changed_edges;
        int r_region = (int)(scan_radius / res);
        int init_index = last.index - r_region * (width + 1);
        for (int j = 0; j < 2 * r_region + 1; j++)
        {
            int start = j * width + init_index;
            int end = start + 2 * r_region;
            int limit = ((start / width) + 2) * width;
            for (int k = start; k < end + 1; k++)
            {
                if (k >= 0 && k < limit && k < new_data.size() && data[k] != new_data[k])
                {
                    changed_edges.push_back(k);
                }
            }
        }
        if (changed_edges.size() > 0)
        {
            if (unfeasible())
            {
                move_goal();
                init(true);
            }
            else
            {
                km += euclidean_distance(_last.coords, start.coords);
                Node node;
                for (int i = 0; i < changed_edges.size(); i++)
                {
                    int k = changed_edges[i];
                    node.index_to_grid(k, width);
                    float c_old = data[k];
                    data[k] = new_data[k];
                    float c_new = compute_cost(node.coords);
                    if (c_old > c_new && node.index != goal.index)
                    {
                        rhs(node.coords.first, node.coords.second) = std::min(get_rhs(node.coords), c_new + get_g(node.coords));
                    }
                    else if (std::fabs(get_rhs(node.coords) - (c_old + get_g(node.coords)) < FTOL) && node.index != goal.index)
                    {
                        int x = node.coords.first;
                        int y = node.coords.second;
                        float min = std::numeric_limits<float>::infinity();
                        float cost;
                        Node nbr;
                        for (int i = std::max(0, x - 1); i < std::min(height, x + 2); i++)
                        {
                            for (int j = std::max(0, y - 1); j < std::min(width, y + 2); j++)
                            {
                                if (i != x || j != y)
                                {
                                    nbr.coords = std::make_pair(i, j);
                                    nbr.grid_to_index(width);
                                    cost = compute_cost(nbr.coords) + get_g(nbr.coords);
                                    // printf("Cost of grid cell (%d, %d) at index %d is: %f\n", nbr.coords.first, nbr.coords.second, nbr.index, cost);
                                    if (cost < min)
                                    {
                                        min = cost;
                                    }
                                }
                            }
                        }
                        rhs(node.coords.first, node.coords.second) = min;
                    }
                    update_vertex(node);
                }
            }
            _last = start;
            shortest_path();
        }
    }

    Node step_in(std::vector<int8_t, std::allocator<int8_t>> &new_data)
    {
        Node out;
        if (step == 0)
        {
            last = start;
            scan(last, new_data);
            shortest_path();
            out = start;
        }
        else if (step < max_its)
        {
            out.coords = std::make_pair(-1, -1);
            scan(last, new_data);
            if (euclidean_distance(goal.coords, start.coords) > search_tolerance)
            {
                if (verbose)
                {
                    std::cout << "Current location: [" << start.coords.first << ", " << start.coords.second << "].\n";
                }
                int x = start.coords.first;
                int y = start.coords.second;
                float min = std::numeric_limits<float>::infinity();
                float cost;
                Node nbr;
                for (int i = std::max(0, x - 1); i < std::min(height, x + 2); i++)
                {
                    for (int j = std::max(0, y - 1); j < std::min(width, y + 2); j++)
                    {
                        if (i != x || j != y)
                        {
                            nbr.coords = std::make_pair(i, j);
                            nbr.grid_to_index(width);
                            cost = compute_cost(nbr.coords) + get_g(nbr.coords);
                            // printf("Cost of grid cell (%d, %d) at index %d is: %f\n", nbr.coords.first, nbr.coords.second, nbr.index, cost);
                            if (cost < min)
                            {
                                min = cost;
                                out = nbr;
                            }
                        }
                    }
                }
                if (out.coords.first < 0)
                {
                    ROS_ERROR("Could not find a feasible path.");
                }
                start = out;
            }
        }
        else
        {
            ROS_ERROR("Could not find a feasible path in the given number of steps.");
        }
        step++;
        return out;
    }

    void construct_path(std::vector<int8_t, std::allocator<int8_t>> &new_data)
    {
        Node _step = step_in(new_data);
        path.header.stamp = ros::Time::now();
        geometry_msgs::PoseStamped step_pose;
        step_pose.header.frame_id = frame_id;
        step_pose.pose.orientation.w = 1.0;
        while (_step.coords.first != -1)
        {
            step_pose.header.stamp = ros::Time::now();
            step_pose.pose.position.x = _step.coords.first;
            step_pose.pose.position.y = _step.coords.second;
            path.poses.push_back(step_pose);
            if (path.poses.size() > 2 && path.poses.back() == path.poses[path.poses.size() - 3])
            {
                break;
            }
            _step = step_in(new_data);
        }
    }

    int grid_to_index(const int x, const int y) const
    {
        return x + width * y;
    }

    std::pair<int, int> index_to_grid(const int index) const
    {
        return std::make_pair(index % width, index / width);
    }

    std::pair<int, int> index_to_world(const int index) const
    {
        return std::make_pair(((index % width) * res) + gox + (res / 2), ((index / width) * res) + goy + (res / 2));
    }
};

nav_msgs::OccupancyGrid mapdata;
_Float32 heuristic_weight;
int obstacle_tolerance;
int search_tolerance;
_Float32 scan_radius;
_Float32 start_time;
int obstacle_cost;
_Float32 timeout;
int unknown_cost;
int max_its;

#endif