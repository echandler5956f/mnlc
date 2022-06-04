#ifndef mnlc_a_star_H
#define mnlc_a_star_H
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
#include <functional>
#include <algorithm>
#include "ros/ros.h"
#include <math.h>
#include <queue>
#include <map>

nav_msgs::OccupancyGrid mapdata;
nav_msgs::GridCells frontier_vis;
_Float32 start_time;
_Float32 timeout;
int obstacle_cost;

class Node
{
public:
    int x;
    int y;
    float priority = 0;
    Node()
    {
    }
    Node(int X, int Y)
    {
        x = X;
        y = Y;
    }
    Node(int X, int Y, float Priority)
    {
        x = X;
        y = Y;
        priority = Priority;
    }
};

// Note that the Compare parameter is defined such that it returns true
// if its first argument comes before its second argument in a weak ordering.
// But because the priority queue outputs largest elements first,
// the elements that "come before" are actually output last. That is,
// the front of the queue contains the "last" element according to the
// weak ordering imposed by Compare. This means that if the first argument
// has lower cost, we want Compare to return False, and if the second argument
// has lower cost, we will return True.
bool Compare(Node a, Node b)
{
    return a.priority > b.priority;
}

#endif