#ifndef DSTARLITEROS_H
#define DSTARLITEROS_H

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
#include <inttypes.h>
#include <functional>
#include <algorithm>
#include "ros/ros.h"
#include "planner.h"
#include <math.h>
#include "map.h"
#include <map>

#include <iostream>
#include <fstream>

nav_msgs::OccupancyGrid mapdata;
nav_msgs::OccupancyGrid rhs_map;
nav_msgs::OccupancyGrid h_map;
nav_msgs::OccupancyGrid g_map;
_Float32 heuristic_weight;
int obstacle_tolerance;
nav_msgs::Path path_p;
int search_tolerance;
_Float32 scan_radius;
_Float32 start_time;
int obstacle_cost;
_Float32 timeout;
int unknown_cost;
int max_its;

using namespace DStarLite;

namespace DStarLite
{
    class DStarLiteROS
    {
    public:
        /**
         * Config class.
         */
        class Config
        {
        public:
            /**
             * @var nav_msgs::OccupancyGrid initial mapdata
             */
            nav_msgs::OccupancyGrid _mapdata;

            /**
             * @var pair<double, double> initial start position
             */
            pair<double, double> _start;

            /**
             * @var pair<int, int> initial goal position
             */
            pair<int, int> _goal;

            /**
             * @var int obstacle cost
             */
            int _obstacle_cost;

            /**
             * @var int unknown cost
             */
            int _unknown_cost;

            /**
             * @var float scan radius
             */
            float _scan_radius;

            /**
             * @var bool debug verbose
             */
            bool _verbose;

            /**
             * @var float heuristic weight
             */
            float _heuristic_weight;

            /**
             * @var int obstacle tolerance
             */
            int _obstacle_tolerance;

            /**
             * @var int search tolerance
             */
            int _search_tolerance;

            /**
             * @var int maximum iterations
             */
            int _max_its;

            /**
             * Constructor.
             *
             * @param nav_msgs::OccupancyGrid initial mapdata
             * @param pair<int, int> initial start position
             * @param pair<int, int> initial goal position
             * @param int obstacle cost
             * @param int unknown cost
             * @param float scan radius
             * @param bool debug verbose
             * @param float heuristic weight
             * @param int obstacle tolerance
             * @param int search tolerance
             * @param int maximum iterations
             */
            Config(const nav_msgs::OccupancyGrid &mapdata, const pair<double, double> start, const pair<int, int> &goal,
                   const int obstacle_cost, const int unknown_cost, const float scan_radius, const bool verbose, const float heuristic_weight, const int obstacle_tolerance,
                   const int search_tolerance, const int max_its)
            {
                _mapdata = mapdata;
                _start = (make_pair(start.first, start.second));
                _goal = goal;
                _obstacle_cost = obstacle_cost;
                _unknown_cost = unknown_cost;
                _scan_radius = scan_radius;
                _verbose = verbose;
                _heuristic_weight = heuristic_weight;
                _obstacle_tolerance = obstacle_tolerance;
                _search_tolerance = search_tolerance;
                _max_its = max_its;
            }

            /**
             * Constructor.
             *
             * */
            Config()
            {
            }
        };

    protected:
        /**
         * @var Config ROS interface config options
         */
        Config _config;

        /**
         * @var bool used to signal when the map has been updated
         */
        bool _map_updated;

        /**
         * @var std::vector<int8_t, std::allocator<int8_t>> mapdata data
         */
        std::vector<int8_t, std::allocator<int8_t>> _data;

        /**
         * @var Map* map used internally
         */
        Map *_map;

        /**
         * @var vector<pair<double, double>> latest path
         */
        vector<pair<double, double>> _path;

        /**
         * @var Planner* planner
         */
        Planner *_planner;

    public:
        /**
         * @var vector<double> &cellcosts with indices representing original cell costs which map to non-linearly spaced cell costs
         */
        vector<double> _cellcosts;

        /**
         * @var vector<vector<vector<vector<double>>>> &interpolation lookup table for quickly aquiring cell costs and optimal XY given a cell and two consecutive neighbors.
         */
        vector<vector<vector<vector<double>>>> _I;

        /**
         * Constructor.
         *
         * @param Config config options
         */
        DStarLiteROS(Config config)
        {
            // Set config
            _config = config;

            // Map has just been initialized
            _map_updated = true;

            // Start with a fresh path
            _path.clear();

            // Initial mapdata data
            _data = _config._mapdata.data;

            // Height and width of the initial occupancy grid
            int height, width;
            height = _config._mapdata.info.height;
            width = _config._mapdata.info.width;

            // Make the map
            _map = new Map(height, width);

            // Set current and goal position
            Map::Cell *_current = (*_map)(static_cast<int>(_config._start.second), static_cast<int>(_config._start.first));
            Map::Cell *_goal = (*_map)(_config._goal.second, _config._goal.first);

            {
                int k, c;

                // Build map
                for (int i = 0; i < height; i++)
                {
                    for (int j = 0; j < width; j++)
                    {
                        k = (i * width) + j;
                        c = _data[k];

                        // Cell is unwalkable
                        if (c > _config._obstacle_cost)
                            c = _config._obstacle_cost;
                        else if (c < 0)
                            c = _config._unknown_cost;

                        (*_map)(i, j)->cost = c;
                    }
                }
            }

            int Nc = _config._obstacle_cost;

            // Initializes cell cost table, which indices representing original cell costs which map to non-lineraly space cell costs
            {
                _cellcosts.resize(Nc + 1);

                for (int i = 0; i < Nc; i++)
                    _cellcosts[i] = round(255.0 * pow(Math::EUL, ((0.33333 * static_cast<double>(i)) / 11.8125)) - 250.0);
                _cellcosts[Nc] = Map::Cell::COST_UNWALKABLE;
                int Mc = static_cast<int>(_cellcosts[Nc - 1]);

                // Generates interpolation lookup table for quickly aquiring cell costs.
                _I.resize(Nc + 1);
                for (int i = 0; i < Nc + 1; i++)
                {
                    _I[i].resize(Nc + 1);
                    for (int j = 0; j < Nc + 1; j++)
                    {
                        _I[i][j].resize(Mc + 1);
                        for (int k = 0; k < Mc + 1; k++)
                            _I[i][j][k].resize(3);
                    }
                }

                int ci, bi, f;
                double c, b, x, y;

                ci = 0;

                while (ci < Nc)
                {
                    c = _cellcosts[ci];
                    bi = 0;

                    while (bi < Nc)
                    {
                        b = _cellcosts[bi];
                        f = 1;
                        while (f <= Mc)
                        {
                            if (Math::less(static_cast<double>(f), b))
                            {
                                if (Math::less(c, static_cast<double>(f)) || Math::equals(c, static_cast<double>(f)))
                                {
                                    _I[ci][bi][f][0] = c * Math::SQRT2;
                                    _I[ci][bi][f][1] = 1.0;
                                    _I[ci][bi][f][2] = 0.0;
                                }
                                else
                                {
                                    y = min(static_cast<double>(f) / (sqrt(pow(c, 2) - pow(static_cast<double>(f), 2))), 1.0);
                                    _I[ci][bi][f][0] = (c * sqrt(1.0 + pow(y, 2))) + (static_cast<double>(f) * (1.0 - y));
                                    _I[ci][bi][f][1] = 1.0;
                                    _I[ci][bi][f][2] = y;
                                }
                            }
                            else
                            {
                                if (Math::less(c, b) || Math::equals(c, b))
                                {
                                    _I[ci][bi][f][0] = c * Math::SQRT2;
                                    _I[ci][bi][f][1] = 1.0;
                                    _I[ci][bi][f][2] = 0.0;
                                }
                                else
                                {
                                    x = 1.0 - min(b / (sqrt(pow(c, 2) - pow(b, 2))), 1.0);
                                    _I[ci][bi][f][0] = (c * sqrt(1.0 + pow((1.0 - x), 2))) + (b * x);
                                    _I[ci][bi][f][1] = x;
                                    _I[ci][bi][f][2] = 0.0;
                                }
                            }
                            f++;
                        }
                        bi++;
                    }
                    ci++;
                }
            }

            // Make planner
            _planner = new Planner(_map, _current, _goal, _cellcosts, _I);
        }
        /**
         * Deconstructor.
         */
        ~DStarLiteROS()
        {
            delete _map;
            delete _planner;
        }

        /**
         * Main execution method.
         *
         * @param pair<double, double> current position in grid coordinates
         * @return vector<pair<double, double>> path
         */
        vector<pair<double, double>> execute(pair<double, double> current)
        {
            // uint64_t timer_start, timer_end;
            // timer_start = ros::Time::now().toNSec();

            // double timer_start, timer_end;
            // timer_start = ros::Time::now().toSec();

            // Step
            _planner->start(current);

            // Check if map was updated
            if (_map_updated)
            {
                _map_updated = false;
                // Replan the path

                if (!_planner->replan())
                    ROS_ERROR("No Solution Found!");

                // timer_end = ros::Time::now().toNSec();
                // printf("Execute Field D* took: %" PRIu64 "\n", timer_end - timer_start);

                // timer_end = ros::Time::now().toSec();
                // printf("Execute Field D* took: %lf\n", timer_end - timer_start);

                _path = _planner->path();
            }
            return _path;
        }

        /**
         * Scans map for updated cells.
         *
         * @param std::vector<int8_t, std::allocator<int8_t>> new data
         */
        void update_map(std::vector<int8_t, std::allocator<int8_t>> new_data)
        {
            // uint64_t timer_start, timer_end;
            // timer_start = ros::Time::now().toNSec();

            // double timer_start, timer_end;
            // timer_start = ros::Time::now().toSec();

            bool unknown_flip;
            _map_updated = false;
            int rows, cols;
            rows = _map->rows();
            cols = _map->cols();
            int k, c, c_old;

            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    k = (i * cols) + j;
                    c_old = _data[k];
                    c = new_data[k];

                    // Check if an update is required
                    if (c_old != c)
                    {
                        _map_updated = true;
                        _data[k] = new_data[k];

                        if (c_old < 0 || (c_old >= 0 && c < 0))
                        {
                            if (c < 0)
                                c = _config._unknown_cost;
                            unknown_flip = true;
                        }
                        else
                            unknown_flip = false;

                        // Cell is unwalkable
                        if (c > _config._obstacle_cost)
                            c = _config._obstacle_cost;

                        _planner->update_cell_cost((*_map)(i, j), c, unknown_flip);
                    }
                }
            }

            // if (_map_updated)
            // {
            //     // timer_end = ros::Time::now().toNSec();
            //     // printf("Update map of Field D* took: %" PRIu64 "\n", timer_end - timer_start);

            //     timer_end = ros::Time::now().toSec();
            //     printf("Update map of Field D* took: %lf\n", timer_end - timer_start);
            // }
        }

        /**
         * Returns a path-to-goal map
         *
         * @return vector<double> map
         */
        vector<double> get_g_map()
        {
            return _planner->g_map();
        }

        /**
         * Returns a distance-to-start map
         *
         * @return vector<double> map
         */
        vector<double> get_h_map()
        {
            return _planner->h_map();
        }

        /**
         * Returns a lookahead of the path-to-goal map
         *
         * @return vector<double> map
         */
        vector<double> get_rhs_map()
        {
            return _planner->rhs_map();
        }
    };
};
#endif // DSTARLITEROS_H