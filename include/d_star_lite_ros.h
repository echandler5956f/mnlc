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

nav_msgs::OccupancyGrid mapdata;
nav_msgs::OccupancyGrid rhs_map;
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
             * @var pair<int, int> initial start position
             */
            pair<int, int> _start;
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
            Config(const nav_msgs::OccupancyGrid &mapdata, const pair<int, int> &start, const pair<int, int> &goal,
                   const int obstacle_cost, const int unknown_cost, const float scan_radius, const bool verbose, const float heuristic_weight, const int obstacle_tolerance,
                   const int search_tolerance, const int max_its)
            {
                _mapdata = mapdata;
                _start = start;
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
            Map::Cell *_current = (*_map)(_config._start.second, _config._start.first);
            Map::Cell *_goal = (*_map)(_config._goal.second, _config._goal.first);
            int k, c, v;

            // Build map
            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    k = (i * width) + j;
                    c = _data[k];

                    // Cell is unwalkable
                    if (c >= _config._obstacle_cost)
                    {
                        v = _config._obstacle_cost;
                    }
                    else if (c < 0)
                    {
                        v = _config._unknown_cost;
                    }
                    else
                    {
                        v = c;
                    }

                    (*_map)(i, j)->cost = v;
                }
            }

            // Make planner
            _planner = new Planner(_map, _current, _goal, _config._obstacle_cost);
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
         * @param Map::Cell* current cell
         * @return vector<pair<double, double>> path
         */
        vector<pair<double, double>> execute(pair<int, int> current)
        {
            // Step
            _planner->start((*_map)(current.second, current.first));

            // Check if map was updated
            if (_map_updated)
            {
                _map_updated = false;
                // Replan the path

                if (!_planner->replan())
                {
                    ROS_ERROR("No Solution Found!");
                }

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
            _map_updated = false;
            int rows, cols;
            rows = _map->rows();
            cols = _map->cols();
            int k, c, v;

            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    k = (i * cols) + j;
                    // Check if an update is required
                    if (_data[k] != new_data[k])
                    {
                        _map_updated = true;
                        _data[k] = new_data[k];
                        c = _data[k];

                        // Cell is unwalkable
                        if (c >= _config._obstacle_cost)
                        {
                            v = _config._obstacle_cost;
                        }
                        else if (c < 0)
                        {
                            v = _config._unknown_cost;
                        }
                        else
                        {
                            v = c;
                        }
                        _planner->update_cell_cost((*_map)(i, j), v);
                    }
                }
            }
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