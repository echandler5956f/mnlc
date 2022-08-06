#ifndef DSTARLITE_PLANNER_H
#define DSTARLITE_PLANNER_H

#include <list>
#include <map>
#ifdef WIN32
#include <unordered_map>
#else
#include <tr1/unordered_map>
#endif
#include "map.h"
#include "math.h"
#include <bits/stdc++.h>

using namespace std;
using namespace DStarLite;

namespace DStarLite
{
	class Planner
	{
	public:
		/**
		 * Key compare struct.
		 */
		struct KeyCompare : public binary_function<pair<double, double>, pair<double, double>, bool>
		{
			bool operator()(const pair<double, double> &p1, const pair<double, double> &p2) const;
		};

		/**
		 * Path Key compare struct.
		 */
		struct PKeyCompare : public binary_function<double, double, bool>
		{
			bool operator()(const double &d1, const double &d2) const;
		};

		/**
		 * @var static const double [radians] minimum backpointer gradient angle before resorting to one-step lookahead
		 */
		static const double GRAD_ANG_MIN;

		/**
		 * @var static const double [radians] maximum backpointer gradient angle before resorting to one-step lookahead
		 */
		static const double GRAD_ANG_MAX;

		/**
		 * @var static const int max steps before assuming no solution possible
		 */
		static const int MAX_STEPS;

		/**
		 * @var static const int max steps before giving up on path extraction
		 */
		static const int MAX_EXTRACTION_STEPS;

		/**
		 * Constructor.
		 *
		 * @param Map* map
		 * @param Map::Cell* start cell
		 * @param Map::Cell* goal cell
		 * @param int Nc
		 */
		Planner(Map *map, Map::Cell *start, Map::Cell *goal, int Nc);

		/**
		 * Deconstructor.
		 */
		~Planner();

		/**
		 * Returns the generated path.
		 *
		 * @return vector<pair<double, double>> path
		 */
		vector<pair<double, double>> path();

		/**
		 * Gets/Sets a new goal.
		 *
		 * @param Map::Cell* [optional] goal
		 * @return Map::Cell* new goal
		 */
		Map::Cell *goal(Map::Cell *u = nullptr);

		/**
		 * Returns a path-to-goal map
		 *
		 * @return vector<double> map
		 */
		vector<double> g_map();

		/**
		 * Replans the path.
		 *
		 * @return bool solution found
		 */
		bool replan();

		/**
		 * Returns a lookahead of the path-to-goal map
		 *
		 * @return vector<double> map
		 */
		vector<double> rhs_map();

		/**
		 * Gets/Sets start.
		 *
		 * @param Map::Cell* [optional] new start
		 * @return Map::Cell* start
		 */
		Map::Cell *start(Map::Cell *u = nullptr);

		/**
		 * Update map.
		 *
		 * @param Map::Cell* cell to update
		 * @param int new cost of the cell
		 * @return void
		 */
		void update_cell_cost(Map::Cell *u, int cost);

	protected:
		/**
		 * @var unordered_map cell hash (keeps track of all the cells)
		 */
		typedef tr1::unordered_map<Map::Cell *, pair<double, double>, Map::Cell::Hash> CH;
		CH _cell_hash;

		/**
		 * @var Map* map
		 */
		Map *_map;

		/**
		 * @var vector<pair<double, double>> interpolated path
		 */
		vector<pair<double, double>> _interpol_path;

		/**
		 * @var multimap open list
		 */
		typedef pair<pair<double, double>, Map::Cell *> OL_PAIR;
		typedef multimap<pair<double, double>, Map::Cell *, KeyCompare> OL;
		OL _open_list;

		/**
		 * @var unordered_map open hash (stores position in open multimap)
		 */
		typedef tr1::unordered_map<Map::Cell *, OL::iterator, Map::Cell::Hash> OH;
		OH _open_hash;

		/**
		 * @var multimap path list
		 */
		typedef pair<double, Map::CellPath *> PL_PAIR;
		typedef multimap<double, Map::CellPath *, PKeyCompare> PL;
		PL _primary_cp_list;
		PL _secondary_cp_list;

		/**
		 * @var unordered_map path hash (stores position in path multimap)
		 */
		typedef tr1::unordered_map<Map::CellPath *, PL::iterator, Map::CellPath::Hash> PH;
		PH _primary_cp_hash;
		PH _secondary_cp_hash;

		/**
		 * @var Map::Cell* start and goal tile
		 */
		Map::Cell *_start;
		Map::Cell *_goal;

		/**
		 * @var int Nc: all distinct traversal costs including infinite cost of traversing an obstacle and Mc: distinct traversable cell costs
		 */
		int _Nc, _Mc;

		/**
		 * @var vector<double> cellcosts with indices representing original cell costs which map to non-linearly spaced cell costs
		 */
		vector<double> _cellcosts;

		/**
		 * @var vector<vector<vector<vector<double>>>> interpolation lookup table for quickly aquiring cell costs and optimal XY given a cell and two consecutive neighbors.
		 */
		vector<vector<vector<vector<double>>>> _I;

		/** Calculates the cost of traversing the path
		 *
		 * @return double the path cost
		 */
		double _calculate_path_cost(vector<pair<double, double>> &path);

		/**
		 * Generates a cell.
		 *
		 * @param Map::Cell*
		 * @return void
		 */
		void _cell(Map::Cell *u);

		/**
		 * Initializes cell cost table, which indices representing original cell costs which map to non-lineraly space cell costs
		 *
		 * @return int Mc maximum traversable (non-obstacle) cost
		 */
		int _construct_cellcosts();

		/**
		 * Generates interpolation lookup table for quickly aquiring cell costs.
		 *
		 * @return void
		 */
		void _construct_interpolation_table();

		/**
		 * Calculates the interpolated cost of 's' given any two consecutive neighbors 'sa' and 'sb'.
		 *
		 * @param Map::Cell* cell s
		 * @param Map::Cell* cell sa
		 * @param Map::Cell* cell sb
		 * @return double cost of s
		 */
		double _compute_cost(Map::Cell *s, Map::Cell *sa, Map::Cell *sb);

		/**
		 * Calculates the interpolated [x, y] position of a backpointer of 's' given any two consecutive neighbors 'sa' and 'sb'.
		 *
		 * @param Map::Cell* cell s
		 * @param Map::Cell* cell sa
		 * @param Map::Cell* cell sb
		 * @return pair<pair<double, double>, int> position of bptr of s and type of path (1 = direct, 2 = more than one path segment)
		 */
		pair<pair<double, double>, int> _compute_bp(Map::Cell *s, Map::Cell *sa, Map::Cell *sb);

		/**
		 * This computes the minimum cost from an arbitrary point inside cell c to
		 * the edge (of c) that connects nodes s_1 and s_2. The node s is also
		 * located on a corner of c, and defines a neighboring cell b that can
		 * also be used en-route to the edge (s_a s_b). Assuming that s is the
		 * origin of a coordinate system where the y axis starts at node s and goes
		 * along the edge between c and b, and the x axis starts at s and goes
		 * along the other edge of c, and that side lengths of c are 1,
		 * (y_hat, x_hat) is the location of the arbitrary point
		 * this returns a CellPath containing relevant info
		 *
		 * @param double y_hat y coordinate of point
		 * @param double x_hat x coordinate of point
		 * @param Map::Cell* s
		 * @param Map::Cell* s_a
		 * @param Map::Cell* s_b
		 * @param Map::CellPath** sub_paths
		 */
		void _compute_cost_of_point_to_edge(double y_hat, double x_hat, Map::Cell *s, Map::Cell *s_a, Map::Cell *s_b, Map::CellPath **sub_paths);

		/**
		 * Computes the local cost of getting from the point s' to the goal, s' is
		 * described first in terms of the cell that contains it [cell_y, cell_x]
		 * and then in terms of x and y both in the range [0 1], describing the
		 * local position of the point within the cell. Puts cellPaths into a heap
		 * so that they are sorted by best cost
		 *
		 * @param int cy cell's global y value
		 * @param int cx cell's global x value
		 * @param double y point's local y value
		 * @param double x point's local x value
		 * @param bool true to use primary path list, false to use secondary
		 */
		void _compute_local_point_cost(int cy, int cx, double y, double x, bool primary);

		/**
		 * Computes the local point cost for each possible cell and puts local paths into the heap
		 *
		 * @param double py
		 * @param double px
		 * @param bool true to use primary path list, false to use secondary

		 */
		void _compute_local_point_costs(double py, double px, bool primary);

		/**
		 * Computes shortest interpolated path.
		 *
		 * @return bool successful
		 */
		bool _compute_shortest_path();

		/**
		 * Extracts interpolated path using a combination of a backpointer gradient and a one step lookahead
		 *
		 * @return bool successful
		 */
		bool _extract_path();

		/**
		 * Gets/Sets g value for a cell.
		 *
		 * @param Map::Cell* cell to retrieve/update
		 * @param double [optional] new g value
		 * @return double g value
		 */
		double _g(Map::Cell *u, double value = DBL_MIN);

		/** Gets the path of backpointers that links sa to sb in the graph
		 *
		 * @param vector<pair<double, double>> &path
		 * @param Map::Cell * first cell
		 * @param Map::Cell * second cell
		 * @return bool successful
		 */
		bool _get_path(vector<pair<double, double>> &path, Map::Cell *s_a, Map::Cell *s_b);

		/**
		 * Calculates heuristic between two cells (euclidean distance).
		 *
		 * @param Map::Cell* cell a
		 * @param Map::Cell* cell b
		 * @return double heuristic value
		 */
		double _h(Map::Cell *a, Map::Cell *b);

		/**
		 * Calculates key value for cell.
		 *
		 * @param Map::Cell* cell to calculate for
		 * @return pair<double,double> key value
		 */
		pair<double, double> _k(Map::Cell *u);

		/**
		 * Inserts cell into open list.
		 *
		 * @param Map::Cell* cell to insert
		 * @param pair<double,double> key value for the cell
		 * @return void
		 */
		void _list_insert(Map::Cell *u, pair<double, double> k);

		/**
		 * Removes cell from the open list.
		 *
		 * @param Map::Cell* cell to remove
		 * @return void
		 */
		void _list_remove(Map::Cell *u);

		/**
		 * Updates cell in the open list.
		 *
		 * @param Map::Cell*
		 * @param pair<double,double>
		 * @return void
		 */
		void _list_update(Map::Cell *u, pair<double, double> k);

		/**
		 * Finds the minimum successor cell using interpolated costs.
		 *
		 * @param Map::Cell* root
		 * @return <Map::Cell*,double> successor
		 */
		pair<Map::Cell *, double> _min_interpol_succ(Map::Cell *u);

		/**
		 * Inserts cell into the primary path list.
		 *
		 * @param Map::CellPath* cell path to insert
		 * @param double key value for the cell path
		 * @return void
		 */
		void _path_p_list_insert(Map::CellPath *p, double k);

		/**
		 * Removes cell path from the primary path list.
		 *
		 * @param Map::CellPath* cell path to remove
		 * @return void
		 */
		void _path_p_list_remove(Map::CellPath *p);

		/**
		 * Updates cell path in the primary path list.
		 *
		 * @param Map::CellPath* sub_path to update the key of
		 * @param double k new key
		 * @return void
		 */
		void _path_p_list_update(Map::CellPath *p, double k);

		/**
		 * Inserts cell into the secondary path list.
		 *
		 * @param Map::CellPath* cell path to insert
		 * @param double key value for the cell path
		 * @return void
		 */
		void _path_s_list_insert(Map::CellPath *p, double k);

		/**
		 * Removes cell path from the secondary path list.
		 *
		 * @param Map::CellPath* cell path to remove
		 * @return void
		 */
		void _path_s_list_remove(Map::CellPath *p);

		/**
		 * Updates cell path in the secondary path list.
		 *
		 * @param Map::CellPath* sub_path to update the key of
		 * @param double k new key
		 * @return void
		 */
		void _path_s_list_update(Map::CellPath *p, double k);

		/** Removes all but the first point in a sub-path containing repeated points
		 *
		 * @return vector<pair<double, double>> &path
		 */
		void _remove_repeated_points(vector<pair<double, double>> &path);

		/**
		 * Gets/Sets rhs value for a cell.
		 *
		 * @param Map::Cell* cell to retrieve/update
		 * @param double [optional] new rhs value
		 * @return double rhs value
		 */
		double _rhs(Map::Cell *u, double value = DBL_MIN);

		/**
		 * Updates cell.
		 *
		 * @param Map::Cell* cell to update
		 * @return void
		 */
		void _update_state(Map::Cell *u);
	};
};

#endif // DSTARLITE_PLANNER_H