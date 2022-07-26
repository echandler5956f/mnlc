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
		 * @var static const double max steps before assuming no solution possible
		 */
		static const double MAX_STEPS;

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
		 * Replans the path.
		 *
		 * @return bool solution found
		 */
		bool replan();

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
		 * @var vector<Map::Cell*> path
		 */
		vector<Map::Cell *> _path;

		/**
		 * @var unordered_set<Map::Cell*> path set
		 * 
		 */
		unordered_set<Map::Cell *> _path_set;

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
		 * @var unordered_map open hash (stores position in multimap)
		 */
		typedef tr1::unordered_map<Map::Cell *, OL::iterator, Map::Cell::Hash> OH;
		OH _open_hash;

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
		 * @var double**** interpolation lookup table for quickly aquiring cell costs and optimal XY given a cell and two consecutive neighbors.
		 */
		double _I[70][70][69][3];

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
		 * @param int Nc number of distinct traversal costs (including the infinite cost of traversing an obstacle cell)
		 * @return void
		 */
		void _construct_cellcosts(int Nc);

		/**
		 * Generates interpolation lookup table for quickly aquiring cell costs.
		 *
		 * @param int Nc number of distinct traversal costs (including the infinite cost of traversing an obstacle cell)
		 * @param int Mc maximum traversal cost of any traversable (i.e. non-obstacle) cell
		 * @return void
		 */
		void _construct_interpolation_table(int Nc, int Mc);

		/**
		 * Computes shortest interpolated path.
		 *
		 * @return bool successful
		 */
		bool _compute_shortest_path();

		/**
		 * Calculates the interpolated cost of 's' given any two consecutive neighbors 'sa' and 'sb'.
		 *
		 * @param Map::Cell* cell s
		 * @param Map::Cell* cell sa
		 * @param Map::Cell* cell sb
		 * @return pair<double, pair<pair<double, double>, pair<double, double>>> cost of s and point1 x,y and point2 x,y
		 */
		pair<double, pair<pair<double, double>, pair<double, double>>> _compute_cost(Map::Cell *s, Map::Cell *sa, Map::Cell *sb);

		/**
		 * Extracts interpolated path
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
		 * @param pair<double,double> key vakue for the cell
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
		 * @param bool check if cell is already in _path_set
		 * @return <Map::Cell*,double> successor
		 */
		pair<Map::Cell *, double> _min_interpol_succ(Map::Cell *u, bool check_path = false);

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