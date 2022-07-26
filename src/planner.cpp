#include "planner.h"

/**
 * @var static const double max steps before assuming no solution possible
 */
const double Planner::MAX_STEPS = 1000000;

/**
 * Constructor.
 *
 * @param Map* map
 * @param Map::Cell* start cell
 * @param Map::Cell* goal cell
 * @param int Nc
 */
Planner::Planner(Map *map, Map::Cell *start, Map::Cell *goal, int Nc)
{
	// Clear lists
	_interpol_path.clear();
	_path_set.clear();
	_open_list.clear();
	_open_hash.clear();
	_path.clear();

	_map = map;
	_start = start;
	_goal = goal;
	_Nc = Nc;
	_Mc = Nc - 1;

	_construct_interpolation_table(_Nc, _Mc);

	_rhs(_goal, 0.0);

	_list_insert(_goal, pair<double, double>(_h(_start, _goal), 0));
}

/**
 * Deconstructor.
 */
Planner::~Planner()
{
}

/**
 * Returns the generated path.
 *
 * @return vector<pair<double, double>>
 */
vector<pair<double, double>> Planner::path()
{
	return _interpol_path;
}

/**
 * Gets/Sets a new goal.
 *
 * @param Map::Cell* [optional] goal
 * @return Map::Cell* new goal
 */
Map::Cell *Planner::goal(Map::Cell *u)
{
	if (u == nullptr)
		return _goal;

	// Hack implementation
	_goal = u;

	return _goal;
}

/**
 * Replans the path.
 *
 * @return bool solution found
 */
bool Planner::replan()
{
	_path.clear();
	_path_set.clear();
	_interpol_path.clear();

	bool result = _compute_shortest_path();

	// Couldn't find a solution
	if (!result)
	{
		return false;
	}

	result = _extract_path();

	return result;
}

/**
 * Gets/Sets start.
 *
 * @param Map::Cell* [optional] new start
 * @return Map::Cell* start
 */
Map::Cell *Planner::start(Map::Cell *u)
{
	if (u == nullptr)
		return _start;

	_start = u;

	return _start;
}

/**
 * Update map.
 *
 * @param Map::Cell* cell to update
 * @param int new cost of the cell
 * @return void
 */
void Planner::update_cell_cost(Map::Cell *u, int cost)
{
	int c_old = u->cost;
	u->cost = cost;
	_cell(u);

	double rhs_min;

	Map::Cell *u_new;
	Map::Cell **cnrs = u->cnrs();
	pair<Map::Cell *, double> argmin_min;

	if (cost > c_old)
	{
		for (unsigned int i = 0; i < Map::Cell::NUM_CNRS; i++)
		{
			if (cnrs[i] != nullptr && cnrs[i]->bptr() != nullptr && cnrs[i]->ccknbr(cnrs[i]->bptr()) != nullptr)
			{
				if (u->is_corner(cnrs[i]->bptr()) || u->is_corner(cnrs[i]->ccknbr(cnrs[i]->bptr())))
				{
					if (!Math::equals(_rhs(cnrs[i]), _compute_cost(cnrs[i], cnrs[i]->bptr(), cnrs[i]->ccknbr(cnrs[i]->bptr())).first))
					{
						if (Math::less(_g(cnrs[i]), _rhs(cnrs[i])) || _open_hash.find(cnrs[i]) == _open_hash.end())
						{
							_rhs(cnrs[i], Math::INF);
							_update_state(cnrs[i]);
						}
						else
						{
							argmin_min = _min_interpol_succ(cnrs[i]);
							_rhs(cnrs[i], argmin_min.second);
							cnrs[i]->bptr(argmin_min.first);
							_update_state(cnrs[i]);
						}
					}
				}
			}
		}
	}
	else
	{
		rhs_min = Math::INF;
		for (unsigned int i = 0; i < Map::Cell::NUM_CNRS; i++)
		{
			if (cnrs[i] != nullptr)
			{
				if (_cell_hash.find(cnrs[i]) == _cell_hash.end())
				{
					_cell(cnrs[i]);
				}
				else if (Math::less(_rhs(cnrs[i]), rhs_min))
				{
					rhs_min = _rhs(cnrs[i]);
					u_new = cnrs[i];
				}
			}
		}
		if (!Math::equals(rhs_min, Math::INF))
		{
			_update_state(u_new);
		}
	}
}

/**
 * Generates a cell.
 *
 * @param Map::Cell*
 * @return void
 */
void Planner::_cell(Map::Cell *u)
{
	if (_cell_hash.find(u) != _cell_hash.end())
		return;

	double h = Math::INF;
	_cell_hash[u] = pair<double, double>(h, h);
}

/**
 * Initializes cell cost table, which indices representing original cell costs which map to non-lineraly space cell costs
 *
 * @param int Nc number of distinct traversal costs (including the infinite cost of traversing an obstacle cell)
 * @return void
 */
void Planner::_construct_cellcosts(int Nc)
{
	_cellcosts.resize(Nc);

	for (unsigned int i = 0; i < _cellcosts.size() - 1; i++)
	{
		_cellcosts[i] = i + 1;
		// _cellcosts[i] = 255.0 * pow(Math::EUL, ((0.33333 * i) / 11.8125));
	}
	_cellcosts[_cellcosts.size() - 1] = Map::Cell::COST_UNWALKABLE;
}

/**
 * Generates interpolation lookup table for quickly aquiring cell costs.
 *
 * @param int Nc number of distinct traversal costs (including the infinite cost of traversing an obstacle cell)
 * @param int Mc maximum traversal cost of any traversable (i.e. non-obstacle) cell
 * @return void
 */
void Planner::_construct_interpolation_table(int Nc, int Mc)
{

	_construct_cellcosts(Nc);

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

/**
 * Computes shortest interpolated path.
 *
 * @return bool successful
 */
bool Planner::_compute_shortest_path()
{
	if (_open_list.empty())
	{
		printf("ERROR: OPEN LIST IS EMPTY\n");
		return false;
	}

	KeyCompare key_compare;

	int attempts = 0;

	Map::Cell *u;
	Map::Cell **nbrs;
	double rhs_old, cost_interpol;
	pair<Map::Cell *, double> argmin_min;

	while ((!_open_list.empty() && key_compare(_open_list.begin()->first, _k(_start))) || !Math::equals(_rhs(_start), _g(_start)))
	{
		// Reached max steps, quit
		if (++attempts > Planner::MAX_STEPS)
		{
			printf("ERROR: REACHED MAXIMUM PLANNER ATTEMPTS\n");
			return false;
		}

		u = _open_list.begin()->second;
		if (Math::greater(_g(u), _rhs(u)) || Math::equals(_g(u), _rhs(u)))
		{
			_g(u, _rhs(u));
			_update_state(u);
			nbrs = u->nbrs();
			for (unsigned int i = 0; i < Map::Cell::NUM_NBRS; i++)
			{
				if (nbrs[i] != nullptr)
				{
					if (_cell_hash.find(nbrs[i]) == _cell_hash.end())
					{
						_cell(nbrs[i]);
					}
					rhs_old = _rhs(nbrs[i]);
					if (nbrs[i]->ccknbr(u) != nullptr)
					{
						cost_interpol = _compute_cost(nbrs[i], u, nbrs[i]->ccknbr(u)).first;
						if (Math::greater(_rhs(nbrs[i]), cost_interpol))
						{
							_rhs(nbrs[i], cost_interpol);
							nbrs[i]->bptr(u);
						}
					}
					if (nbrs[i]->cknbr(u) != nullptr)
					{
						cost_interpol = _compute_cost(nbrs[i], u, nbrs[i]->cknbr(u)).first;
						if (Math::greater(_rhs(nbrs[i]), cost_interpol))
						{
							_rhs(nbrs[i], _compute_cost(nbrs[i], nbrs[i]->cknbr(u), u).first);
							nbrs[i]->bptr(nbrs[i]->cknbr(u));
						}
					}
					if (!Math::equals(_rhs(u), rhs_old))
					{
						_update_state(nbrs[i]);
					}
				}
			}
		}
		else
		{
			argmin_min = _min_interpol_succ(u);
			_rhs(u, argmin_min.second);
			u->bptr(argmin_min.first);
			if (Math::less(_g(u), _rhs(u)))
			{
				_g(u, Math::INF);
				nbrs = u->nbrs();
				for (unsigned int i = 0; i < Map::Cell::NUM_NBRS; i++)
				{
					if (nbrs[i] != nullptr && nbrs[i]->bptr() != nullptr && nbrs[i]->ccknbr(nbrs[i]->bptr()) != nullptr)
					{
						if (nbrs[i]->bptr() == u || nbrs[i]->bptr() == nbrs[i]->cknbr(u))
						{
							if (!Math::equals(_rhs(nbrs[i]), _compute_cost(nbrs[i], nbrs[i]->bptr(), nbrs[i]->ccknbr(nbrs[i]->bptr())).first))
							{
								if (Math::less(_g(nbrs[i]), _rhs(nbrs[i])) || _open_hash.find(nbrs[i]) == _open_hash.end())
								{
									_rhs(nbrs[i], Math::INF);
									_update_state(nbrs[i]);
								}
								else
								{
									argmin_min = _min_interpol_succ(nbrs[i]);
									_rhs(nbrs[i], argmin_min.second);
									nbrs[i]->bptr(argmin_min.first);
									_update_state(nbrs[i]);
								}
							}
						}
					}
				}
			}
			_update_state(u);
		}
	}
	printf("Planner Attempts: %d\n", attempts);
	return true;
}

/**
 * Calculates the interpolated cost of 's' given any two consecutive neighbors 'sa' and 'sb'.
 *
 * @param Map::Cell* cell s
 * @param Map::Cell* cell sa
 * @param Map::Cell* cell sb
 * @return pair<double, pair<pair<double, double>, pair<double, double>>> cost of s and point1 x,y and point2 x,y
 */
pair<double, pair<pair<double, double>, pair<double, double>>> Planner::_compute_cost(Map::Cell *s, Map::Cell *sa, Map::Cell *sb)
{
	if (s == nullptr || sa == nullptr || sb == nullptr)
	{
		printf("ERROR: NULL CELL FOUND WHEN COMPUTING COST\n");
		return make_pair(Math::INF, make_pair(make_pair(Math::INF, Math::INF), make_pair(Math::INF, Math::INF)));
	}
	Map::Cell **nbrs = s->nbrs();
	Map::Cell *s1;
	Map::Cell *s2;
	bool is_diag = (abs(static_cast<int>(s->x()) - static_cast<int>(sa->x())) + abs(static_cast<int>(s->y()) - static_cast<int>(sa->y()))) > 1;
	if (is_diag)
	{
		s1 = sb;
		s2 = sa;
	}
	else
	{
		s1 = sa;
		s2 = sb;
	}
	int bi, ci, edge;
	double b, c;
	if ((s1->x() == nbrs[0]->x() && s1->y() == nbrs[0]->y()) && (s2->x() == nbrs[1]->x() && s2->y() == nbrs[1]->y()))
	{
		edge = 1;
		ci = s->cost;
		bi = nbrs[6]->cost;
	}
	else if ((s1->x() == nbrs[2]->x() && s1->y() == nbrs[2]->y()) && (s2->x() == nbrs[1]->x() && s2->y() == nbrs[1]->y()))
	{
		edge = 2;
		ci = s->cost;
		bi = nbrs[4]->cost;
	}
	else if ((s1->x() == nbrs[2]->x() && s1->y() == nbrs[2]->y()) && (s2->x() == nbrs[3]->x() && s2->y() == nbrs[3]->y()))
	{
		edge = 3;
		ci = nbrs[4]->cost;
		bi = s->cost;
	}
	else if ((s1->x() == nbrs[4]->x() && s1->y() == nbrs[4]->y()) && (s2->x() == nbrs[3]->x() && s2->y() == nbrs[3]->y()))
	{
		edge = 4;
		ci = nbrs[4]->cost;
		bi = nbrs[5]->cost;
	}
	else if ((s1->x() == nbrs[4]->x() && s1->y() == nbrs[4]->y()) && (s2->x() == nbrs[5]->x() && s2->y() == nbrs[5]->y()))
	{
		edge = 5;
		ci = nbrs[5]->cost;
		bi = nbrs[4]->cost;
	}
	else if ((s1->x() == nbrs[6]->x() && s1->y() == nbrs[6]->y()) && (s2->x() == nbrs[5]->x() && s2->y() == nbrs[5]->y()))
	{
		edge = 6;
		ci = nbrs[5]->cost;
		bi = nbrs[6]->cost;
	}
	else if ((s1->x() == nbrs[6]->x() && s1->y() == nbrs[6]->y()) && (s2->x() == nbrs[7]->x() && s2->y() == nbrs[7]->y()))
	{
		edge = 7;
		ci = nbrs[6]->cost;
		bi = nbrs[5]->cost;
	}
	else if ((s1->x() == nbrs[0]->x() && s1->y() == nbrs[0]->y()) && (s2->x() == nbrs[7]->x() && s2->y() == nbrs[7]->y()))
	{
		edge = 8;
		ci = nbrs[6]->cost;
		bi = s->cost;
	}
	else
	{
		printf("ERROR: INVALID CELL COORDINATES FOUND WHEN COMPUTING COST\n");
		return make_pair(Math::INF, make_pair(make_pair(Math::INF, Math::INF), make_pair(Math::INF, Math::INF)));
	}
	c = _cellcosts[ci];
	b = _cellcosts[bi];
	double g1 = _g(s1);
	double g2 = _g(s2);
	double vs, x1, y1, x2, y2, tx, ty;
	if (Math::equals(min(c, b), Map::Cell::COST_UNWALKABLE))
	{
		// no valid path
		return make_pair(Math::INF, make_pair(make_pair(Math::INF, Math::INF), make_pair(Math::INF, Math::INF)));
	}
	else if (Math::less(g1, g2) || Math::equals(g1, g2))
	{
		// goes straight to s1
		vs = min(c, b) + g1;
		tx = 1.0;
		ty = 0.0;
		x2 = 0.0;
		y2 = 0.0;
	}
	else
	{
		// f is the cost of traversing the right edge
		double f = g1 - g2;
		if (Math::greater(f, min(c, b)))
		{
			// travel a distance x along the bottom edge then take a straight-line path to s2
			vs = _I[ci][bi][_Mc][0] + g2;
			tx = _I[ci][bi][_Mc][1];
			ty = 0.0;
			x2 = 1.0;
			y2 = 1.0;
		}
		else
		{
			// take a straight-line path from s to some point sy on the right edge
			vs = _I[ci][bi][static_cast<int>(f)][0] + g2;
			tx = 1.0;
			ty = _I[ci][bi][static_cast<int>(f)][2];
			x2 = 0.0;
			y2 = 0.0;
		}
	}

	switch (edge)
	{
	case 1:
		x1 = tx;
		y1 = ty;
		break;
	case 2:
		x1 = ty;
		y1 = tx;
		break;
	case 3:
		x1 = -ty;
		y1 = tx;
		x2 = -y2;
		break;
	case 4:
		x1 = -tx;
		y1 = ty;
		x2 = -x2;
		break;
	case 5:
		x1 = -tx;
		y1 = -ty;
		x2 = -x2;
		y2 = -y2;
		break;
	case 6:
		x1 = -ty;
		y1 = -tx;
		x2 = -y2;
		y2 = -x2;
		break;
	case 7:
		x1 = ty;
		y1 = -tx;
		y2 = -x2;
		break;
	case 8:
		x1 = tx;
		y1 = -ty;
		y2 = -y2;
		break;
	default:
		printf("ERROR: INVALID EDGE\n"); // should never reach here
		return make_pair(Math::INF, make_pair(make_pair(Math::INF, Math::INF), make_pair(Math::INF, Math::INF)));
		break;
	}
	return make_pair(vs, make_pair(make_pair(x1, y1), make_pair(x2, y2)));
}

/**
 * Extracts interpolated path
 *
 * @return bool successful
 */
bool Planner::_extract_path()
{
	pair<pair<double, double>, pair<double, double>> points;
	pair<double, double> point, tpoint;
	double x, y;
	Map::Cell *prev;
	Map::Cell *current;

	// Follow the path with the least cost until goal is reached
	current = _start;
	printf("Path start:\n{");
	while (current != _goal)
	{
		if (current == nullptr)
		{
			printf("\nERROR: INVALID CELL\n");
			return false;
		}
		printf("[%u, %u], ", current->x(), current->y());
		_path.push_back(current);
		_path_set.insert(current);
		prev = current;
		current = prev->bptr();
		if (_path_set.find(prev->bptr()) != _path_set.end())
		{
			current = _min_interpol_succ(prev, true).first;
			printf("\nFAKE BPTR!!!!!!!!!!!!!!!!!!!!!!!!\n");
			// return false;
		}
	}
	_path.push_back(current);
	printf("[%u, %u]}\n", current->x(), current->y());

	point = make_pair(_path.front()->x(), _path.front()->y());
	_interpol_path.push_back(point);
	for (unsigned int i = 0; i < _path.size() - 1; i++)
	{
		if (_path[i] == nullptr || _path[i + 1] == nullptr || _path[i]->ccknbr(_path[i + 1]) == nullptr)
		{
			printf("ERROR: A CELL NEEDED TO COMPUTE THE INTERPOLATED PATH WAS NULL\n"); // this should not be possible
			return false;
		}
		points = _compute_cost(_path[i], _path[i + 1], _path[i]->ccknbr(_path[i + 1])).second;
		if (Math::equals(points.first.first, Math::INF) || Math::equals(points.first.second, Math::INF) || Math::equals(points.second.first, Math::INF) || Math::equals(points.second.second, Math::INF))
		{
			printf("ERROR: PATH POINT WAS INFINITE\n"); // this should not be possible
			return false;
		}
		tpoint = make_pair(_path[i]->x(), _path[i]->y());
		point = make_pair(tpoint.first + points.first.first, tpoint.second + points.first.second);
		_interpol_path.push_back(point);
		if (!Math::equals(points.second.first, 0.0) || !Math::equals(points.second.second, 0.0))
		{
			point = make_pair(tpoint.first + points.second.first, tpoint.second + points.second.second);
			_interpol_path.push_back(point);
		}
	}
	point = make_pair(_path.back()->x(), _path.back()->y());
	_interpol_path.push_back(point);
	return true;
}

/**
 * Gets/Sets g value for a cell.
 *
 * @param Map::Cell* cell to retrieve/update
 * @param double [optional] new g value
 * @return double g value
 */
double Planner::_g(Map::Cell *u, double value)
{
	_cell(u);
	pair<double, double> *g_rhs = &_cell_hash[u];

	if (value != DBL_MIN)
	{
		g_rhs->first = value;
	}

	return g_rhs->first;
}

/**
 * Calculates heuristic between two cells (euclidean distance).
 *
 * @param Map::Cell* cell a
 * @param Map::Cell* cell b
 * @return double heuristic value
 */
double Planner::_h(Map::Cell *a, Map::Cell *b)
{
	return (0.5 * sqrt(pow(static_cast<int>(b->x()) - static_cast<int>(a->x()), 2) + pow(static_cast<int>(b->y()) - static_cast<int>(a->y()), 2)));
}

/**
 * Calculates key value for cell.
 *
 * @param Map::Cell* cell to calculate for
 * @return pair<double,double> key value
 */
pair<double, double> Planner::_k(Map::Cell *u)
{
	double g = _g(u);
	double rhs = _rhs(u);
	double min = (Math::less(g, rhs)) ? g : rhs;
	return pair<double, double>((min + _h(_start, u)), min);
}

/**
 * Inserts cell into open list.
 *
 * @param Map::Cell* cell to insert
 * @param pair<double,double> key vakue for the cell
 * @return void
 */
void Planner::_list_insert(Map::Cell *u, pair<double, double> k)
{
	OL::iterator pos = _open_list.insert(OL_PAIR(k, u));
	_open_hash[u] = pos;
}

/**
 * Removes cell from the open list.
 *
 * @param Map::Cell* cell to remove
 * @return void
 */
void Planner::_list_remove(Map::Cell *u)
{
	_open_list.erase(_open_hash[u]);
	_open_hash.erase(_open_hash.find(u));
}

/**
 * Updates cell in the open list.
 *
 * @param Map::Cell*
 * @param pair<double,double>
 * @return void
 */
void Planner::_list_update(Map::Cell *u, pair<double, double> k)
{
	OL::iterator pos1 = _open_hash[u];
	OL::iterator pos2 = pos1;

	if (pos1 == _open_list.end())
	{
		pos2 = _open_list.end();
	}
	else
	{
		pos2++;
	}

	_open_list.erase(pos1);
	_open_hash[u] = _open_list.insert(pos2, OL_PAIR(k, u));
}

/**
 * Finds the minimum successor cell using interpolated costs.
 *
 * @param Map::Cell* root
 * @param bool check if cell is already in _path_set
 * @return <Map::Cell*,double> successor
 */
pair<Map::Cell *, double> Planner::_min_interpol_succ(Map::Cell *u, bool check_path)
{
	Map::Cell **nbrs = u->nbrs();

	double tmp_cost, tmp_g;

	Map::Cell *min_cell = nullptr;
	double min_cost = Math::INF;

	for (unsigned int i = 0; i < Map::Cell::NUM_NBRS; i++)
	{
		if (nbrs[i] != nullptr && u->ccknbr(nbrs[i]) != nullptr)
		{
			if (!check_path || _path_set.find(nbrs[i]) == _path_set.end())
			{
				tmp_cost = _compute_cost(u, nbrs[i], u->ccknbr(nbrs[i])).first;

				if (Math::less(tmp_cost, min_cost))
				{
					min_cell = nbrs[i];
					min_cost = tmp_cost;
				}
			}
		}
	}
	if (min_cell == nullptr || min_cost == Math::INF)
	{
		// printf("Warning: Minimum interpolated successor was NULL or cost was infinite\n");
	}
	return pair<Map::Cell *, double>(min_cell, min_cost);
}

/**
 * Gets/Sets rhs value for a cell.
 *
 * @param Map::Cell* cell to retrieve/update
 * @param double [optional] new rhs value
 * @return double rhs value
 */
double Planner::_rhs(Map::Cell *u, double value)
{
	if (u == _goal)
		return 0;

	_cell(u);
	pair<double, double> *g_rhs = &_cell_hash[u];

	if (value != DBL_MIN)
	{
		g_rhs->second = value;
	}

	return g_rhs->second;
}

/**
 * Updates cell.
 *
 * @param Map::Cell* cell to update
 * @return void
 */
void Planner::_update_state(Map::Cell *u)
{

	bool diff = !Math::equals(_g(u), _rhs(u));
	bool exists = (_open_hash.find(u) != _open_hash.end());

	if (diff && exists)
	{
		_list_update(u, _k(u));
	}
	else if (diff && !exists)
	{
		_list_insert(u, _k(u));
	}
	else if (!diff && exists)
	{
		_list_remove(u);
	}
}

/**
 * Key compare function.
 */
bool Planner::KeyCompare::operator()(const pair<double, double> &p1, const pair<double, double> &p2) const
{
	if (Math::less(p1.first, p2.first))
		return true;
	else if (Math::greater(p1.first, p2.first))
		return false;
	else if (Math::less(p1.second, p2.second))
		return true;
	else if (Math::greater(p1.second, p2.second))
		return false;
	return false;
}