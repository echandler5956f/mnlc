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
	printf("Initializing Field D* Planner\n");

	// Clear lists
	_interpol_path.clear();
	_open_list.clear();
	_open_hash.clear();

	_map = map;
	_start = start;
	_goal = goal;
	_Nc = Nc;

	printf("Constructing interpolation table\n");

	_Mc = _construct_cellcosts();
	printf("_Nc: %d, _Mc: %d\n", _Nc, _Mc);
	_construct_interpolation_table();
	_rhs(_goal, 0.0);
	_list_insert(_goal, pair<double, double>(_h(_start, _goal), 0));

	printf("Field D* Planner is ready\n");
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
 * Returns a path-to-goal map
 *
 * @return vector<double> map
 */
vector<double> Planner::g_map()
{
	unsigned int cols = _map->cols();
	unsigned int rows = _map->rows();
	vector<double> gmap;
	for (unsigned int i = 0; i < rows; i++)
	{
		for (unsigned int j = 0; j < cols; j++)
		{
			gmap.push_back(_g((*_map)(i, j)));
		}
	}
	return gmap;
}

/**
 * Replans the path.
 *
 * @return bool solution found
 */
bool Planner::replan()
{
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
 * Returns a lookahead of the path-to-goal map
 *
 * @return vector<double> map
 */
vector<double> Planner::rhs_map()
{
	unsigned int cols = _map->cols();
	unsigned int rows = _map->rows();
	vector<double> rhsmap;
	for (unsigned int i = 0; i < rows; i++)
	{
		for (unsigned int j = 0; j < cols; j++)
		{
			rhsmap.push_back(_rhs((*_map)(i, j)));
		}
	}
	return rhsmap;
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
 * @return int Mc maximum traversable (non-obstacle) cost
 */
int Planner::_construct_cellcosts()
{
	_cellcosts.resize(_Nc + 1);

	for (unsigned int i = 0; i < _Nc; i++)
	{
		_cellcosts[i] = round(255.0 * pow(Math::EUL, ((0.33333 * static_cast<double>(i)) / 11.8125)) - 254.0);
	}
	_cellcosts[_Nc] = Map::Cell::COST_UNWALKABLE;
	return static_cast<int>(_cellcosts[_Nc - 1]);
}

/**
 * Generates interpolation lookup table for quickly aquiring cell costs.
 *
 * @return void
 */
void Planner::_construct_interpolation_table()
{
	_I.resize(_Nc + 1);
	for (int i = 0; i < _Nc + 1; i++)
	{
		_I[i].resize(_Nc + 1);
		for (int j = 0; j < _Nc + 1; j++)
		{
			_I[i][j].resize(_Mc + 1);
			for (int k = 0; k < _Mc + 1; k++)
			{
				_I[i][j][k].resize(3);
			}
		}
	}

	int ci, bi, f;
	double c, b, x, y;

	ci = 0;

	while (ci < _Nc)
	{
		c = _cellcosts[ci];
		bi = 0;

		while (bi < _Nc)
		{
			b = _cellcosts[bi];
			f = 1;
			while (f <= _Mc)
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
			_list_remove(u);
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
						if (Math::greater(_rhs(nbrs[i]), _compute_cost(nbrs[i], u, nbrs[i]->cknbr(u)).first))
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
	pair<double, pair<pair<double, double>, pair<double, double>>> vsp1p2;
	if (s == nullptr || sa == nullptr || sb == nullptr)
	{
		printf("ERROR: NULL CELL FOUND WHEN COMPUTING COST\n");
		vsp1p2 = make_pair(Math::INF, make_pair(make_pair(Math::INF, Math::INF), make_pair(Math::INF, Math::INF)));
	}
	else
	{
		double vs, b, c, g1, g2, f, x1, y1, x2, y2, tx, ty;
		pair<double, double> p1, p2;
		bool is_diag, two_points;
		int bi, ci, edge;
		Map::Cell **nbrs;
		Map::Cell *s1;
		Map::Cell *s2;
		nbrs = s->nbrs();
		is_diag = (abs(static_cast<int>(sa->x()) - static_cast<int>(s->x())) + abs(static_cast<int>(sa->y()) - static_cast<int>(s->y()))) > 1;
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
		edge = 0;
		if (nbrs[6] != nullptr && (nbrs[0] != nullptr && s1->x() == nbrs[0]->x() && s1->y() == nbrs[0]->y()) && (nbrs[1] != nullptr && s2->x() == nbrs[1]->x() && s2->y() == nbrs[1]->y()))
		{
			edge = 1;
			ci = s->cost;
			bi = nbrs[6]->cost;
		}
		else if (nbrs[4] != nullptr && (nbrs[2] != nullptr && s1->x() == nbrs[2]->x() && s1->y() == nbrs[2]->y()) && (nbrs[1] != nullptr && s2->x() == nbrs[1]->x() && s2->y() == nbrs[1]->y()))
		{
			edge = 2;
			ci = s->cost;
			bi = nbrs[4]->cost;
		}
		else if (nbrs[4] != nullptr && (nbrs[2] != nullptr && s1->x() == nbrs[2]->x() && s1->y() == nbrs[2]->y()) && (nbrs[3] != nullptr && s2->x() == nbrs[3]->x() && s2->y() == nbrs[3]->y()))
		{
			edge = 3;
			ci = nbrs[4]->cost;
			bi = s->cost;
		}
		else if (nbrs[5] != nullptr && (nbrs[4] != nullptr && s1->x() == nbrs[4]->x() && s1->y() == nbrs[4]->y()) && (nbrs[3] != nullptr && s2->x() == nbrs[3]->x() && s2->y() == nbrs[3]->y()))
		{
			edge = 4;
			ci = nbrs[4]->cost;
			bi = nbrs[5]->cost;
		}
		else if ((nbrs[4] != nullptr && s1->x() == nbrs[4]->x() && s1->y() == nbrs[4]->y()) && (nbrs[5] != nullptr && s2->x() == nbrs[5]->x() && s2->y() == nbrs[5]->y()))
		{
			edge = 5;
			ci = nbrs[5]->cost;
			bi = nbrs[4]->cost;
		}
		else if ((nbrs[6] != nullptr && s1->x() == nbrs[6]->x() && s1->y() == nbrs[6]->y()) && (nbrs[5] != nullptr && s2->x() == nbrs[5]->x() && s2->y() == nbrs[5]->y()))
		{
			edge = 6;
			ci = nbrs[5]->cost;
			bi = nbrs[6]->cost;
		}
		else if (nbrs[5] != nullptr && (nbrs[6] != nullptr && s1->x() == nbrs[6]->x() && s1->y() == nbrs[6]->y()) && (nbrs[7] != nullptr && s2->x() == nbrs[7]->x() && s2->y() == nbrs[7]->y()))
		{
			edge = 7;
			ci = nbrs[6]->cost;
			bi = nbrs[5]->cost;
		}
		else if (nbrs[6] != nullptr && (nbrs[0] != nullptr && s1->x() == nbrs[0]->x() && s1->y() == nbrs[0]->y()) && (nbrs[7] != nullptr && s2->x() == nbrs[7]->x() && s2->y() == nbrs[7]->y()))
		{
			edge = 8;
			ci = nbrs[6]->cost;
			bi = s->cost;
		}
		if (edge == 0)
		{
			printf("ERROR: INVALID CELL COORDINATES FOUND WHEN COMPUTING COST\n");
			vsp1p2 = make_pair(Math::INF, make_pair(make_pair(Math::INF, Math::INF), make_pair(Math::INF, Math::INF)));
		}
		else
		{
			if (ci < 0 || bi < 0)
			{
				printf("ERROR: INVALID COST INDICES\n");
				vsp1p2 = make_pair(Math::INF, make_pair(make_pair(Math::INF, Math::INF), make_pair(Math::INF, Math::INF)));
			}
			else
			{
				c = _cellcosts[ci];
				b = _cellcosts[bi];

				if (Math::equals(min(c, b), Map::Cell::COST_UNWALKABLE))
				{
					// no valid path
					vsp1p2 = make_pair(Math::INF, make_pair(make_pair(Math::INF, Math::INF), make_pair(Math::INF, Math::INF)));
				}
				else
				{
					c = round(c);
					b = round(b);
					g1 = round(_g(s1));
					g2 = round(_g(s2));
					two_points = false;

					if (Math::less(g1, g2) || Math::equals(g1, g2))
					{
						// goes straight to s1
						vs = round(min(c, b) + g1);
						tx = 1.0;
						ty = 0.0;
					}
					else
					{
						// f is the cost of traversing the right edge
						f = round(g1 - g2);
						if (Math::greater(f, min(c, b)))
						{
							// travel a distance x along the bottom edge then take a straight-line path to s2
							vs = round(_I[ci][bi][_Mc][0] + g2);
							tx = _I[ci][bi][_Mc][1];
							ty = 0.0;
							x2 = 1.0;
							y2 = 1.0;
							two_points = true;
						}
						else
						{
							// take a straight-line path from s to some point sy on the right edge
							vs = round(_I[ci][bi][static_cast<int>(f)][0] + g2);
							tx = 1.0;
							ty = _I[ci][bi][static_cast<int>(f)][2];
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
						edge = 0;
						printf("ERROR: INVALID EDGE\n"); // should never reach here
						vsp1p2 = make_pair(Math::INF, make_pair(make_pair(Math::INF, Math::INF), make_pair(Math::INF, Math::INF)));
						break;
					}
					if (!two_points)
					{
						x2 = DBL_MIN;
						y2 = DBL_MIN;
					}
					if (edge != 0)
					{
						p1 = make_pair(x1, y1);
						p2 = make_pair(x2, y2);
						vsp1p2 = make_pair(vs, make_pair(p1, p2));
					}
				}
			}
		}
	}
	return vsp1p2;
}

/**
 * Extracts interpolated path
 *
 * @return bool successful
 */
bool Planner::_extract_path()
{
	pair<double, pair<pair<double, double>, pair<double, double>>> vsp1p2, temp_vsp1p2;
	pair<pair<double, double>, pair<double, double>> points;
	double x, y, temp_temp_cost, min_cost, min_min_cost;
	unordered_set<Map::Cell *> _cnr_of_set;
	pair<double, double> point, tpoint;
	Map::Cell **nbrsnbrs;
	Map::Cell *current;
	Map::Cell **cnr_of;
	Map::Cell **cnrs;
	Map::Cell **nbrs;
	Map::Cell *prev;

	// Follow the path with the least cost until goal is reached
	current = _start;
	point = make_pair(current->x(), current->y());
	_interpol_path.push_back(point);
	while (current != _goal)
	{
		if (current == nullptr)
		{
			printf("ERROR: INVALID CELL_1\n");
			return false;
		}
		// printf("Current x: %u, current y: %u\n", current->x(), current->y());
		// if one point in the path resides within some grid cell g, then the next point in the path can only be a corner state of
		// g if the backpointer of this corner state points to at least one state not in g
		cnr_of = current->cnr_of();
		for (unsigned int i = 0; i < Map::Cell::NUM_CNRS; i++)
		{
			if (cnr_of[i] != nullptr)
			{
				cnrs = cnr_of[i]->cnrs();
				for (unsigned int j = 0; j < Map::Cell::NUM_CNRS; j++)
				{
					if (cnrs[j] != nullptr)
					{
						_cnr_of_set.insert(cnrs[j]);
					}
				}
			}
		}
		prev = current;
		current = nullptr;
		vsp1p2 = make_pair(Math::INF, make_pair(make_pair(Math::INF, Math::INF), make_pair(Math::INF, Math::INF)));
		min_cost = Math::INF;
		nbrs = prev->nbrs();
		// get direct (simple) approximation of path cost
		for (unsigned int i = 0; i < Map::Cell::NUM_NBRS; i++)
		{
			if (nbrs[i] != nullptr)
			{
				if (prev->ccknbr(nbrs[i]) != nullptr)
				{
					temp_vsp1p2 = _compute_cost(prev, nbrs[i], prev->ccknbr(nbrs[i]));
					if (!Math::equals(temp_vsp1p2.first, Math::INF))
					{
						// get indirect (precise) approximation of path cost by locally finding an optimal path to the interpolated edge point
						min_min_cost = Math::INF;
						temp_temp_cost = Math::INF;
						nbrsnbrs = nbrs[i]->nbrs();
						for (unsigned int j = 0; j < Map::Cell::NUM_NBRS; j++)
						{
							if (nbrsnbrs[j] != nullptr)
							{
								if (_cnr_of_set.find(nbrsnbrs[j]) == _cnr_of_set.end() && nbrs[i]->ccknbr(nbrsnbrs[j]) != nullptr)
								{
									temp_temp_cost = _compute_cost(nbrs[i], nbrsnbrs[j], nbrs[i]->ccknbr(nbrsnbrs[j])).first;
									if (Math::less(temp_temp_cost, min_min_cost))
									{
										min_min_cost = temp_temp_cost;
									}
								}
							}
						}
						if (!Math::equals(min_min_cost, Math::INF) && Math::less(temp_vsp1p2.first + min_min_cost, min_cost))
						{
							current = nbrs[i];
							vsp1p2 = temp_vsp1p2;
							min_cost = temp_vsp1p2.first + min_min_cost;
						}
					}
				}
			}
		}
		if (current == nullptr)
		{
			printf("ERROR: PATH EXTRACTION FAILED\n");
			return false;
		}
		// add the interpolated points to the list
		points = vsp1p2.second;
		if (Math::equals(points.first.first, Math::INF) || Math::equals(points.first.second, Math::INF) || Math::equals(points.second.first, Math::INF) || Math::equals(points.second.second, Math::INF))
		{
			printf("ERROR: PATH POINT WAS INFINITE\n");
			return false;
		}
		tpoint = make_pair(prev->x(), prev->y());
		point = make_pair(tpoint.first + points.first.first, tpoint.second + points.first.second);
		_interpol_path.push_back(point);
		if (points.second.first != DBL_MIN || points.second.second != DBL_MIN)
		{
			point = make_pair(tpoint.first + points.second.first, tpoint.second + points.second.second);
			_interpol_path.push_back(point);
		}
	}
	point = make_pair(current->x(), current->y());
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
	return (7.5 * sqrt(pow(static_cast<int>(b->x()) - static_cast<int>(a->x()), 2) + pow(static_cast<int>(b->y()) - static_cast<int>(a->y()), 2)));
	// return (82.0 * sqrt(pow(static_cast<int>(b->x()) - static_cast<int>(a->x()), 2) + pow(static_cast<int>(b->y()) - static_cast<int>(a->y()), 2)));
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
 * @return <Map::Cell*,double> successor
 */
pair<Map::Cell *, double> Planner::_min_interpol_succ(Map::Cell *u)
{
	Map::Cell **nbrs = u->nbrs();
	Map::Cell *min_cell = nullptr;
	double tmp_cost, min_cost;

	min_cost = Math::INF;

	for (unsigned int i = 0; i < Map::Cell::NUM_NBRS; i++)
	{
		if (nbrs[i] != nullptr && u->ccknbr(nbrs[i]) != nullptr)
		{
			tmp_cost = _compute_cost(u, nbrs[i], u->ccknbr(nbrs[i])).first;

			if (Math::less(tmp_cost, min_cost))
			{
				min_cell = nbrs[i];
				min_cost = tmp_cost;
			}
		}
	}
	if (min_cell == nullptr || Math::equals(min_cost, Math::INF))
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