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
 */
Planner::Planner(Map *map, Map::Cell *start, Map::Cell *goal)
{
	// Clear lists
	_open_list.clear();
	_open_hash.clear();
	_path.clear();

	_map = map;
	_start = start;
	_goal = goal;

	// _construct_interpolation_table(Map::Cell::DIST_TRAV_COSTS, Map::Cell::DIST_TRAV_COSTS - 1);
	// _g(_start, Math::INF);
	// _rhs(_start, Math::INF);
	// _g(_goal, Math::INF);
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
 * @return list<Map::Cell*>
 */
list<Map::Cell *> Planner::path()
{
	return _path;
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
	unordered_set<Map::Cell *> _path_lu;
	Map::Cell **nbrs;
	Map::Cell *current;
	Map::Cell *min_cell;
	double min_cost, tmp_cost;

	// _path.clear();

	// bool result = _compute_shortest_path();

	// // Couldn't find a solution
	// if (!result)
	// 	return false;

	// current = _start;

	// printf("\nPath start:\n{");
	// while (current != _goal)
	// {
	// 	if (current == nullptr)
	// 	{
	// 		printf("\nInvalid cell.\n");
	// 		return false;
	// 	}
	// 	printf("[%u, %u], ", current->x(), current->y());
	// 	_path_lu.insert(current);
	// 	_path.push_back(current);
	// 	nbrs = current->nbrs();
	// 	min_cost = Math::INF;
	// 	min_cell = nullptr;
	// 	for (unsigned int i = 0; i < Map::Cell::NUM_NBRS; i++)
	// 	{
	// 		if (nbrs[i] != nullptr)
	// 		{
	// 			if (_path_lu.find(nbrs[i]) == _path_lu.end())
	// 			{
	// 				tmp_cost = _compute_cost(current, nbrs[i], current->ccknbr(nbrs[i]));

	// 				if (Math::less(tmp_cost, min_cost))
	// 				{
	// 					min_cell = nbrs[i];
	// 					min_cost = tmp_cost;
	// 				}
	// 			}
	// 		}
	// 	}
	// 	if (min_cell == nullptr || Math::equals(min_cost, Math::INF))
	// 	{
	// 		printf("Minimum cell was not found or had infinite cost.\n");
	// 		return false;
	// 	}
	// 	else
	// 	{
	// 		current = min_cell;
	// 	}
	// }
	// printf("[%u, %u]", current->x(), current->y());
	// printf("}\n");

	// _path.push_back(current);

	// return true;

	_path.clear();

	bool result = _compute_shortest_path();

	// Couldn't find a solution
	if (!result)
	{
		printf("Max steps reached.\n");
		return false;
	}

	current = _start;
	printf("\nPath start:\n{");
	// Follow the path with the least cost until goal is reached
	while (current != _goal)
	{
		if (current == nullptr)
		{
			printf("\nInvalid cell.\n");
			return false;
		}
		printf("[%u, %u], ", current->x(), current->y());
		_path.push_back(current);
		current = current->bptr();
	}
	printf("[%u, %u]", current->x(), current->y());
	printf("}\n");

	_path.push_back(current);

	return true;
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
 * @param double new cost of the cell
 * @return void
 */
void Planner::update_cell_cost(Map::Cell *u, double cost)
{
	// printf("Updating cell cost.\n");
	_cell(u);
	u->cost = cost;

	Map::Cell **cnrs = u->cnrs();
	Map::Cell **nbrs = u->nbrs();
	double min_cost, tmp_cost;
	for (unsigned int i = 0; i < Map::Cell::NUM_CNRS; i++)
	{
		if (cnrs[i] != nullptr)
		{
			if (_cell_hash.find(cnrs[i]) == _cell_hash.end())
			{
				_cell(cnrs[i]);
			}
			if (cnrs[i] != _goal)
			{
				min_cost = Math::INF;
				for (unsigned int j = 0; j < Map::Cell::NUM_NBRS; j++)
				{
					if (nbrs[j] != nullptr)
					{
						tmp_cost = _compute_cost(cnrs[i], nbrs[j], cnrs[i]->ccknbr(nbrs[j]));
						if (Math::less(tmp_cost, min_cost))
						{
							min_cost = tmp_cost;
						}
					}
				}
				if (Math::equals(min_cost, Math::INF))
				{
					// printf("Update Cell Cost - Minimum cell was not found or had infinite cost.\n");
				}
				_rhs(cnrs[i], min_cost);
				_update_state(cnrs[i]);
			}
		}
	}

	// _cell(u);
	// u->cost = cost;

	// Map::Cell **cnrs = u->cnrs();
	// for (unsigned int i = 0; i < Map::Cell::NUM_CNRS; i++)
	// {
	// 	if (cnrs[i] != nullptr)
	// 	{
	// 		_update_state(cnrs[i]);
	// 	}
	// }

	// double c_old = u->cost;
	// u->cost = cost;
	// _cell(u);

	// double rhs_min;

	// Map::Cell *u_new;
	// Map::Cell **cnrs = u->cnrs();
	// // printf("(ux: %u, uy: %u), (cnrs0x: %u, cnrs0y: %u), (cnrs1x: %u, cnrs1y: %u), , (cnrs2x: %u, cnrs2y: %u), (cnrs3x: %u, cnrs3y: %u\n)", u->x(), u->y(), cnrs[0]->x(), cnrs[0]->y(), cnrs[1]->x(), cnrs[1]->y(), cnrs[2]->x(), cnrs[2]->y(), cnrs[3]->x(), cnrs[3]->y());
	// pair<Map::Cell *, double> argmin_min;

	// if (Math::greater(cost, c_old))
	// {
	// 	printf("Entering If statement of updating cell cost.\n");
	// 	for (unsigned int i = 0; i < Map::Cell::NUM_CNRS; i++)
	// 	{
	// 		printf("Entering For Loop of If statement of updating cell cost.\n");
	// 		if (cnrs[i] != nullptr)
	// 		// if (cnrs[i] != nullptr && cnrs[i]->bptr() != nullptr && cnrs[i]->ccknbr(cnrs[i]->bptr()) != nullptr)
	// 		{
	// 			if (u->is_corner(cnrs[i]->bptr()) || u->is_corner(cnrs[i]->ccknbr(cnrs[i]->bptr())))
	// 			{
	// 				printf("Entering If statement of For Loop of If statement of updating cell cost.\n");
	// 				// printf("_compute_cost(cnrs[i], cnrs[i]->bptr(), cnrs[i]->ccknbr(cnrs[i]->bptr()))\n");
	// 				if (!Math::equals(_rhs(cnrs[i]), _compute_cost(cnrs[i], cnrs[i]->bptr(), cnrs[i]->ccknbr(cnrs[i]->bptr()))))
	// 				{
	// 					if (Math::less(_g(cnrs[i]), _rhs(cnrs[i])) || _open_hash.find(cnrs[i]) == _open_hash.end())
	// 					{
	// 						printf("Entering If statement of If statement of For Loop of If statement of updating cell cost.\n");
	// 						_rhs(cnrs[i], Math::INF);
	// 						_update_state(cnrs[i]);
	// 					}
	// 					else
	// 					{
	// 						printf("Entering Else statement of If statement of For Loop of If statement of updating cell cost.\n");
	// 						argmin_min = _min_interpol_succ(cnrs[i]);
	// 						_rhs(cnrs[i], argmin_min.second);
	// 						cnrs[i]->bptr(argmin_min.first);
	// 						_update_state(cnrs[i]);
	// 					}
	// 				}
	// 			}
	// 		}
	// 	}
	// }
	// else
	// {
	// 	printf("Entering Else statement of updating cell cost.\n");
	// 	rhs_min = Math::INF;
	// 	for (unsigned int i = 0; i < Map::Cell::NUM_CNRS; i++)
	// 	{
	// 		printf("Entering For Loop of Else statement of updating cell cost.\n");
	// 		if (cnrs[i] != nullptr)
	// 		{
	// 			if (_cell_hash.find(cnrs[i]) == _cell_hash.end())
	// 			{
	// 				_cell(cnrs[i]);
	// 			}
	// 			else if (Math::less(_rhs(cnrs[i]), rhs_min))
	// 			{
	// 				printf("Entering Else If statement of For Loop of Else statement of updating cell cost.\n");
	// 				rhs_min = _rhs(cnrs[i]);
	// 				u_new = cnrs[i];
	// 			}
	// 		}
	// 	}
	// 	if (!Math::equals(rhs_min, Math::INF))
	// 	{
	// 		_list_insert(u_new, _k(u_new));
	// 	}
	// }
	// printf("Finished updating cell cost.\n");
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
 * @param unsigned int Nc number of distinct traversal costs (including the infinite cost of traversing an obstacle cell)
 * @return void
 */
void Planner::_construct_cellcosts(unsigned int Nc)
{
	for (unsigned int i = 0; i < Nc - 1; i++)
	{
		_cellcosts[i] = 255.34 * pow(Math::EUL, 0.0283 * (i - 1) / (101 - 1) * (7 - 1) + 1);
	}
	_cellcosts[Nc - 1] = Map::Cell::COST_UNWALKABLE;
}

/**
 * Generates interpolation lookup table for quickly aquiring cell costs.
 *
 * @param unsigned int Nc number of distinct traversal costs (including the infinite cost of traversing an obstacle cell)
 * @param unsigned int Mc maximum traversal cost of any traversable (i.e. non-obstacle) cell
 * @return void
 */
void Planner::_construct_interpolation_table(unsigned int Nc, unsigned int Mc)
{
	_construct_cellcosts(Nc);

	unsigned int ci, bi, f;
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
				if (f < b)
				{
					if (c <= f)
					{
						_I[ci][bi][f] = c * Math::SQRT2;
					}
					else
					{
						y = min((double)(f) / (sqrt(pow(c, 2) - pow(f, 2))), 1.0);
						_I[ci][bi][f] = c * sqrt(1.0 + pow(y, 2)) + f * (1.0 - y);
					}
				}
				else
				{
					if (Math::less(c, b) || Math::equals(c, b))
					{
						_I[ci][bi][f] = c * Math::SQRT2;
					}
					else
					{
						x = 1.0 - min(b / (sqrt(pow(c, 2) - pow(b, 2))), 1.0);
						_I[ci][bi][f] = c * sqrt(1.0 + pow((1.0 - x), 2));
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
		printf("Open list is empty!\n");
		return false;
	}

	KeyCompare key_compare;

	int attempts = 0;

	Map::Cell *u;
	Map::Cell *min_cell;
	Map::Cell **nbrs;
	Map::Cell **nbrsnbrs;
	double rhs_old, cost_interpol, min_cost, tmp_cost;
	pair<Map::Cell *, double> argmin_min;

	while ((!_open_list.empty() && key_compare(_open_list.begin()->first, _k(_start))) || !Math::equals(_rhs(_start), _g(_start)))
	{
		// Reached max steps, quit
		if (++attempts > Planner::MAX_STEPS)
		{
			printf("Reached maximum planner attempts!\n");
			return false;
		}
		u = _open_list.begin()->second;
		nbrs = u->nbrs();
		if (Math::greater(_g(u), _rhs(u)))
		{
			_g(u, _rhs(u));
			_list_remove(u);
			for (unsigned int i = 0; i < Map::Cell::NUM_NBRS; i++)
			{
				if (nbrs[i] != nullptr)
				{
					if (_cell_hash.find(nbrs[i]) == _cell_hash.end())
					{
						_cell(nbrs[i]);
					}
					if (nbrs[i]->ccknbr(u) != nullptr)
					{
						cost_interpol = _compute_cost(nbrs[i], u, nbrs[i]->ccknbr(u));
						if (Math::greater(_rhs(nbrs[i]), cost_interpol))
						{
							_rhs(nbrs[i], cost_interpol);
							nbrs[i]->bptr(u);
						}
					}
					if (nbrs[i]->cknbr(u) != nullptr)
					{
						cost_interpol = _compute_cost(nbrs[i], u, nbrs[i]->cknbr(u));
						if (Math::greater(_rhs(nbrs[i]), cost_interpol))
						{
							_rhs(nbrs[i], _compute_cost(nbrs[i], nbrs[i]->cknbr(u), u));
							nbrs[i]->bptr(nbrs[i]->cknbr(u));
						}
					}
					_update_state(nbrs[i]);
				}
			}
		}
		else
		{
			_g(u, Math::INF);
			for (unsigned int i = 0; i < Map::Cell::NUM_NBRS; i++)
			{
				if (nbrs[i] != nullptr)
				{
					if (nbrs[i]->bptr() == u || nbrs[i]->bptr() == nbrs[i]->cknbr(u))
					{
						nbrsnbrs = nbrs[i]->nbrs();
						min_cost = Math::INF;
						min_cell = nullptr;
						for (unsigned int j = 0; j < Map::Cell::NUM_NBRS; j++)
						{
							if (nbrsnbrs[j] != nullptr)
							{
								tmp_cost = _compute_cost(nbrs[i], nbrsnbrs[j], nbrs[i]->ccknbr(nbrsnbrs[j]));

								if (Math::less(tmp_cost, min_cost))
								{
									min_cell = nbrsnbrs[j];
									min_cost = tmp_cost;
								}
							}
						}
						if (min_cell == nullptr || Math::equals(min_cost, Math::INF))
						{
							// printf("Compute Shortest Path - Minimum cell was not found or had infinite cost.\n");
						}
						_rhs(nbrs[i], min_cost);
						nbrs[i]->bptr(min_cell);
						_update_state(nbrs[i]);
					}
				}
			}
			_update_state(u);
		}

		// u = _open_list.begin()->second;
		// nbrs = u->nbrs();
		// _list_remove(u);
		// if (Math::greater(_g(u), _rhs(u)))
		// {
		// 	_g(u, _rhs(u));
		// 	for (unsigned int i = 0; i < Map::Cell::NUM_NBRS; i++)
		// 	{
		// 		if (nbrs[i] != nullptr)
		// 		{
		// 			_update_state(nbrs[i]);
		// 		}
		// 	}
		// }
		// else
		// {
		// 	_g(u, Math::INF);
		// 	for (unsigned int i = 0; i < Map::Cell::NUM_NBRS; i++)
		// 	{
		// 		if (nbrs[i] != nullptr)
		// 		{
		// 			_update_state(nbrs[i]);
		// 		}
		// 	}
		// 	_update_state(u);
		// }

		// u = _open_list.begin()->second;
		// if (Math::greater(_g(u), _rhs(u)) || Math::equals(_g(u), _rhs(u)))
		// {
		// 	printf("Entering If statement of _compute_shortest_path.\n");
		// 	_g(u, _rhs(u));
		// 	_list_remove(u);
		// 	nbrs = u->nbrs();
		// 	printf("Entering For loop of If statement of _compute_shortest_path.\n");
		// 	for (unsigned int i = 0; i < Map::Cell::NUM_NBRS; i++)
		// 	{
		// 		if (nbrs[i] != nullptr)
		// 		{
		// 			if (_cell_hash.find(nbrs[i]) == _cell_hash.end())
		// 			{
		// 				_cell(nbrs[i]);
		// 			}
		// 			rhs_old = _rhs(nbrs[i]);
		// 			if (nbrs[i]->ccknbr(u) != nullptr)
		// 			{
		// 				// printf("_compute_cost(nbrs[i], u, nbrs[i]->ccknbr(u))\n"); //marked
		// 				cost_interpol = _compute_cost(nbrs[i], u, nbrs[i]->ccknbr(u));
		// 				if (Math::greater(_rhs(nbrs[i]), cost_interpol))
		// 				{
		// 					_rhs(nbrs[i], cost_interpol);
		// 					nbrs[i]->bptr(u);
		// 				}
		// 			}
		// 			if (nbrs[i]->cknbr(u) != nullptr)
		// 			{
		// 				// printf("_compute_cost(nbrs[i], u, nbrs[i]->cknbr(u))\n"); //marked
		// 				cost_interpol = _compute_cost(nbrs[i], u, nbrs[i]->cknbr(u));
		// 				if (Math::greater(_rhs(nbrs[i]), cost_interpol))
		// 				{
		// 					// printf("_compute_cost(nbrs[i], nbrs[i]->cknbr(u), u)\n"); // marked
		// 					_rhs(nbrs[i], _compute_cost(nbrs[i], nbrs[i]->cknbr(u), u));
		// 					nbrs[i]->bptr(nbrs[i]->cknbr(u));
		// 				}
		// 			}
		// 			if (!Math::equals(_rhs(u), rhs_old))
		// 			{
		// 				_update_state(nbrs[i]);
		// 			}
		// 		}
		// 	}
		// 	printf("Exiting If statement of _compute_shortest_path.\n");
		// }
		// else
		// {
		// 	printf("Entering ELSE statement of _compute_shortest_path!!!\n");
		// 	argmin_min = _min_interpol_succ(u);
		// 	_rhs(u, argmin_min.second);
		// 	u->bptr(argmin_min.first);
		// 	if (Math::less(_g(u), _rhs(u)))
		// 	{
		// 		_g(u, Math::INF);
		// 		nbrs = u->nbrs();
		// 		for (unsigned int i = 0; i < Map::Cell::NUM_NBRS; i++)
		// 		{
		// 			if (nbrs[i] != nullptr)
		// 			// if (nbrs[i] != nullptr && nbrs[i]->bptr() != nullptr && nbrs[i]->cknbr(u) != nullptr && nbrs[i]->ccknbr(nbrs[i]->bptr()) != nullptr)
		// 			{
		// 				if (nbrs[i]->bptr() == u || nbrs[i]->bptr() == nbrs[i]->cknbr(u))
		// 				{
		// 					// printf("_compute_cost(nbrs[i], nbrs[i]->bptr(), nbrs[i]->ccknbr(nbrs[i]->bptr()))\n");
		// 					if (!Math::equals(_rhs(nbrs[i]), _compute_cost(nbrs[i], nbrs[i]->bptr(), nbrs[i]->ccknbr(nbrs[i]->bptr()))))
		// 					{
		// 						if (Math::less(_g(nbrs[i]), _rhs(nbrs[i])) || _open_hash.find(nbrs[i]) == _open_hash.end())
		// 						{
		// 							_rhs(nbrs[i], Math::INF);
		// 							_update_state(nbrs[i]);
		// 						}
		// 						else
		// 						{
		// 							argmin_min = _min_interpol_succ(nbrs[i]);
		// 							_rhs(nbrs[i], argmin_min.second);
		// 							nbrs[i]->bptr(argmin_min.first);
		// 							_update_state(nbrs[i]);
		// 						}
		// 					}
		// 				}
		// 			}
		// 		}
		// 	}
		// 	_update_state(u);
		// 	printf("Exiting ELSE statement of _compute_shortest_path!!!\n");
		// }
	}
	// if (_open_list.empty())
	// {
	// 	printf("Open list empty\n");
	// }
	// if (!key_compare(_open_list.begin()->first, _k(_start)))
	// {
	// 	printf("!key_compare\n");
	// }
	// if (Math::equals(_rhs(_start), _g(_start)))
	// {
	// 	printf("_rhs(_start) = %lf\t_g(_start) = %lf\n", _rhs(_start), _g(_start));
	// }
	printf("Attempts: %d", attempts);
	return true;
}

/**
 * Calculates the interpolated cost of 's' given any two consecutive neighbors 'sa' and 'sb'.
 *
 * @param Map::Cell* cell s
 * @param Map::Cell* cell sa
 * @param Map::Cell* cell sb
 * @return double cost of s
 */
double Planner::_compute_cost(Map::Cell *s, Map::Cell *sa, Map::Cell *sb)
{
	// printf("Computing interpolated cost.\n");
	if (s == nullptr || sa == nullptr || sb == nullptr)
	{
		printf("NULL CELL WHEN COMPUTING COST!\n");
		return Math::INF;
	}
	Map::Cell **nbrs = s->nbrs();
	Map::Cell *s1;
	Map::Cell *s2;
	bool is_diag = (abs((int)(s->x()) - (int)(sa->x())) + abs((int)(s->y()) - (int)(sa->y()))) > 1;
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
	// printf("(sx: %u, sy: %u), (sax: %u, say: %u), (sbx: %u, sby: %u)\n", s->x(), s->y(), sa->x(), sa->y(), sb->x(), sb->y());
	// printf("(snb0x: %u, snb0y: %u), (snb1x: %u, snb1y: %u), (snb2x: %u, snb2y: %u), (snb3x: %u, snb3y: %u), (snb4x: %u, snb4y: %u), (snb5x: %u, snb5y: %u), (snb6x: %u, snb6y: %u), (snb7x: %u, snb7y: %u)\n", nbrs[0]->x(), nbrs[0]->y(), nbrs[1]->x(), nbrs[1]->y(), nbrs[2]->x(), nbrs[2]->y(), nbrs[3]->x(), nbrs[3]->y(), nbrs[4]->x(), nbrs[4]->y(), nbrs[5]->x(), nbrs[5]->y(), nbrs[6]->x(), nbrs[6]->y(), nbrs[7]->x(), nbrs[7]->y());

	double b, c;
	int dx1 = s1->x() - s->x();
	int dy1 = -(s1->y() - s->y());
	int dx2 = s2->x() - s->x();
	int dy2 = -(s2->y() - s->y());
	if (dx1 == 1 && dy1 == 0 && dx2 == 1 && dy2 == 1)
	{
		c = s->cost;
		b = nbrs[1]->cost;
	}
	else if (dx1 == 0 && dy1 == 1 && dx2 == 1 && dy2 == 1)
	{
		c = s->cost;
		b = nbrs[7]->cost;
	}
	else if (dx1 == 0 && dy1 == 1 && dx2 == -1 && dy2 == 1)
	{
		c = nbrs[7]->cost;
		b = s->cost;
	}
	else if (dx1 == -1 && dy1 == 0 && dx2 == -1 && dy2 == 1)
	{
		c = nbrs[7]->cost;
		b = nbrs[0]->cost;
	}
	else if (dx1 == -1 && dy1 == 0 && dx2 == -1 && dy2 == -1)
	{
		c = nbrs[0]->cost;
		b = nbrs[7]->cost;
	}
	else if (dx1 == 0 && dy1 == -1 && dx2 == -1 && dy2 == -1)
	{
		c = nbrs[0]->cost;
		b = nbrs[1]->cost;
	}
	else if (dx1 == 0 && dy1 == -1 && dx2 == 1 && dy2 == -1)
	{
		c = nbrs[1]->cost;
		b = nbrs[0]->cost;
	}
	else if (dx1 == 1 && dy1 == 0 && dx2 == 1 && dy2 == -1)
	{
		c = nbrs[1]->cost;
		b = s->cost;
	}
	else
	{
		// printf("sx: %u sy: %u sax: %u say: %u sbx: %u sby: %u\n", s->x(), s->y(), sa->x(), sa->y(), sb->x(), sb->y());
		// printf("dx1: %d\tdy1: %d\tdx2: %d\tdy2: %d\n", dx1, dy1, dx2, dy2);
		printf("ERROR COMPUTING COST!\n");
		return Math::INF;
	}
	double g1 = _g(s1);
	double g2 = _g(s2);
	double vs;
	if (Math::equals(min(c, b), Map::Cell::COST_UNWALKABLE))
	{
		vs = Math::INF;
	}
	else if (Math::less(g1, g2) || Math::equals(g1, g2))
	{
		vs = min(c, b) + g1;
		// printf("Interpolation cost: %lf\n", vs);
	}
	else
	{
		double f = g1 - g2;
		if (Math::less(f, b) || Math::equals(f, b))
		{
			if (Math::less(c, f) || Math::equals(c, f))
			{
				vs = c * Math::SQRT2 + g2;
				// printf("Interpolation cost: %lf\n", vs);
			}
			else
			{
				double y = min(f / (sqrt(pow(c, 2) - pow(f, 2))), 1.0);
				vs = c * sqrt(1.0 + pow(y, 2)) + f * (1.0 - y) + g2;
				// printf("Interpolation cost: %lf\n", vs);
			}
		}
		else
		{
			if (Math::less(c, b) || Math::equals(c, b))
			{
				vs = c * Math::SQRT2 + g2;
				// printf("Interpolation cost: %lf\n", vs);
			}
			else
			{
				double x = 1.0 - min(b / (sqrt(pow(c, 2) - pow(b, 2))), 1.0);
				vs = c * sqrt(1.0 + pow((1.0 - x), 2)) + b * x + g2;
				// printf("Interpolation cost: %lf\n", vs);
			}
		}
	}

	// printf("Finished computing interpolated cost.\n");
	return vs;
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
	return (0.5 * std::sqrt(std::pow(b->x() - a->x(), 2) + std::pow(b->y() - a->y(), 2)));
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
	printf("Finding minimum interpolated successor.\n");
	Map::Cell **nbrs = u->nbrs();

	double tmp_cost, tmp_g;

	Map::Cell *min_cell = nullptr;
	double min_cost = Math::INF;

	for (unsigned int i = 0; i < Map::Cell::NUM_NBRS; i++)
	{
		if (nbrs[i] != nullptr)
		// if (nbrs[i] != nullptr && u->ccknbr(nbrs[i]) != nullptr)
		{
			// printf("_compute_cost(u, nbrs[i], u->ccknbr(nbrs[i]))\n");
			tmp_cost = _compute_cost(u, nbrs[i], u->ccknbr(nbrs[i]));

			if (Math::less(tmp_cost, min_cost))
			{
				min_cell = nbrs[i];
				min_cost = tmp_cost;
			}
		}
	}
	if (min_cell != nullptr)
	{
		printf("Found minimum interpolated successor.\n");
	}
	else
	{
		printf("Minimum interpolated successor was nullptr!\n");
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
	// if (_cell_hash.find(u) == _cell_hash.end())
	// {
	// 	_cell(u);
	// }
	// if (u != _goal)
	// {
	// 	Map::Cell **nbrs = u->nbrs();

	// 	double tmp_cost;

	// 	double min_cost = Math::INF;

	// 	for (unsigned int i = 0; i < Map::Cell::NUM_NBRS; i++)
	// 	{
	// 		if (nbrs[i] != nullptr)
	// 		{
	// 			tmp_cost = _compute_cost(u, nbrs[i], u->ccknbr(nbrs[i]));

	// 			if (Math::less(tmp_cost, min_cost))
	// 			{
	// 				min_cost = tmp_cost;
	// 			}
	// 		}
	// 	}
	// 	_rhs(u, min_cost);
	// }
	// if (_open_hash.find(u) != _open_hash.end())
	// {
	// 	_list_remove(u);
	// }
	// if (!Math::equals(_g(u), _rhs(u)))
	// {
	// 	_list_insert(u, _k(u));
	// }

	if (!Math::equals(_g(u), _rhs(u)) && _open_hash.find(u) != _open_hash.end())
	{
		_list_update(u, _k(u));
	}
	else if (!Math::equals(_g(u), _rhs(u)) && _open_hash.find(u) == _open_hash.end())
	{
		_list_insert(u, _k(u));
	}
	else if (Math::equals(_g(u), _rhs(u)) && _open_hash.find(u) != _open_hash.end())
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