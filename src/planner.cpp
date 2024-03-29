#include "planner.h"

/**
 * @var static const double [radians] minimum backpointer gradient angle before resorting to one-step lookahead
 */
const double Planner::GRAD_ANG_MIN = Math::PI / 9.0;

/**
 * @var static const double [radians] maximum backpointer gradient angle before resorting to one-step lookahead
 */
const double Planner::GRAD_ANG_MAX = Math::PI / 6.0;

/**
 * @var static const int max steps before assuming no solution possible
 */
const int Planner::MAX_STEPS = 1000000;

/**
 * @var static const int max steps before giving up on path extraction
 */
const int Planner::MAX_EXTRACTION_STEPS = 250;

/**
 * Constructor.
 * All planner instances share the same instance of the very memory hungry cellcosts and interpolation tables.
 * Saving as a reference instead of a pointer also lets us avoid using the ugly ->operator[]() syntax
 *
 * @param Map* map
 * @param Map::Cell* start cell
 * @param Map::Cell* goal cell
 * @param vector<double> & cellcosts table
 * @param vector<vector<vector<vector<double>>>> & interpolation table
 */
Planner::Planner(Map *map, Map::Cell *start, Map::Cell *goal, vector<double> &cellcosts, vector<vector<vector<vector<double>>>> &I)
	: _cellcosts(cellcosts), _I(I)
{
	printf("Initializing Field D* Planner\n");

	// Clear lists
	_interpol_path.clear();
	_open_list.clear();
	_open_hash.clear();
	_primary_cp_list.clear();
	_primary_cp_hash.clear();
	_secondary_cp_list.clear();
	_secondary_cp_hash.clear();

	_map = map;
	_start = start;
	_goal = goal;
	_cellcosts = cellcosts;
	_I = I;
	_Nc = cellcosts.size() - 1;
	_Mc = static_cast<int>(_cellcosts[_Nc - 1]);

	printf("_Nc: %d, _Mc: %d\n", _Nc, _Mc);

	_rhs(_goal, 0.0);
	_h(_goal, 0.0);
	_list_insert(_goal, pair<double, double>(_heuristic(_start, _goal), 0));

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
	int cols = _map->cols();
	int rows = _map->rows();
	vector<double> gmap;
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++)
			gmap.push_back(_g((*_map)(i, j)));
	return gmap;
}

/**
 * Returns a distance-to-start map
 *
 * @return vector<double> map
 */
vector<double> Planner::h_map()
{
	int cols = _map->cols();
	int rows = _map->rows();
	vector<pair<int, int>> coords;
	vector<double> hmap;
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++)
		{
			hmap.push_back(_h((*_map)(i, j)));
			coords.push_back(make_pair(i, j));
		}

	double max = 0.0;
	for (int i = 0; i < hmap.size(); i++)
		if (!Math::equals(hmap[i], Math::INF))
			if (Math::greater(hmap[i], max))
				max = hmap[i];

	if (!Math::equals(max, 0.0))
		for (int i = 0; i < hmap.size(); i++)
			if (!Math::equals(hmap[i], Math::INF))
				hmap[i] = max - hmap[i];

	ofstream file1;
	file1.open("heuristic_hash_coords1.txt");
	for (int i = 0; i < coords.size(); i++)
		file1 << coords[i].first << endl;
	file1.close();

	ofstream file2;
	file2.open("heuristic_hash_coords2.txt");
	for (int i = 0; i < coords.size(); i++)
		file2 << coords[i].second << endl;
	file2.close();

	ofstream file3;
	file3.open("heuristic_hash_val.txt");
	for (int i = 0; i < hmap.size(); i++)
	{
		if (Math::equals(hmap[i], Math::INF))
			file3 << -1 << endl;
		else
			file3 << hmap[i] << endl;
	}
	file3.close();

	return hmap;
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
		return false;

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
	int cols = _map->cols();
	int rows = _map->rows();
	vector<double> rhsmap;
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++)
			rhsmap.push_back(_rhs((*_map)(i, j)));
	return rhsmap;
}

/**
 * Gets/Sets start.
 *
 * @param pair<double, double> *u_d [optional] new start
 * @return pair<double, double> * start
 */
pair<double, double> Planner::start(pair<double, double> u_d)
{
	_current = u_d;
	_start = (*_map)(static_cast<int>(u_d.second), static_cast<int>(u_d.first));

	return _current;
}

/**
 * Update map.
 *
 * @param Map::Cell* cell to update
 * @param int new cost of the cell
 * @param bool unknown flip whether or not the unknown status of the cell has flipped
 * @return void
 */
void Planner::update_cell_cost(Map::Cell *u, int cost, bool unknown_flip)
{
	int c_old = u->cost;
	u->cost = cost;

	double rhs_min;

	Map::Cell *u_new;
	Map::Cell **cnrs = u->cnrs();
	pair<Map::Cell *, double> argmin_min;

	// we want to check if the unknown cost status has changed in case the new cost is less than the unknown cell cost
	if (cost > c_old || unknown_flip)
	{
		for (int i = 0; i < Map::Cell::NUM_CNRS; i++)
		{
			if (cnrs[i] != nullptr && cnrs[i]->bptr() != nullptr && cnrs[i]->ccknbr(cnrs[i]->bptr()) != nullptr)
			{
				if (u->is_corner(cnrs[i]->bptr()) || u->is_corner(cnrs[i]->ccknbr(cnrs[i]->bptr())))
				{
					if (!Math::equals(_rhs(cnrs[i]), _compute_cost(cnrs[i], cnrs[i]->bptr(), cnrs[i]->ccknbr(cnrs[i]->bptr()))))
					{
						if (Math::less(_g(cnrs[i]), _rhs(cnrs[i])) || _open_hash.find(cnrs[i]) == _open_hash.end())
						{
							_rhs(cnrs[i], Math::INF);
							// _h(cnrs[i], Math::INF);
							_update_state(cnrs[i]);
						}
						else
						{
							argmin_min = _min_interpol_succ(cnrs[i]);
							_rhs(cnrs[i], argmin_min.second);
							cnrs[i]->bptr(argmin_min.first);
							if (!Math::equals(argmin_min.second, Math::INF))
								_h(cnrs[i], get<3>(_compute_bp(cnrs[i], argmin_min.first, cnrs[i]->ccknbr(argmin_min.first))));
							else
								_h(cnrs[i], Math::INF);
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
		for (int i = 0; i < Map::Cell::NUM_CNRS; i++)
		{
			if (cnrs[i] != nullptr)
			{
				if (_cell_hash.find(cnrs[i]) == _cell_hash.end())
					_cell(cnrs[i]);
				else if (Math::less(_rhs(cnrs[i]), rhs_min))
				{
					rhs_min = _rhs(cnrs[i]);
					u_new = cnrs[i];
				}
			}
		}
		if (!Math::equals(rhs_min, Math::INF))
		{
			if (_open_hash.find(u_new) != _open_hash.end())
				_list_update(u_new, _k(u_new));
			else
				_list_insert(u_new, _k(u_new));
		}
	}
}

/** Calculates the cost of traversing the path
 *
 * @return double the path cost
 */
double Planner::_calculate_path_cost(vector<pair<double, double>> &path)
{
	double total_cost;
	int n, m, nci;

	total_cost = 0.0;

	for (int i = 1; i < path.size(); i++)
	{
		if (Math::equals(path[i - 1].first, path[i].first) && Math::equals(path[i].first, floor(path[i].first))) // path segment is on a vertical edge
		{
			n = static_cast<int>(min(path[i].second, path[i - 1].second)); // this is the row of the map grid

			// the edge is between map[n][path->x[i]-1] and map[][path->x[i], we want to use the least cost of the two
			if (Math::less(path[i].first - 1.0, 0.0)) // then on edge of map
				nci = (*_map)(n, static_cast<int>(path[i].first))->cost;
			else if (static_cast<int>(path[i].first) == _map->cols() - 1) // on other edge of map
				nci = (*_map)(n, static_cast<int>(path[i].first) - 1)->cost;
			else if ((*_map)(n, static_cast<int>(path[i].first) - 1)->cost < (*_map)(n, static_cast<int>(path[i].first))->cost)
				nci = (*_map)(n, static_cast<int>(path[i].first) - 1)->cost;
			else
				nci = (*_map)(n, static_cast<int>(path[i].first))->cost;

			total_cost += _cellcosts[nci] * (abs(path[i].second - path[i - 1].second));
		}
		else if (Math::equals(path[i - 1].second, path[i].second) && Math::equals(path[i].second, floor(path[i].second))) // path segment is on a horizontal edge
		{
			m = static_cast<int>(min(path[i].first, path[i - 1].first)); // this is the column of the map grid

			// the edge is between map[path->y[i]-1][m] and map[path->y[i][m], we want to use the least cost of the two
			if (Math::less(path[i].second - 1.0, 0.0)) // then on edge of map
				nci = (*_map)(static_cast<int>(path[i].second), m)->cost;
			else if (static_cast<int>(path[i].second) == _map->rows() - 1) // on other edge of map
				nci = (*_map)(static_cast<int>(path[i].second) - 1, m)->cost;
			else if ((*_map)(static_cast<int>(path[i].second) - 1, m)->cost < (*_map)(static_cast<int>(path[i].second), m)->cost)
				nci = (*_map)(static_cast<int>(path[i].second) - 1, m)->cost;
			else
				nci = (*_map)(static_cast<int>(path[i].second), m)->cost;

			total_cost += _cellcosts[nci] * (abs(path[i].first - path[i - 1].first));
		}
		else // path segment goes through the grid map[floor(min(path->y[i],path->y[i-1]))] [floor(min(path->x[i],path->x[i-1]))]
		{
			total_cost += (sqrt(((path[i].second - path[i - 1].second) * (path[i].second - path[i - 1].second)) + ((path[i].first - path[i - 1].first) * (path[i].first - path[i - 1].first)))) * round(_cellcosts[(*_map)(static_cast<int>(min(path[i].second, path[i - 1].second)), static_cast<int>(min(path[i].first, path[i - 1].first)))->cost]);
		}
	}
	return total_cost;
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
	_heuristic_hash[u] = h;
}

/**
 * Calculates the interpolated [x, y] position of a backpointer of 's' given any two consecutive neighbors 'sa' and 'sb'.
 *
 * @param Map::Cell* cell s
 * @param Map::Cell* cell sa
 * @param Map::Cell* cell sb
 * @return tuple<double, double, int, double> position of bptr of s and type of path (1 = direct, 2 = more than one path segment) and shortest path length
 */
tuple<double, double, int, double> Planner::_compute_bp(Map::Cell *s, Map::Cell *sa, Map::Cell *sb)
{
	double x, y, distance, tmp;
	int path_type;

	x = 1.0;
	y = 1.0;
	distance = Math::INF;
	path_type = 1;

	if (s == nullptr || sa == nullptr || sb == nullptr)
	{
		x = 0.0;
		y = 0.0;
		path_type = 2;
		printf("ERROR: NULLTR CELL FOUND WHEN COMPUTING COST BPTR\n");
	}
	else
	{
		double c, b, d1, d2, g1, g2, f;
		int ci, bi, by, bx;
		Map::Cell *s1;
		Map::Cell *s2;

		if (s->x() == sb->x() || s->y() == sb->y())
		{
			s1 = sb;
			s2 = sa;
		}
		else
		{
			s1 = sa;
			s2 = sb;
		}

		ci = (*_map)(Math::min3(s->y(), s1->y(), s2->y()), Math::min3(s->x(), s1->x(), s2->x()))->cost;
		by = min(s->y(), s1->y());
		bx = min(s->x(), s1->x());

		if (s1->y() < s2->y())
			by--;
		else if (s1->x() < s2->x())
			bx--;

		if (by == -1 || bx == -1 || by == _map->rows() - 1 || bx == _map->cols() - 1)
			bi = ci;
		else
			bi = (*_map)(by, bx)->cost;

		if (ci > _Nc || bi > _Nc)
		{
			x = 0.0;
			y = 0.0;
			path_type = 2;
			printf("ERROR: INVALID COST INDEX WHEN COMPUTING COST\n");
		}
		else
		{
			c = _cellcosts[ci];
			b = _cellcosts[bi];

			if (Math::less(c, Map::Cell::COST_UNWALKABLE) || Math::less(b, Map::Cell::COST_UNWALKABLE))
			{
				c = round(c);
				b = round(b);
				g1 = round(_g(s1));
				g2 = round(_g(s2));
				d1 = _h(s1);
				d2 = _h(s2);

				if (Math::less(g1, g2) || Math::equals(g1, g2))
				{
					// goes straight to s1
					y = 0.0;
					distance = 1.0 + d1;
				}
				else
				{
					// f is the cost of traversing the right edge
					f = round(g1 - g2);

					if (Math::greater(f, min(c, b)))
					{
						// travel a distance x along the bottom edge then take a straight-line path to s2
						x = _I[ci][bi][_Mc][1];

						if (Math::equals(x, 1.0)) // then path spends some time in both grids b and c
							path_type = 2;
						distance = x + sqrt(pow(1.0 - x, 2) + 1.0) + d2;
					}
					else
					{
						// take a straight-line path from s to some point sy on the right edge
						y = _I[ci][bi][static_cast<int>(f)][2];
						distance = sqrt(1.0 + pow(y, 2)) + d2;
					}
				}

				if (s->y() == s1->y()) // s_1 is to the left or right of s
				{
					if (s1->x() > s->x()) // s_1 is to the right of s
					{
						if (s2->y() > s1->y()) // s_2 is below (greater than) s_1
						{
						}
						else // s_2 is above (less than) s_1
							y = -y;
					}
					else // s_1 is to the left of s
					{
						x = -x;
						if (s2->y() > s1->y()) // s_2 is below (greater than) s_1
						{
						}
						else // s_2 is above (less than) s_1
							y = -y;
					}
				}
				else // s_1 is up or down of s (x and y are swapped)
				{
					tmp = x;
					x = y;
					y = tmp;

					if (s1->y() > s->y()) // s_1 is below (greater than) s
					{
						if (s2->x() > s1->x()) // s_2 is to the right of s_1
						{
						}
						else // s_2 is to the left of s_1
							x = -x;
					}
					else // s_1 is above (less than) s
					{
						y = -y;
						if (s2->x() > s1->x()) // s_2 is to the right of s_1
						{
						}
						else // s_2 is to the left of s_1
							x = -x;
					}
				}
			}
			else
			{
				x = 0.0;
				y = 0.0;
				path_type = 2;
			}
		}
	}

	return make_tuple(y, x, path_type, distance);
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
	double vs = Math::INF;

	if (s == nullptr || sa == nullptr || sb == nullptr)
		printf("ERROR: NULLTR CELL FOUND WHEN COMPUTING COST\n");
	else
	{
		double c, b, g1, g2, f;
		int ci, bi, by, bx;
		Map::Cell *s1;
		Map::Cell *s2;

		if (s->x() == sb->x() || s->y() == sb->y())
		{
			s1 = sb;
			s2 = sa;
		}
		else
		{
			s1 = sa;
			s2 = sb;
		}

		ci = (*_map)(Math::min3(s->y(), s1->y(), s2->y()), Math::min3(s->x(), s1->x(), s2->x()))->cost;
		by = min(s->y(), s1->y());
		bx = min(s->x(), s1->x());

		if (s1->y() < s2->y())
			by--;
		else if (s1->x() < s2->x())
			bx--;

		if (by == _map->rows() - 1 || bx == _map->cols() - 1 || by == -1 || bx == -1)
			bi = ci;
		else
			bi = (*_map)(by, bx)->cost;

		if (ci > _Nc || bi > _Nc)
			printf("ERROR: INVALID COST INDEX WHEN COMPUTING COST\n");
		else
		{
			c = _cellcosts[ci];
			b = _cellcosts[bi];

			if (Math::less(c, Map::Cell::COST_UNWALKABLE) || Math::less(b, Map::Cell::COST_UNWALKABLE))
			{
				c = round(c);
				b = round(b);
				g1 = round(_g(s1));
				g2 = round(_g(s2));

				if (Math::less(g1, g2) || Math::equals(g1, g2))
					// goes straight to s1
					vs = round(min(c, b) + g1);
				else
				{
					// f is the cost of traversing the right edge
					f = round(g1 - g2);

					if (Math::greater(f, min(c, b)))
						// travel a distance x along the bottom edge then take a straight-line path to s2
						vs = round(_I[ci][bi][_Mc][0] + g2);
					else
						// take a straight-line path from s to some point sy on the right edge
						vs = round(_I[ci][bi][static_cast<int>(f)][0] + g2);
				}
			}
		}
	}

	return vs;
}

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
void Planner::_compute_cost_of_point_to_edge(double y_hat, double x_hat, Map::Cell *s, Map::Cell *s_a, Map::Cell *s_b, Map::CellPath **sub_paths)
{
	// printf("Computing a cost to edge\n");

	double c, b, x_1, x_2, ybar_1, ybar_2, g_1, g_2, diagonal_g, straight_g, c_prime;
	Map::CellPath *sub_path = sub_paths[0];
	Map::Cell *s_1;
	Map::Cell *s_2;
	int b_y, b_x;

	if (s == nullptr || s_a == nullptr || s_b == nullptr)
	{
		printf("ERROR: NULLPTR FOUND IN COMPUTE COST TO EDGE\n");
		sub_path->g = Math::INF;
		sub_path->length = 0;
		return;
	}

	if (s->x() == s_b->x() || s->y() == s_b->y()) // then s_b is horizontal or vertical neighbor of s, thus, s_a is a diagonal neighbor of s
	{
		s_1 = s_b;
		s_2 = s_a;
	}
	else if (s->x() == s_b->x() && s->y() == s_b->y())
		printf("ERROR: THIS SHOULD NEVER HAPPEN 0\n");
	else
	{
		s_1 = s_a;
		s_2 = s_b;
	}

	// c is traversal cost of map cell with corners s, s_1, s_2
	c = _cellcosts[((*_map)(Math::min3(s->y(), s_1->y(), s_2->y()), Math::min3(s->x(), s_1->x(), s_2->x())))->cost];

	// b is traversal cost of map cell with corners s and s_1, not s_2
	b_y = min(s->y(), s_1->y());
	b_x = min(s->x(), s_1->x());

	// the above assumes that either c is above b or c is to the left of b, we need to modify things if this is not the case
	if (s_1->y() < s_2->y()) // then c is actually below b
		b_y--;
	else if (s_1->x() < s_2->x()) // then c is actually to the right of b
		b_x--;

	// need to take care of edge cases (WIDTH or HEIGHT)
	if (b_y == _map->rows() - 1 || b_x == _map->cols() - 1 || b_y == -1 || b_x == -1)
		b = c;
	else
		b = _cellcosts[((*_map)(b_y, b_x))->cost];

	// find min cost of path going only through cell c, need to find the actual of the two possibilities returned by the quadratic equation
	// printf(" here 1 [%f %f] %f %f %f \n", x_hat, y_hat, s_1->g, s_2->g, c);

	// remember the original node values, because we will modify them locally if necessary (see below) but want to leave them the same after we are done
	double tmp_s1_g = _g(s_1);
	double tmp_s2_g = _g(s_2);

	if (Math::equals(tmp_s1_g, Math::INF) && Math::equals(tmp_s2_g, Math::INF))
		printf("ERROR: THIS SHOULD NEVER HAPPEN 1\n"); // except possible on the first path step of path extraction
	else if (Math::greater(tmp_s1_g, tmp_s2_g + c))	   // to avoide getting imaginary answer, need to give this an appropriate g based on s_2
		tmp_s1_g = tmp_s2_g + c;
	else if (Math::greater(tmp_s2_g, tmp_s1_g + c)) // to avoid getting imaginary answer, need to give this an appropriate g based on s_1
		tmp_s2_g = tmp_s1_g + c;

	// printf(" here 2 [%f %f] %f %f %f \n", x_hat, y_hat, s_1->g, s_2->g, c);

	if (Math::equals(tmp_s1_g - tmp_s2_g, c) || Math::equals(tmp_s2_g - tmp_s1_g, c)) // then the equation explodes, but we know that the best option is to go straight at the minimum of g1 or g2
	{
		if (Math::less(tmp_s1_g, tmp_s2_g)) // go toward s_1
		{
			sub_path->x[0] = 0.0;
			sub_path->local_g = c * sqrt(((x_hat) * (x_hat)) + ((1.0 - y_hat) * (1.0 - y_hat)));
			sub_path->g = sub_path->local_g + tmp_s1_g;
		}
		else if (Math::less(tmp_s2_g, tmp_s1_g)) // go toward s_2
		{
			sub_path->x[0] = 1.0;
			sub_path->local_g = c * sqrt(((1.0 - x_hat) * (1.0 - x_hat)) + ((1.0 - y_hat) * (1.0 - y_hat)));
			sub_path->g = sub_path->local_g + tmp_s2_g;
		}
		else
			printf("ERROR: THIS SHOULD NEVER HAPPEN 2\n");
		sub_path->y[0] = 1.0;
		sub_path->g_to_edge = sub_path->local_g;
		sub_path->length = 1;
	}
	else
	{
		// x_1 = quadraticPlus(1, -2*x_hat, (x_hat*x_hat) - ((1-y_hat)*(1-y_hat))/(((c/(s_1->g - s_2->g))*(c/(s_1->g - s_2->g))) - 1));
		// x_2 = quadraticMinus(1, -2*x_hat, (x_hat*x_hat) - ((1-y_hat)*(1-y_hat))/(((c/(s_1->g - s_2->g))*(c/(s_1->g - s_2->g))) - 1));

		c_prime = sqrt(((1.0 - y_hat) * (1.0 - y_hat)) / (((c / (tmp_s1_g - tmp_s2_g)) * (c / (tmp_s1_g - tmp_s2_g))) - 1.0));
		x_1 = x_hat + c_prime;
		x_2 = x_hat - c_prime;

		x_1 = max(min(x_1, 1.0), 0.0);
		x_2 = max(min(x_2, 1.0), 0.0);

		g_1 = c * sqrt(((1.0 - y_hat) * (1.0 - y_hat)) + ((x_1 - x_hat) * (x_1 - x_hat))) + x_1 * tmp_s2_g + (1.0 - x_1) * tmp_s1_g;
		g_2 = c * sqrt(((1.0 - y_hat) * (1.0 - y_hat)) + ((x_2 - x_hat) * (x_2 - x_hat))) + x_2 * tmp_s2_g + (1.0 - x_2) * tmp_s1_g;

		if (Math::less(g_1, g_2))
		{
			sub_path->x[0] = x_1;
			sub_path->local_g = c * sqrt(((1.0 - y_hat) * (1.0 - y_hat)) + ((x_1 - x_hat) * (x_1 - x_hat)));
			sub_path->g = sub_path->local_g + x_1 * tmp_s2_g + (1.0 - x_1) * tmp_s1_g;
		}
		else
		{
			sub_path->x[0] = x_2;
			sub_path->local_g = c * sqrt(((1.0 - y_hat) * (1.0 - y_hat)) + ((x_2 - x_hat) * (x_2 - x_hat)));
			sub_path->g = sub_path->local_g + x_2 * tmp_s2_g + (1.0 - x_2) * tmp_s1_g;
		}
		sub_path->y[0] = 1.0;
		sub_path->g_to_edge = sub_path->local_g;
		sub_path->length = 1;
	}

	if (Math::equals(y_hat, 1.0)) // then the point is on the edge that we are trying to get to, nothing will be better than staying at that point
	{
		sub_path->length = 0;
		return;
	}

	if (Math::less(b, c)) // path may involve going through part of cell b (two options)
	{
		sub_path = sub_paths[1];

		// option 1, the path goes to b and then along b and then across c to s_2. This will only happen if s_2->g is less than s_1->g
		if (Math::less(tmp_s2_g, tmp_s1_g))
		{
			// need to find the actual of the two possibilities returned by the quadratic equation
			// ybar_1 = quadraticPlus(1,-2,1 - (((1+x_hat)*(1+x_hat))/(((c*c)/(b*b))-1)));
			// ybar_2 = quadraticPlus(1,-2,1 - (((1+x_hat)*(1+x_hat))/(((c*c)/(b*b))-1)));

			c_prime = sqrt(((1.0 + x_hat) * (1.0 + x_hat)) / (((c * c) / (b * b)) - 1.0));
			ybar_1 = 1.0 + c_prime;
			ybar_2 = 1.0 - c_prime;

			ybar_1 = max(min(ybar_1, 1.0), 0.0);
			ybar_2 = max(min(ybar_2, 1.0), 0.0);

			g_1 = b * (ybar_1 - y_hat) + c * sqrt(((1.0 - ybar_1) * (1.0 - ybar_1)) + ((1.0 + x_hat) * (1.0 + x_hat))) + tmp_s2_g;
			g_2 = b * (ybar_2 - y_hat) + c * sqrt(((1.0 - ybar_2) * (1.0 - ybar_2)) + ((1.0 + x_hat) * (1.0 + x_hat))) + tmp_s2_g;

			// need to compare to old g value to see if this is any better, if it is, then this path will eventually go to s_2
			if (Math::less(min(g_1, g_2), sub_path->g))
			{
				sub_path->x[0] = 0.0;
				sub_path->x[1] = 0.0;

				if (Math::less(g_1, g_2))
				{
					sub_path->y[0] = (((1.0 - ybar_1) * x_hat) / (1.0 + x_hat)) + y_hat;
					sub_path->y[1] = 1.0 - (1.0 - ybar_1) / (1.0 + x_hat);
					diagonal_g = c * sqrt(((1.0 - ybar_1) * (1.0 - ybar_1)) + ((1.0 + x_hat) * (1.0 + x_hat)));
					straight_g = b * (ybar_1 - y_hat);
				}
				else // g_2 <= g_1
				{
					sub_path->y[0] = (((1.0 - ybar_2) * x_hat) / (1.0 + x_hat)) + y_hat;
					sub_path->y[1] = 1.0 - (1.0 - ybar_2) / (1.0 + x_hat);
					diagonal_g = c * sqrt(((1.0 - ybar_2) * (1.0 - ybar_2)) + ((1.0 + x_hat) * (1.0 + x_hat)));
					straight_g = b * (ybar_2 - y_hat);
				}

				if (Math::equals(x_hat, 0.0)) // then the first point should be ignored because (sub_path->y[0] == y_hat && sub_path->x[0] == x_hat), also sub_path->g_to_edge == 0
				{
					sub_path->y[0] = sub_path->y[1];
					sub_path->x[0] = sub_path->x[1];
					sub_path->x[1] = 1.0;
					sub_path->y[1] = 1.0;
					sub_path->g_to_edge = straight_g;
					sub_path->length = 2;
				}
				else // all points are used
				{
					sub_path->x[2] = 1.0;
					sub_path->y[2] = 1.0;
					sub_path->g_to_edge = diagonal_g * x_hat / (1.0 + x_hat);
					sub_path->length = 3;
				}
				sub_path->local_g = diagonal_g + straight_g;
				sub_path->g = sub_path->local_g + tmp_s2_g;
			}
		}

		sub_path = sub_paths[2];

		// option 2, the path goes to b and then along b to s_1. This may be better even if s_2->g is less than s_1->g
		// need to find the actual of the two possibilities returned by the quadratic equation
		// ybar_1 = quadraticPlus(1,-2*y_hat, (y_hat*y_hat) - ((x_hat*x_hat)/(((c*c)/(b*b))-1)));
		// ybar_2 = quadraticPlus(1,-2*y_hat, (y_hat*y_hat) - ((x_hat*x_hat)/(((c*c)/(b*b))-1)));

		c_prime = sqrt((x_hat * x_hat) / (((c * c) / (b * b)) - 1.0));
		ybar_1 = y_hat + c_prime;
		ybar_2 = y_hat - c_prime;

		ybar_1 = max(min(ybar_1, 1.0), 0.0);
		ybar_2 = max(min(ybar_2, 1.0), 0.0);

		g_1 = b * (1.0 - ybar_1) + c * sqrt(((ybar_1 - y_hat) * (ybar_1 - y_hat)) + (x_hat * x_hat)) + tmp_s1_g;
		g_2 = b * (1.0 - ybar_2) + c * sqrt(((ybar_2 - y_hat) * (ybar_2 - y_hat)) + (x_hat * x_hat)) + tmp_s1_g;

		// need to compare to old g value to see if this is any better, if it is, then this path will eventually go to s_1
		if (Math::less(min(g_1, g_2), sub_path->g))
		{
			sub_path->x[0] = 0.0;

			if (Math::less(g_1, g_2))
			{
				sub_path->y[0] = ybar_1;
				sub_path->g_to_edge = c * sqrt(((ybar_1 - y_hat) * (ybar_1 - y_hat)) + (x_hat * x_hat));
				sub_path->local_g = sub_path->g_to_edge + b * (1.0 - ybar_1);
			}
			else // g_2 <= g_1
			{
				sub_path->y[0] = ybar_2;

				sub_path->g_to_edge = c * sqrt(((ybar_2 - y_hat) * (ybar_2 - y_hat)) + (x_hat * x_hat));
				sub_path->local_g = sub_path->g_to_edge + b * (1.0 - ybar_2);
			}

			if (Math::equals(x_hat, 0.0)) // then the first point should be ignored because (sub_path->y[0] == y_hat && sub_path->x[0] == x_hat), also sub_path->g_to_edge == 0
			{
				sub_path->x[0] = 0.0;
				sub_path->y[0] = 1.0;
				sub_path->g_to_edge = sub_path->local_g;
				sub_path->length = 1;
			}
			else // all points are used
			{
				sub_path->x[1] = 0.0;
				sub_path->y[1] = 1.0;
				sub_path->length = 2;
			}
			sub_path->g = sub_path->local_g + tmp_s1_g;
		}
	}
	// printf("Finished computing a cost to edge\n");
}

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
void Planner::_compute_local_point_cost(int cy, int cx, double y, double x, bool primary)
{
	// printf("Computing a local point cost\n");

	double x_hat, y_hat, temp_x, temp_y;
	Map::CellPath *sub_paths[3];
	Map::CellPath *sub_path;
	Map::Cell *s_a;
	Map::Cell *s_b;
	Map::Cell *s;
	int n, i, j;

	// there are 4 possible edges to go to (defined by s_a and s_b), and each
	// edge can potentially involve two other sides (defined by the remaining
	// two nodes), therefore we need to call computePointCostToEdge 8 times
	for (n = 0; n < Map::Cell::NUM_CNRS; n++)
	{
		if (cy + Map::Cell::DELTAY[n] < 0 || cy + Map::Cell::DELTAY[n] > _map->rows() - 1 || cx + Map::Cell::DELTAX[n] < 0 || cx + Map::Cell::DELTAX[n] > _map->cols() - 1)
			continue;
		s_a = (*_map)(cy + Map::Cell::DELTAY[n], cx + Map::Cell::DELTAX[n]);

		if (n == Map::Cell::NUM_CNRS - 1)
		{
			if (cy + Map::Cell::DELTAY[0] < 0 || cy + Map::Cell::DELTAY[0] > _map->rows() - 1 || cx + Map::Cell::DELTAX[0] < 0 || cx + Map::Cell::DELTAX[0] > _map->cols() - 1)
				continue;
			s_b = (*_map)(cy + Map::Cell::DELTAY[0], cx + Map::Cell::DELTAX[0]);
		}
		else
		{
			if (cy + Map::Cell::DELTAY[n + 1] < 0 || cy + Map::Cell::DELTAY[n + 1] > _map->rows() - 1 || cx + Map::Cell::DELTAX[n + 1] < 0 || cx + Map::Cell::DELTAX[n + 1] > _map->cols() - 1)
				continue;
			s_b = (*_map)(cy + Map::Cell::DELTAY[n + 1], cx + Map::Cell::DELTAX[n + 1]);
		}
		// at this point, s_b is clockwise of s_a, relative to the cell

		if (Math::equals(_g(s_a), Math::INF) && Math::equals(_g(s_b), Math::INF))
			continue;

		// case 1: s is the next neighbor of s_a that is clockwise of s_b
		s = s_a->cknbr(s_b);

		// the input to computePointCostToEdge is in relative and not absolute terms,
		// we need to figure out which way x_hat and y_hat go
		if (n == 0) // s_a is in the upper left
		{
			x_hat = 1.0 - x;
			y_hat = 1.0 - y;
		}
		else if (n == 1) // s_a is in the upper right
		{
			x_hat = 1.0 - y;
			y_hat = x;
		}
		else if (n == 2) // s_a is in the lower right
		{
			x_hat = x;
			y_hat = y;
		}
		else if (n == 3) // s_a is in the lower left
		{
			x_hat = y;
			y_hat = 1.0 - x;
		}
		// get CellPaths to put stuff in (there are 3 possibilities)
		for (j = 0; j < 3; j++)
			sub_paths[j] = new Map::CellPath(cy, cx);

		_compute_cost_of_point_to_edge(y_hat, x_hat, s, s_a, s_b, sub_paths);

		// the output from computePointCostToEdge is also relative, so we need to figure out which way best_y and best_x go
		for (j = 0; j < 3; j++)
		{
			sub_path = sub_paths[j];

			if (n == 0) // s_a is in the upper left
			{
				for (i = 0; i < sub_path->length; i++)
				{
					temp_x = 1.0 - sub_path->x[i];
					temp_y = 1.0 - sub_path->y[i];
					sub_path->x[i] = temp_x;
					sub_path->y[i] = temp_y;
				}
			}
			else if (n == 1) // s_a is in the upper right
			{
				for (i = 0; i < sub_path->length; i++)
				{
					temp_x = sub_path->y[i];
					temp_y = 1.0 - sub_path->x[i];
					sub_path->x[i] = temp_x;
					sub_path->y[i] = temp_y;
				}
			}
			else if (n == 2) // s_a is in the lower right
			{
				for (i = 0; i < sub_path->length; i++)
				{
					temp_x = sub_path->x[i];
					temp_y = sub_path->y[i];
					sub_path->x[i] = temp_x;
					sub_path->y[i] = temp_y;
				}
			}
			else if (n == 3) // s_a is in the lower left
			{
				for (i = 0; i < sub_path->length; i++)
				{
					temp_x = 1.0 - sub_path->y[i];
					temp_y = sub_path->x[i];
					sub_path->x[i] = temp_x;
					sub_path->y[i] = temp_y;
				}
			}

			// add this CellPath to the CellPath heap
			if (sub_path->length > 0)
			{
				if (primary)
				{
					if (_primary_cp_hash.find(sub_path) != _primary_cp_hash.end())
						_path_p_list_update(sub_path, sub_path->g);
					else
						_path_p_list_insert(sub_path, sub_path->g);
				}
				else
				{
					if (_secondary_cp_hash.find(sub_path) != _secondary_cp_hash.end())
						_path_s_list_update(sub_path, sub_path->g);
					else
						_path_s_list_insert(sub_path, sub_path->g);
				}
			}
			else
				delete sub_path;
		}

		// case 2: s is the next neighbor of s_b that is counter-clockwise of s_a
		s = s_b->ccknbr(s_a);

		// the input to computePointCostToEdge is in relative and not absolute terms,
		// we need to figure out which way x_hat and y_hat go
		if (n == 0) // s_a is in the upper left
		{
			x_hat = x;
			y_hat = 1.0 - y;
		}
		else if (n == 1) // s_a is in the upper right
		{
			x_hat = y;
			y_hat = x;
		}
		else if (n == 2) // s_a is in the lower right
		{
			x_hat = 1.0 - x;
			y_hat = y;
		}
		else if (n == 3) // s_a is in the lower left
		{
			x_hat = 1.0 - y;
			y_hat = 1.0 - x;
		}

		// get a new CellPath to put stuff in (there are 3 possibilities)
		for (j = 0; j < 3; j++)
			sub_paths[j] = new Map::CellPath(cy, cx);
		_compute_cost_of_point_to_edge(y_hat, x_hat, s, s_a, s_b, sub_paths);

		// the output from computePointCostToEdge is also relative, so we need to figure out which way best_y and best_x go
		for (j = 0; j < 3; j++)
		{
			sub_path = sub_paths[j];

			if (n == 0) // s_a is in the upper left
			{
				for (i = 0; i < sub_path->length; i++)
				{
					temp_x = sub_path->x[i];
					temp_y = 1.0 - sub_path->y[i];
					sub_path->x[i] = temp_x;
					sub_path->y[i] = temp_y;
				}
			}
			else if (n == 1) // s_a is in the upper right
			{
				for (i = 0; i < sub_path->length; i++)
				{
					temp_x = sub_path->y[i];
					temp_y = sub_path->x[i];
					sub_path->x[i] = temp_x;
					sub_path->y[i] = temp_y;
				}
			}
			else if (n == 2) // s_a is in the lower right
			{
				for (i = 0; i < sub_path->length; i++)
				{
					temp_x = 1.0 - sub_path->x[i];
					temp_y = sub_path->y[i];
					sub_path->x[i] = temp_x;
					sub_path->y[i] = temp_y;
				}
			}
			else if (n == 3) // s_a is in the lower left
			{
				for (i = 0; i < sub_path->length; i++)
				{
					temp_x = 1.0 - sub_path->y[i];
					temp_y = 1.0 - sub_path->x[i];
					sub_path->x[i] = temp_x;
					sub_path->y[i] = temp_y;
				}
			}

			// add this CellPath to the CellPath heap
			if (sub_path->length > 0)
			{
				if (primary)
				{
					if (_primary_cp_hash.find(sub_path) != _primary_cp_hash.end())
						_path_p_list_update(sub_path, sub_path->g);
					else
						_path_p_list_insert(sub_path, sub_path->g);
				}
				else
				{
					if (_secondary_cp_hash.find(sub_path) != _secondary_cp_hash.end())
						_path_s_list_update(sub_path, sub_path->g);
					else
						_path_s_list_insert(sub_path, sub_path->g);
				}
			}
			else
				delete sub_path;
		}
	}
	// printf("Finished computing a local point cost\n");
}

/**
 * Computes the local point cost for each possible cell and puts local paths into the heap
 *
 * @param double py
 * @param double px
 * @param bool true to use primary path list, false to use secondary
 */
void Planner::_compute_local_point_costs(double py, double px, bool primary)
{

	// printf("Computing local point costs\n");
	int lr, ud, pcx, pcy, ipx, ipy;
	double pbx, pby;

	ipx = static_cast<int>(px);
	ipy = static_cast<int>(py);

	if (Math::equals(px, static_cast<double>(ipx))) // then on a vertical edge, need to check the cells to the left and right of the robot
		lr = 1;
	else // only need to check the cells with x == int_pos_x
		lr = 0;

	while (lr != -1)
	{
		// figure out the cell and robot x coordinates for this particular cell
		if (lr == 1) // check the cells with x == int_pos_x - 1
		{
			pcx = ipx - 1;
			pbx = 1.0;
		}
		else // lr == 0, check the cells with x == int_pos_x
		{
			pcx = ipx;
			pbx = px - static_cast<double>(ipx);
		}

		if (pcx < 0 || pcx > _map->cols() - 1) // then outside of map
		{
			lr--;
			continue;
		}

		if (Math::equals(py, static_cast<double>(ipy))) // then on a horizontal edge, need to check the cells to the top and bottom of the robot
			ud = 1;
		else // only need to check the cells with y == int_pos_y
			ud = 0;

		while (ud != -1)
		{
			// figure out the cell and robot y coordinates for this particular cell
			if (ud == 1) // check the cells with y == int_pos_y - 1
			{
				pcy = ipy - 1;
				pby = 1.0;
			}
			else // ud == 0, check the cells with y == int_pos_y
			{
				pcy = ipy;
				pby = py - static_cast<double>(ipy);
			}

			if (pcy < 0 || pcy > _map->rows() - 1) // then outside of map
			{
				ud--;
				continue;
			}
			// the folowing function puts all possible transitions out of this cell into the CellPath heap
			_compute_local_point_cost(pcy, pcx, pby, pbx, primary);

			ud--;
		}
		lr--;
	}
	// printf("Finished computing local point costs\n");
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
			for (int i = 0; i < Map::Cell::NUM_NBRS; i++)
			{
				if (nbrs[i] != nullptr)
				{
					if (_cell_hash.find(nbrs[i]) == _cell_hash.end())
						_cell(nbrs[i]);
					rhs_old = _rhs(nbrs[i]);
					if (nbrs[i]->ccknbr(u) != nullptr)
					{
						cost_interpol = _compute_cost(nbrs[i], u, nbrs[i]->ccknbr(u));
						if (Math::greater(_rhs(nbrs[i]), cost_interpol))
						{
							_rhs(nbrs[i], cost_interpol);
							nbrs[i]->bptr(u);
							if (!Math::equals(cost_interpol, Math::INF))
								_h(nbrs[i], get<3>(_compute_bp(nbrs[i], u, nbrs[i]->ccknbr(u))));
							else
								_h(nbrs[i], Math::INF);
						}
					}
					if (nbrs[i]->cknbr(u) != nullptr)
					{
						if (Math::greater(_rhs(nbrs[i]), _compute_cost(nbrs[i], u, nbrs[i]->cknbr(u))))
						{
							_rhs(nbrs[i], _compute_cost(nbrs[i], nbrs[i]->cknbr(u), u));
							nbrs[i]->bptr(nbrs[i]->cknbr(u));
							if (!Math::equals(_rhs(nbrs[i]), Math::INF))
								_h(nbrs[i], get<3>(_compute_bp(nbrs[i], nbrs[i]->cknbr(u), u)));
							else
								_h(nbrs[i], Math::INF);
						}
					}
					if (!Math::equals(_rhs(u), rhs_old))
						_update_state(nbrs[i]);
				}
			}
		}
		else
		{
			argmin_min = _min_interpol_succ(u);
			_rhs(u, argmin_min.second);
			u->bptr(argmin_min.first);
			if (!Math::equals(argmin_min.second, Math::INF))
				_h(u, get<3>(_compute_bp(u, argmin_min.first, u->ccknbr(argmin_min.first))));
			else
				_h(u, Math::INF);
			if (Math::less(_g(u), _rhs(u)))
			{
				_g(u, Math::INF);
				nbrs = u->nbrs();
				for (int i = 0; i < Map::Cell::NUM_NBRS; i++)
				{
					if ((nbrs[i] != nullptr && nbrs[i]->bptr() != nullptr && nbrs[i]->ccknbr(nbrs[i]->bptr()) != nullptr) && nbrs[i] != _goal)
					{
						if (nbrs[i]->bptr() == u || nbrs[i]->bptr() == nbrs[i]->cknbr(u))
						{
							if (!Math::equals(_rhs(nbrs[i]), _compute_cost(nbrs[i], nbrs[i]->bptr(), nbrs[i]->ccknbr(nbrs[i]->bptr()))))
							{
								if (Math::less(_g(nbrs[i]), _rhs(nbrs[i])) || _open_hash.find(nbrs[i]) == _open_hash.end())
								{
									_rhs(nbrs[i], Math::INF);
									// _h(nbrs[i], Math::INF);
									_update_state(nbrs[i]);
								}
								else
								{
									argmin_min = _min_interpol_succ(nbrs[i]);
									_rhs(nbrs[i], argmin_min.second);
									nbrs[i]->bptr(argmin_min.first);
									if (!Math::equals(argmin_min.second, Math::INF))
										_h(u, get<3>(_compute_bp(nbrs[i], argmin_min.first, nbrs[i]->ccknbr(argmin_min.first))));
									else
										_h(u, Math::INF);
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
 * Extracts interpolated path using a combination of a backpointer gradient and a one step lookahead
 *
 * @return bool successful
 */
bool Planner::_extract_path()
{
	printf("Extracting path!\n");

	double total_path_cost, px, py, lapy, lapx, x_min, y_min, x_max, y_max, ax, ay, gx, gy;
	int ipx, ipy, iax, iay, attempts1, attempts2;
	tuple<double, double, int, double> endpoints;
	Map::CellPath *lookahead_cell_path;
	Map::CellPath *top_cell_path;
	Map::Cell *another_best;

	gx = static_cast<double>(_goal->x());
	gy = static_cast<double>(_goal->y());

	px = _current.first;
	py = _current.second;

	_interpol_path.push_back(make_pair(px, py));

	attempts1 = 0;

	while (!Math::equals(px, gx) || !Math::equals(py, gy))
	{
		// printf("Inside main path extraction loop\n");

		if (++attempts1 > Planner::MAX_EXTRACTION_STEPS)
		{
			printf("ERROR: REACHED MAXIMUM PATH EXTRACTION ATTEMPTS\n");
			return false;
		}

		Map::CellPath *best_cell_path;

		ax = px;
		ay = py;
		iax = static_cast<int>(ax);
		iay = static_cast<int>(ay);

		_compute_local_point_costs(py, px, true);

		attempts2 = 0;

		while (1)
		{
			if (++attempts2 > Planner::MAX_EXTRACTION_STEPS)
			{
				printf("ERROR: REACHED MAXIMUM NESTED PATH EXTRACTION ATTEMPTS\n");
				_secondary_cp_list.clear();
				_secondary_cp_hash.clear();
				_primary_cp_list.clear();
				_primary_cp_hash.clear();
				return false;
			}

			top_cell_path = _primary_cp_list.begin()->second;
			_path_p_list_remove(top_cell_path);

			if (top_cell_path == nullptr)
			{
				printf("ERROR: NO BEST NEIGHBOR FOUND\n");
				_secondary_cp_list.clear();
				_secondary_cp_hash.clear();
				_primary_cp_list.clear();
				_primary_cp_hash.clear();
				return false;
			}

			if (Math::equals(static_cast<double>(top_cell_path->get_cx()) + top_cell_path->x[0], px) && Math::equals(static_cast<double>(top_cell_path->get_cy()) + top_cell_path->y[0], py))
			{
				printf("ERROR: NEXT POINT IS THIS POINT\n");
				_secondary_cp_list.clear();
				_secondary_cp_hash.clear();
				_primary_cp_list.clear();
				_primary_cp_hash.clear();
				return false;
			}

			// now we want to do a 1 step look ahead to make sure that this point is still the best
			best_cell_path = top_cell_path;					  // this is the current best
			top_cell_path = _primary_cp_list.begin()->second; // this is the current second best

			lapy = static_cast<double>(best_cell_path->get_cy()) + best_cell_path->y[0];
			lapx = static_cast<double>(best_cell_path->get_cx()) + best_cell_path->x[0];

			if (Math::equals(lapy, py) && Math::equals(lapx, px))
			{
				printf("ERROR: THIS SHOULD NEVER HAPPEN 3\n");
				_secondary_cp_list.clear();
				_secondary_cp_hash.clear();
				_primary_cp_list.clear();
				_primary_cp_hash.clear();
				return false;
			}

			// check if the look ahead point is the goal, if so, then we know that this was the optimal path
			if (Math::equals(static_cast<double>(best_cell_path->get_cy()) + best_cell_path->y[0], gy) && Math::equals(static_cast<double>(best_cell_path->get_cx()) + best_cell_path->x[0], gx))
				break;

			_compute_local_point_costs(lapy, lapx, false);
			lookahead_cell_path = _secondary_cp_list.begin()->second;

			// // make sure that a look ahead point on a corner doesn't lead to a path where the next point is on the edge of the cell containing the robot (corners are ok)
			// if (Math::equals(lapy, round(lapy)) && Math::equals(lapx, round(lapx))) // look ahead point is on a corner
			// {

			// 	if (Math::equals(lookahead_cell_path->x[0], round(lookahead_cell_path->x[0])) && Math::equals(lookahead_cell_path->y[0], round(lookahead_cell_path->y[0]))) // next point is also on a corner
			// 	{
			// 		// corners are ok
			// 	}
			// 	else
			// 	{
			// 		if (Math::equals(static_cast<double>(ipy), py)) // then the robot is on a horizontal edge
			// 			y_min = py - 1.0;
			// 		else
			// 			y_min = static_cast<double>(ipy);
			// 		y_max = static_cast<double>(ipy + 1);

			// 		if (Math::equals(static_cast<double>(ipx), px)) // then the robot is on a vertical edge
			// 			x_min = px - 1.0;
			// 		else
			// 			x_min = static_cast<double>(ipx);
			// 		x_max = static_cast<double>(ipx + 1);

			// 		// if the first point after the look ahead point is in this range, then we don't want to use the look ahead point (ever)
			// 		if ((Math::less(x_min, lookahead_cell_path->x[0]) || Math::equals(x_min, lookahead_cell_path->x[0])) && (Math::less(lookahead_cell_path->x[0], x_max) || Math::equals(lookahead_cell_path->x[0], x_max)) &&
			// 			(Math::less(y_min, lookahead_cell_path->y[0]) || Math::equals(y_min, lookahead_cell_path->y[0])) && (Math::less(lookahead_cell_path->y[0], y_max) || Math::equals(lookahead_cell_path->y[0], y_max)))
			// 		{
			// 			_secondary_cp_list.clear();
			// 			_secondary_cp_hash.clear();
			// 			best_cell_path->~CellPath();
			// 			continue;
			// 		}
			// 	}
			// }

			if (Math::greater(best_cell_path->g_to_edge, top_cell_path->g) || Math::greater(lookahead_cell_path->g, top_cell_path->g) || Math::greater(best_cell_path->g_to_edge + lookahead_cell_path->g, top_cell_path->g))
			{
				// then the look ahead fails, so we'll put what we thought might be the best back into the heap after modifying its values
				best_cell_path->g = best_cell_path->g_to_edge + lookahead_cell_path->g;

				if ((_primary_cp_hash.find(best_cell_path) != _primary_cp_hash.end()))
					_path_p_list_update(best_cell_path, best_cell_path->g);
				else
					_path_p_list_insert(best_cell_path, best_cell_path->g);
			}
			else // it is still the best
			{
				_secondary_cp_list.clear();
				_secondary_cp_hash.clear();
				break;
			}

			_secondary_cp_list.clear();
			_secondary_cp_hash.clear();
		}

		px = static_cast<double>(best_cell_path->get_cx()) + best_cell_path->x[0];
		py = static_cast<double>(best_cell_path->get_cy()) + best_cell_path->y[0];

		total_path_cost += best_cell_path->g_to_edge;
		ipx = static_cast<int>(px);
		ipy = static_cast<int>(py);

		if ((px - static_cast<double>(ipx) < Math::SMALL) && (py - static_cast<double>(ipy) < Math::SMALL))
		{
			px = static_cast<double>(ipx);
			py = static_cast<double>(ipy);
		}
		else if ((static_cast<double>(ipx + 1) - px < Math::SMALL) && (py - static_cast<double>(ipy) < Math::SMALL))
		{
			ipx += 1;
			px = static_cast<double>(ipx);
			py = static_cast<double>(ipy);
		}
		else if ((static_cast<double>(ipx + 1) - px < Math::SMALL) && (static_cast<double>(ipy + 1) - py < Math::SMALL))
		{
			ipx += 1;
			ipy += 1;
			px = static_cast<double>(ipx);
			py = static_cast<double>(ipy);
		}
		else if ((px - static_cast<double>(ipx) < Math::SMALL) && (static_cast<double>(ipy + 1) - py < Math::SMALL))
		{
			ipy += 1;
			px = static_cast<double>(ipx);
			py = static_cast<double>(ipy);
		}

		if (_goal == (*_map)(ipy, ipx))
		{
		}
		// if on an edge and back pointers from nodes on either end of this edge converge
		// and end on a parallel edge to this edge, then use gradient method
		else if (Math::equals(ax, static_cast<double>(iax)) && !Math::equals(ay, static_cast<double>(iay)))
		{
			// on a vertical line but not a horizontal line

			double lx, ly, hx, hy;
			int low_path_type, high_path_type;

			lx = -1.0;
			ly = -1.0;
			hx = -1.0;
			hy = -1.0;

			low_path_type = -1;
			high_path_type = -1;

			// find back pointer from low end
			if (((*_map)(iay, iax))->bptr() != nullptr)
			{
				another_best = ((*_map)(iay, iax))->ccknbr(((*_map)(iay, iax))->bptr());
				if (another_best != nullptr)
				{
					endpoints = _compute_bp((*_map)(iay, iax), ((*_map)(iay, iax))->bptr(), another_best);

					ly = static_cast<double>(iay) + get<0>(endpoints);
					lx = static_cast<double>(iax) + get<1>(endpoints);
					low_path_type = get<2>(endpoints);
				}
			}

			// find back pointer from high end
			if (iay + 1 <= _map->rows() - 1)
			{
				if (((*_map)(iay + 1, iax))->bptr() != nullptr)
				{
					another_best = ((*_map)(iay + 1, iax))->ccknbr(((*_map)(iay + 1, iax))->bptr());
					if (another_best != nullptr)
					{
						endpoints = _compute_bp((*_map)(iay + 1, iax), ((*_map)(iay + 1, iax))->bptr(), another_best);

						hy = static_cast<double>(iay) + 1.0 + get<0>(endpoints);
						hx = static_cast<double>(iax) + get<1>(endpoints);
						high_path_type = get<2>(endpoints);
					}
				}
			}

			if (Math::equals(lx, hx) && Math::less(hy - ly, 1.0) && low_path_type != 2 && high_path_type != 2)
			{
				// both end's back pointers go to the same edge, they are converging, and only have one segment each

				// finding grid on other side of edge
				double temp_x;
				if (Math::greater(px, ax))
					temp_x = px;
				else // pos_x < at_x
					temp_x = px - 1.0;

				if (static_cast<int>(temp_x) >= _map->cols() - 1)
					temp_x = static_cast<double>(_map->cols() - 1);
				if (Math::less(temp_x, 0.0))
					temp_x = 0.0;

				if (Math::equals(px, static_cast<double>(ipx)) && Math::equals(py, static_cast<double>(ipy)) && Math::less(_cellcosts[((*_map)(static_cast<int>(min(py, ay)), static_cast<int>(min(px, ax))))->cost], _cellcosts[((*_map)(static_cast<int>(min(py, ay)), static_cast<int>(temp_x)))->cost]))
				{
					// because we lack look ahead, there could be a problem if the grid on the other side of
					// the edge (to which the back pointers end at) has a higher cost value than the one on
					// the current grid. This is only a concern if (due to look ahead) pose is current at a corner
					// so if that happens, we don't use the gradient stuff.
				}
				else
				{
					// find interpolation between high_y and low_y
					double new_pos_y = ly + ((ay - static_cast<double>(iay)) * (hy - ly));
					double new_pos_x = lx;

					// the gradient method is also worse than the linear interpolation method if the resulting angle between the
					// (gradient) back pointer and the closest axis is in between 23 and 28 degrees.
					// double delta_y = max(at_y - pos_y, pos_y - at_y);
					// double delta_x = max(at_x - pos_x, pos_x - at_x);
					double delta_y = max(ay - new_pos_y, new_pos_y - ay);
					double delta_x = max(ax - new_pos_x, new_pos_x - ax);
					double angle;
					if (Math::equals(delta_y, 0.0) || Math::equals(delta_x, 0.0))
						angle = 0.0;
					else if (Math::greater(delta_y, delta_x))
						angle = atan(delta_x / delta_y);
					else
						angle = atan(delta_y / delta_x);

					if (Math::less(GRAD_ANG_MIN, angle) && Math::less(angle, GRAD_ANG_MAX))
					{
						// in bad range
					}
					else
					{
						// reset pos_y based on interpolation between high_y and low_y
						py = new_pos_y;
						px = new_pos_x;

						// if this is in a grid that is up or down of where we started, we need to solve for the exit point of this grid
						if (static_cast<int>(py) != iay) // && high_y != low_y)
						{
							int crossed_y;

							// find the y value of the horizontal line that we crossed
							if (Math::greater(ay, py)) // then we crossed int_at_y
								crossed_y = static_cast<double>(iay);
							else // pos_y > at_y and we crossed (int)pos_y = int_at_y + 1
								crossed_y = static_cast<double>(iay) + 1.0;

							if (Math::greater(px, ax))
								px = ax + (crossed_y - ay) / (py - ay);
							else
								px = ax - (crossed_y - ay) / (py - ay);

							py = crossed_y;
						}

						// readjust totalPathCost
						total_path_cost -= best_cell_path->g_to_edge + _cellcosts[((*_map)(static_cast<int>(min(py, ay)), static_cast<int>(min(px, ax))))->cost] * sqrt((px - ax) * (px - ax) + (py - ay) * (py - ay));

						// printf("on vert edge, gradient \n");
					}
				}
			}
		}
		else if (!Math::equals(ax, static_cast<double>(iax)) && Math::equals(ay, static_cast<double>(iay)))
		{
			// on a horizontal line but not a vertical line
			double lx, ly, hx, hy;
			int low_path_type, high_path_type;

			lx = -1.0;
			ly = -1.0;
			hx = -1.0;
			hy = -1.0;

			low_path_type = -1;
			high_path_type = -1;

			// find back pointer from low end
			if (((*_map)(iay, iax))->bptr() != nullptr)
			{
				another_best = ((*_map)(iay, iax))->ccknbr(((*_map)(iay, iax))->bptr());
				if (another_best != nullptr)
				{
					endpoints = _compute_bp((*_map)(iay, iax), ((*_map)(iay, iax))->bptr(), another_best);

					ly = static_cast<double>(iay) + get<0>(endpoints);
					lx = static_cast<double>(iax) + get<1>(endpoints);
					low_path_type = get<2>(endpoints);
				}
			}

			// find back pointer from high end
			if (iax <= _map->cols() - 1)
			{
				if (((*_map)(iay, iax + 1))->bptr() != nullptr)
				{
					another_best = ((*_map)(iay, iax + 1))->ccknbr(((*_map)(iay, iax + 1))->bptr());
					if (another_best != nullptr)
					{
						endpoints = _compute_bp((*_map)(iay, iax + 1), ((*_map)(iay, iax + 1))->bptr(), another_best);

						hy = static_cast<double>(iay) + get<0>(endpoints);
						hx = static_cast<double>(iax) + 1.0 + get<1>(endpoints);
						high_path_type = get<2>(endpoints);
					}
				}
			}

			if (Math::equals(ly, hy) && Math::less(hx - lx, 1.0) && low_path_type != 2 && high_path_type != 2)
			{
				// both end's back pointers go to the same edge, they are converging, and only have one segment each

				// finding grid on other side of edge
				double temp_y;
				if (Math::greater(py, ay))
					temp_y = py;
				else // pos_y < at_y
					temp_y = py - 1.0;

				if (static_cast<int>(temp_y) >= _map->rows() - 1)
					temp_y = static_cast<double>(_map->rows()) - 1.0;
				if (Math::less(temp_y, 0.0))
					temp_y = 0.0;

				if (Math::equals(px, static_cast<double>(ipx)) && Math::equals(py, static_cast<double>(ipy)) && Math::less(_cellcosts[((*_map)(static_cast<int>(min(py, ay)), static_cast<int>(min(px, ax))))->cost], _cellcosts[((*_map)(static_cast<int>(temp_y), static_cast<int>(min(px, ax))))->cost]))
				{
					// because we lack look ahead, there could be a problem if the grid on the other side of
					// the edge (to which the back pointers end at) has a higher cost value than the one on
					// the current grid. This is only a concern if (due to look ahead) pose is current at a corner
					// so if that happens, we don't use the gradient stuff.
				}
				else
				{
					// find interpolation between high_y and low_y
					double new_pos_x = lx + ((ax - static_cast<double>(iax)) * (hx - lx));
					double new_pos_y = ly;

					// the gradient method is also worse than the linear interpolation method if the resulting angle between the
					// (gradient) back pointer and the closest axis is in between 23 and 28 degrees.
					// double delta_y = max(at_y - pos_y, pos_y - at_y);
					// double delta_x = max(at_x - pos_x, pos_x - at_x);
					double delta_y = max(ay - new_pos_y, new_pos_y - ay);
					double delta_x = max(ax - new_pos_x, new_pos_x - ax);
					double angle;
					if (Math::equals(delta_y, 0.0) || Math::equals(delta_x, 0.0))
						angle = 0.0;
					else if (Math::greater(delta_y, delta_x))
						angle = atan(delta_x / delta_y);
					else
						angle = atan(delta_y / delta_x);

					if (Math::less(GRAD_ANG_MIN, angle) && Math::less(angle, GRAD_ANG_MAX))
					{
						// in the bad range
					}
					else
					{
						// reset pos_x based on interpolation between high_x and low_x
						px = new_pos_x;
						py = new_pos_y;

						// if this is in a grid that is left or right of where we started, we need to solve for the exit point of this grid
						if (static_cast<int>(px) != iax) // && high_x != low_x)
						{
							int crossed_x;
							// printf("changed horiz grid \n");

							// find the x value of the vertical line that we crossed
							if (Math::greater(ax, px)) // then we crossed int_at_x
								crossed_x = static_cast<double>(iax);
							else // pos_x > at_x  we crossed (int)pos_x = int_at_x + 1
								crossed_x = static_cast<double>(iax) + 1.0;

							if (Math::greater(py, ay))
								py = ay + (crossed_x - ax) / (px - ax);
							else
								py = ay - (crossed_x - ax) / (px - ax);

							px = crossed_x;
						}

						// readjust totalPathCost
						total_path_cost -= best_cell_path->g_to_edge + _cellcosts[((*_map)(static_cast<int>(min(py, ay)), static_cast<int>(min(px, ax))))->cost] * sqrt((px - ax) * (px - ax) + (py - ay) * (py - ay));
						// printf("on horiz edge, gradient \n");
					}
				}
			}
		}
		_interpol_path.push_back(make_pair(px, py));

		_primary_cp_list.clear();
		_primary_cp_hash.clear();
		// best_cell_path->~CellPath();
	}

	// _remove_repeated_points(_interpol_path);
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
		g_rhs->first = value;

	return g_rhs->first;
}

/** Gets the path of backpointers that links sa to sb in the graph
 *
 * @param vector<pair<double, double>> &path
 * @param Map::Cell * first cell
 * @param Map::Cell * second cell
 * @return bool successful
 */
bool Planner::_get_path(vector<pair<double, double>> &path, Map::Cell *s_a, Map::Cell *s_b)
{
	if (s_a == nullptr || s_b == nullptr)
	{
		printf("ERROR: CELLS THAT ARE BEING REQUESTED TO FIND THE PATH FOR ARE NULLPTRS\n");
		return false;
	}

	Map::Cell *s_temp = s_a;
	// find length of path
	if (path.size() > 0)
		path.clear();

	while (s_temp != s_b)
	{

		if (s_temp == _goal)
			break;

		if (s_temp == nullptr || s_temp->bptr() == nullptr)
		{
			printf("ERROR: BACKPOINTER TRAIL FOUND A NULLPTR\n");
			return false;
		}
		else if (s_temp->bptr()->bptr() == s_temp && s_temp != _goal && s_temp->bptr() != _goal)
		{
			printf("ERROR: PROBLEM HERE\n");
			return false;
		}

		path.push_back(make_pair(static_cast<double>(s_temp->x()), static_cast<double>(s_temp->y())));
		s_temp = s_temp->bptr();
	}

	path.push_back(make_pair(static_cast<double>(s_temp->x()), static_cast<double>(s_temp->y())));
	return true;
}

/**
 * Gets/Sets h value for a cell.
 *
 * @param Map::Cell* cell to retrieve/update
 * @param double [optional] new h value
 * @return double h value
 */
double Planner::_h(Map::Cell *u, double value)
{
	_cell(u);

	if (value != DBL_MIN)
		_heuristic_hash[u] = value;

	return _heuristic_hash[u];
}

/**
 * Calculates heuristic between two cells (euclidean distance).
 *
 * @param Map::Cell* cell a
 * @param Map::Cell* cell b
 * @return double heuristic value
 */
double Planner::_heuristic(Map::Cell *a, Map::Cell *b)
{
	// return (0.0);
	// return (max(sqrt(pow(b->x() - a->x(), 2) + pow(b->y() - a->y(), 2)) - (2.0 * _Mc * Math::SQRT2), 0.0));
	// return (0.125 * sqrt(pow(b->x() - a->x(), 2) + pow(b->y() - a->y(), 2)));
	return (0.5 * sqrt(pow(b->x() - a->x(), 2) + pow(b->y() - a->y(), 2)));
	// return (15.0 * sqrt(pow(b->x() - a->x(), 2) + pow(b->y() - a->y(), 2)));
	// return (82.0 * sqrt(pow(b->x() - a->x(), 2) + pow(b->y() - a->y(), 2)));
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
	return pair<double, double>((min + _heuristic(_start, u)), min);
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
		pos2 = _open_list.end();
	else
		pos2++;

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

	for (int i = 0; i < Map::Cell::NUM_NBRS; i++)
	{
		if (nbrs[i] != nullptr && u->ccknbr(nbrs[i]) != nullptr)
		{
			tmp_cost = _compute_cost(u, nbrs[i], u->ccknbr(nbrs[i]));

			if (Math::less(tmp_cost, min_cost))
			{
				min_cell = nbrs[i];
				min_cost = tmp_cost;
			}
		}
	}

	return pair<Map::Cell *, double>(min_cell, min_cost);
}

/**
 * Inserts cell into the primary path list.
 *
 * @param Map::CellPath* cell path to insert
 * @param double key value for the cell path
 * @return void
 */
void Planner::_path_p_list_insert(Map::CellPath *p, double k)
{
	PL::iterator pos = _primary_cp_list.insert(PL_PAIR(k, p));
	_primary_cp_hash[p] = pos;
}

/**
 * Removes cell path from the primary path list.
 *
 * @param Map::CellPath* cell path to remove
 * @return void
 */
void Planner::_path_p_list_remove(Map::CellPath *p)
{
	_primary_cp_list.erase(_primary_cp_hash[p]);
	_primary_cp_hash.erase(_primary_cp_hash.find(p));
}

/**
 * Updates cell path in the primary path list.
 *
 * @param Map::CellPath* sub_path to update the key of
 * @param double k new key
 * @return void
 */
void Planner::_path_p_list_update(Map::CellPath *p, double k)
{
	PL::iterator pos1 = _primary_cp_hash[p];
	PL::iterator pos2 = pos1;

	if (pos1 == _primary_cp_list.end())
		pos2 = _primary_cp_list.end();
	else
		pos2++;

	_primary_cp_list.erase(pos1);
	_primary_cp_hash[p] = _primary_cp_list.insert(pos2, PL_PAIR(k, p));
}

/**
 * Inserts cell into the secondary path list.
 *
 * @param Map::CellPath* cell path to insert
 * @param double key value for the cell path
 * @return void
 */
void Planner::_path_s_list_insert(Map::CellPath *p, double k)
{
	PL::iterator pos = _secondary_cp_list.insert(PL_PAIR(k, p));
	_secondary_cp_hash[p] = pos;
}

/**
 * Updates cell path in the secondary path list.
 *
 * @param Map::CellPath* sub_path to update the key of
 * @param double k new key
 * @return void
 */
void Planner::_path_s_list_update(Map::CellPath *p, double k)
{
	PL::iterator pos1 = _secondary_cp_hash[p];
	PL::iterator pos2 = pos1;

	if (pos1 == _secondary_cp_list.end())
		pos2 = _secondary_cp_list.end();
	else
		pos2++;

	_secondary_cp_list.erase(pos1);
	_secondary_cp_hash[p] = _secondary_cp_list.insert(pos2, PL_PAIR(k, p));
}

/** Removes all but the first point in a sub-path containing repeated points
 *
 * @return vector<pair<double, double>> &path
 */
void Planner::_remove_repeated_points(vector<pair<double, double>> &path)
{
	vector<pair<double, double>> path_new;

	// walk through once to figure out the size of the new path
	int size_of_new = 1;
	int last_unique = 0;

	for (int i = 1; i < path.size(); i++)
	{
		// then i is the next unique point
		if (!Math::equals(path[last_unique].first, path[i].first) || !Math::equals(path[last_unique].second, path[i].second))
		{
			size_of_new = size_of_new + 1;
			last_unique = i;
		}
	}

	// set up new path
	path_new.resize(size_of_new);
	path_new[0] = make_pair(path[0].first, path[0].second);

	int j = 0;
	for (int i = 1; i < path.size(); i++)
	{
		// then i is the next unique point
		if (!Math::equals(path_new[j].first, path[i].first) || !Math::equals(path_new[j].second, path[i].second))
		{
			j++;
			path_new[j] = make_pair(path[i].first, path[i].second);
		}
	}

	// set equal to the new path
	path = path_new;
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
		return 0.0;

	_cell(u);
	pair<double, double> *g_rhs = &_cell_hash[u];

	if (value != DBL_MIN)
		g_rhs->second = value;

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
		_list_update(u, _k(u));
	else if (diff && !exists)
		_list_insert(u, _k(u));
	else if (!diff && exists)
		_list_remove(u);
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

/**
 * Path Key compare struct.
 */
bool Planner::PKeyCompare::operator()(const double &d1, const double &d2) const
{
	if (Math::less(d1, d2))
		return true;
	else if (Math::greater(d1, d2))
		return false;
	else
		return false;
}