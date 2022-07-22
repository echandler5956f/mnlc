#include "map.h"

using namespace std;
using namespace DStarLite;

/**
 * @var unsigned int number of cell neighbors
 */
const unsigned int Map::Cell::NUM_NBRS = 8;

/**
 * @var unsigned int number of cell corners
 */
const unsigned int Map::Cell::NUM_CNRS = 4;

/**
 * @var unsigned int number of distinct traversal costs
 */
const unsigned int Map::Cell::DIST_TRAV_COSTS = 70;

/**
 * @var double cost of an unwalkable tile
 */
const double Map::Cell::COST_UNWALKABLE = DBL_MAX;

/**
 * @var int hash "constant" (may need to change if map width exceeds this value)
 */
const int Map::Cell::Hash::C = 1000000;

/**
 * Constructor.
 *
 * @param unsigned int rows
 * @param unsigned int columns
 */
Map::Map(unsigned int rows, unsigned int cols)
{
	_rows = rows;
	_cols = cols;

	_cells = new Cell **[rows];

	for (unsigned int i = 0; i < rows; i++)
	{
		_cells[i] = new Cell *[cols];

		for (unsigned int j = 0; j < cols; j++)
		{
			// Initialize cells
			_cells[i][j] = new Cell(j, i);
		}
	}

	// Attach neighbors
	for (unsigned int i = 0; i < rows; i++)
	{
		for (unsigned int j = 0; j < cols; j++)
		{
			Cell **nbrs = new Cell *[Cell::NUM_NBRS];
			for (unsigned int k = 0; k < Cell::NUM_NBRS; k++)
			{
				nbrs[k] = nullptr;
			}

			Cell **cnrs = new Cell *[Cell::NUM_CNRS];
			for (unsigned int k = 0; k < Cell::NUM_CNRS; k++)
			{
				cnrs[k] = nullptr;
			}

			// Top
			if (i != 0)
			{
				if (j != 0)
				{
					// Top left
					nbrs[0] = _cells[i - 1][j - 1];
				}

				// Top middle
				nbrs[1] = _cells[i - 1][j];

				if (j < cols - 1)
				{
					// Top right
					nbrs[2] = _cells[i - 1][j + 1];
					cnrs[1] = _cells[i - 1][j + 1];
				}
			}

			if (j < cols - 1)
			{
				// Middle right
				nbrs[3] = _cells[i][j + 1];
				cnrs[0] = _cells[i][j + 1];
			}

			// Bottom
			if (i < rows - 1)
			{
				if (j < cols - 1)
				{
					// Bottom right
					nbrs[4] = _cells[i + 1][j + 1];
					cnrs[1] = _cells[i + 1][j + 1];
				}

				// Bottom middle
				nbrs[5] = _cells[i + 1][j];
				cnrs[2] = _cells[i + 1][j];

				if (j != 0)
				{
					// Bottom left
					nbrs[6] = _cells[i + 1][j - 1];
				}
			}

			if (j != 0)
			{
				// Middle left
				nbrs[7] = _cells[i][j - 1];
			}

			cnrs[3] = _cells[i][j];
			_cells[i][j]->init(nbrs, cnrs);
		}
	}
}

/**
 * Deconstructor.
 */
Map::~Map()
{
	for (unsigned int i = 0; i < _rows; i++)
	{
		for (unsigned int j = 0; j < _cols; j++)
		{
			delete _cells[i][j];
		}

		delete[] _cells[i];
	}

	delete[] _cells;
}

/**
 * Retrieves a cell.
 *
 * @param unsigned int row
 * @param unsigned int column
 * @return Map::Cell*
 */
Map::Cell *Map::operator()(const unsigned int row, const unsigned int col)
{
	return _cells[row][col];
}

/**
 * Gets number of cols.
 *
 * @return unsigned int
 */
unsigned int Map::cols()
{
	return _cols;
}

/**
 * Checks if row/col exists.
 *
 * @param unsigned int row
 * @param unsigned int col
 * @return bool
 */
bool Map::has(unsigned int row, unsigned int col)
{
	return (row >= 0 && row < _rows && col >= 0 && col < _cols);
}

/**
 * Gets number of rows.
 *
 * @return unsigned int
 */
unsigned int Map::rows()
{
	return _rows;
}

/**
 * Constructor.
 *
 * @param unsigned int x-coordinate
 * @param unsigned int y-coordinate
 * @param double [optional] cost of the cell
 */
Map::Cell::Cell(unsigned int x, unsigned int y, double cost)
{
	_init = false;

	_bptr = nullptr;

	_nbrs = nullptr;
	_cnrs = nullptr;

	_x = x;
	_y = y;

	this->cost = cost;
}

/**
 * Deconstructor.
 */
Map::Cell::~Cell()
{
	if (_nbrs != nullptr)
		delete[] _nbrs;
	if (_cnrs != nullptr)
		delete[] _cnrs;
}

/**
 * Initialize.
 *
 * @param Cell** cell neighbors
 * @param Cell** cell corners
 * @return void
 */
void Map::Cell::init(Cell **nbrs, Cell **cnrs)
{
	if (_init)
		return;

	_init = true;

	_nbrs = nbrs;
	_cnrs = cnrs;
}

/**
 * Check if cnr is a corner of ->this
 *
 * @param Cell *cnr
 * @return true or false
 */
bool Map::Cell::is_corner(Cell *cnr)
{
	if (cnr == nullptr)
	{
		// printf("CORNER CELL WAS INVALID!\n");
		return false;
	}

	return ((cnr->x() == _cnrs[0]->x() && cnr->y() == _cnrs[0]->y()) || (cnr->x() == _cnrs[1]->x() && cnr->y() == _cnrs[1]->y()) || (cnr->x() == _cnrs[2]->x() && cnr->y() == _cnrs[2]->y()) || (cnr->x() == _cnrs[3]->x() && cnr->y() == _cnrs[3]->y()));
}

/**
 * Gets clockwise neighbor relative to some neighbor at neighbor s1.
 *
 * @param Cell *
 * @return Cell*
 */
Map::Cell *Map::Cell::cknbr(Cell *s1)
{
	if (s1 == nullptr)
	{
		// printf("Invalid call to cknbr1!\n");
		return nullptr;
	}
	int dx = s1->_x - _x;
	int dy = -(s1->_y - _y);
	if (dx == 1 && dy == 0)
	{
		// printf("(sx: %u, sy: %u), (s1x: %u, s1y: %u), (s->cknbr2(s1)x: %u, s->cknbr2(s1)y: %u)\n", _x, _y, s1->x(), s1->y(), _nbrs[2]->x(), _nbrs[2]->y());
		return _nbrs[2];
	}
	else if (dx == 1 && dy == -1)
	{
		// printf("(sx: %u, sy: %u), (s1x: %u, s1y: %u), (s->cknbr3(s1)x: %u, s->cknbr3(s1)y: %u)\n", _x, _y, s1->x(), s1->y(), _nbrs[3]->x(), _nbrs[3]->y());

		return _nbrs[3];
	}
	else if (dx == 0 && dy == -1)
	{
		// printf("(sx: %u, sy: %u), (s1x: %u, s1y: %u), (s->cknbr4(s1)x: %u, s->cknbr4(s1)y: %u)\n", _x, _y, s1->x(), s1->y(), _nbrs[4]->x(), _nbrs[4]->y());

		return _nbrs[4];
	}
	else if (dx == -1 && dy == -1)
	{
		// printf("(sx: %u, sy: %u), (s1x: %u, s1y: %u), (s->cknbr5(s1)x: %u, s->cknbr5(s1)y: %u)\n", _x, _y, s1->x(), s1->y(), _nbrs[5]->x(), _nbrs[5]->y());

		return _nbrs[5];
	}
	else if (dx == -1 && dy == 0)
	{
		// printf("(sx: %u, sy: %u), (s1x: %u, s1y: %u), (s->cknbr6(s1)x: %u, s->cknbr6(s1)y: %u)\n", _x, _y, s1->x(), s1->y(), _nbrs[6]->x(), _nbrs[6]->y());

		return _nbrs[6];
	}
	else if (dx == -1 && dy == 1)
	{
		// printf("(sx: %u, sy: %u), (s1x: %u, s1y: %u), (s->cknbr7(s1)x: %u, s->cknbr7(s1)y: %u)\n", _x, _y, s1->x(), s1->y(), _nbrs[7]->x(), _nbrs[7]->y());

		return _nbrs[7];
	}
	else if (dx == 0 && dy == 1)
	{
		// printf("(sx: %u, sy: %u), (s1x: %u, s1y: %u), (s->cknbr0(s1)x: %u, s->cknbr0(s1)y: %u)\n", _x, _y, s1->x(), s1->y(), _nbrs[0]->x(), _nbrs[0]->y());

		return _nbrs[0];
	}
	else if (dx == 1 && dy == 1)
	{
		// printf("(sx: %u, sy: %u), (s1x: %u, s1y: %u), (s->cknbr1(s1)x: %u, s->cknbr1(s1)y: %u)\n", _x, _y, s1->x(), s1->y(), _nbrs[1]->x(), _nbrs[1]->y());

		return _nbrs[1];
	}
	else
	{
		// printf("Invalid call to cknbr2!\n");
		return nullptr;
	}
}

/**
 * Gets counterclockwise neighbor relative to some neighbor at neighbor s1.
 *
 * @param Cell * s1
 * @return Cell*
 */
Map::Cell *Map::Cell::ccknbr(Cell *s1)
{
	if (s1 == nullptr)
	{
		// printf("Invalid call to ccknbr1!\n");
		return nullptr;
	}
	int dx = s1->_x - _x;
	int dy = -(s1->_y - _y);
	if (dx == 1 && dy == 0)
	{
		// printf("(sx: %u, sy: %u), (s1x: %u, s1y: %u), (s->ccknbr4(s1)x: %u, s->ccknbr4(s1)y: %u)\n", _x, _y, s1->x(), s1->y(), _nbrs[4]->x(), _nbrs[4]->y());
		return _nbrs[4];
	}
	else if (dx == 1 && dy == -1)
	{
		// printf("(sx: %u, sy: %u), (s1x: %u, s1y: %u), (s->ccknbr5(s1)x: %u, s->ccknbr5(s1)y: %u)\n", _x, _y, s1->x(), s1->y(), _nbrs[5]->x(), _nbrs[5]->y());

		return _nbrs[5];
	}
	else if (dx == 0 && dy == -1)
	{
		// printf("(sx: %u, sy: %u), (s1x: %u, s1y: %u), (s->ccknbr6(s1)x: %u, s->ccknbr6(s1)y: %u)\n", _x, _y, s1->x(), s1->y(), _nbrs[6]->x(), _nbrs[6]->y());

		return _nbrs[6];
	}
	else if (dx == -1 && dy == -1)
	{
		// printf("(sx: %u, sy: %u), (s1x: %u, s1y: %u), (s->ccknbr7(s1)x: %u, s->ccknbr7(s1)y: %u)\n", _x, _y, s1->x(), s1->y(), _nbrs[7]->x(), _nbrs[7]->y());

		return _nbrs[7];
	}
	else if (dx == -1 && dy == 0)
	{
		// printf("(sx: %u, sy: %u), (s1x: %u, s1y: %u), (s->ccknbr0(s1)x: %u, s->ccknbr0(s1)y: %u)\n", _x, _y, s1->x(), s1->y(), _nbrs[0]->x(), _nbrs[0]->y());

		return _nbrs[0];
	}
	else if (dx == -1 && dy == 1)
	{
		// printf("(sx: %u, sy: %u), (s1x: %u, s1y: %u), (s->ccknbr1(s1)x: %u, s->ccknbr1(s1)y: %u)\n", _x, _y, s1->x(), s1->y(), _nbrs[1]->x(), _nbrs[1]->y());

		return _nbrs[1];
	}
	else if (dx == 0 && dy == 1)
	{
		// printf("(sx: %u, sy: %u), (s1x: %u, s1y: %u), (s->ccknbr2(s1)x: %u, s->ccknbr2(s1)y: %u)\n", _x, _y, s1->x(), s1->y(), _nbrs[2]->x(), _nbrs[2]->y());

		return _nbrs[2];
	}
	else if (dx == 1 && dy == 1)
	{
		// printf("(sx: %u, sy: %u), (s1x: %u, s1y: %u), (s->ccknbr3(s1)x: %u, s->ccknbr3(s1)y: %u)\n", _x, _y, s1->x(), s1->y(), _nbrs[3]->x(), _nbrs[3]->y());

		return _nbrs[3];
	}
	else
	{
		// printf("Invalid call to ccknbr2!\n");
		return nullptr;
	}
}

/**
 * Gets/sets cell backpointer from which ->this cell derives its path cost.
 *
 * Cell* [optional] backpointer
 * @return Cell*
 */
Map::Cell *Map::Cell::bptr(Cell *backpointer)
{
	if (backpointer != nullptr)
	{
		_bptr = backpointer;
	}
	return _bptr;
}

/**
 * Gets cell neighbors.
 *
 * @return Cell**
 */
Map::Cell **Map::Cell::nbrs()
{
	return _nbrs;
}

/**
 * Gets cell corners.
 *
 * @return Cell**
 */
Map::Cell **Map::Cell::cnrs()
{
	return _cnrs;
}

/**
 * Gets x-coordinate.
 *
 * @return unsigned int
 */
unsigned int Map::Cell::x()
{
	return _x;
}

/**
 * Gets y-coordinate.
 *
 * @return unsigned int
 */
unsigned int Map::Cell::y()
{
	return _y;
}

/**
 * Hashes cell based on coordinates.
 *
 * @param Cell*
 * @return size_t
 */
size_t Map::Cell::Hash::operator()(Cell *c) const
{
	return Cell::Hash::C * c->y() + c->x();
}