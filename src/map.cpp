#include "map.h"

using namespace std;
using namespace DStarLite;

/**
 * @var int number of cell neighbors
 */
const int Map::Cell::NUM_NBRS = 8;

/**
 * @var int number of cell corners
 */
const int Map::Cell::NUM_CNRS = 4;

/**
 * @var double cost of an unwalkable tile
 */
const double Map::Cell::COST_UNWALKABLE = DBL_MAX;

/**
 * @var static const int[] used to find nodes on the corners of a grid cell
 */
const int Map::Cell::DELTAY[] = {0, 0, 1, 1};

/**
 * @var static const int[] used to find nodes on the corners of a grid cell
 */
const int Map::Cell::DELTAX[] = {0, 1, 1, 0};

/**
 * @var int hash "constant" (may need to change if map width exceeds this value)
 */
const int Map::Cell::Hash::C = 1000000;

/**
 * @var int hash "constant" (may need to change if map width exceeds this value)
 */
const int Map::CellPath::Hash::C = 1000000;

/**
 * Constructor.
 *
 * @param int rows
 * @param int columns
 */
Map::Map(int rows, int cols)
{
	_rows = rows;
	_cols = cols;

	_cells = new Cell **[rows];

	for (int i = 0; i < rows; i++)
	{
		_cells[i] = new Cell *[cols];

		for (int j = 0; j < cols; j++)
			// Initialize cells
			_cells[i][j] = new Cell(j, i);
	}

	// Attach neighbors
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			Cell **nbrs = new Cell *[Cell::NUM_NBRS];
			for (int k = 0; k < Cell::NUM_NBRS; k++)
				nbrs[k] = nullptr;

			Cell **cnrs = new Cell *[Cell::NUM_CNRS];
			for (int k = 0; k < Cell::NUM_CNRS; k++)
				cnrs[k] = nullptr;

			Cell **cnr_of = new Cell *[Cell::NUM_CNRS];
			for (int k = 0; k < Cell::NUM_CNRS; k++)
				cnr_of[k] = nullptr;

			// Top
			if (i != 0)
			{
				if (j != 0)
					// Top left
					nbrs[3] = _cells[i - 1][j - 1];

				// Top middle
				nbrs[2] = _cells[i - 1][j];
				cnrs[2] = _cells[i - 1][j];

				if (j < cols - 1)
				{
					// Top right
					nbrs[1] = _cells[i - 1][j + 1];
					cnrs[1] = _cells[i - 1][j + 1];
				}
			}

			if (j < cols - 1)
			{
				// Middle right
				nbrs[0] = _cells[i][j + 1];
				cnrs[0] = _cells[i][j + 1];
			}

			// Bottom
			if (i < rows - 1)
			{
				if (j < cols - 1)
					// Bottom right
					nbrs[7] = _cells[i + 1][j + 1];

				// Bottom middle
				nbrs[6] = _cells[i + 1][j];
				cnr_of[0] = _cells[i + 1][j];

				if (j != 0)
				{
					// Bottom left
					nbrs[5] = _cells[i + 1][j - 1];
					cnr_of[1] = _cells[i + 1][j - 1];
				}
			}

			if (j != 0)
			{
				// Middle left
				nbrs[4] = _cells[i][j - 1];
				cnr_of[2] = _cells[i][j - 1];
			}

			cnrs[3] = _cells[i][j];
			cnr_of[3] = _cells[i][j];
			_cells[i][j]->init(nbrs, cnrs, cnr_of);
		}
	}
}

/**
 * Deconstructor.
 */
Map::~Map()
{
	for (int i = 0; i < _rows; i++)
	{
		for (int j = 0; j < _cols; j++)
			delete _cells[i][j];

		delete[] _cells[i];
	}

	delete[] _cells;
}

/**
 * Retrieves a cell.
 *
 * @param int row
 * @param int column
 * @return Map::Cell*
 */
Map::Cell *Map::operator()(const int row, const int col)
{
	return _cells[row][col];
}

/**
 * Gets number of cols.
 *
 * @return int
 */
int Map::cols()
{
	return _cols;
}

/**
 * Checks if row/col exists.
 *
 * @param int row
 * @param int col
 * @return bool
 */
bool Map::has(int row, int col)
{
	return (row >= 0 && row < _rows && col >= 0 && col < _cols);
}

/**
 * Gets number of rows.
 *
 * @return int
 */
int Map::rows()
{
	return _rows;
}

/**
 * Constructor.
 *
 * @param Cell * cell the local path resides within
 */
Map::CellPath::CellPath(int cell_x, int cell_y)
{
	g_to_edge = Math::INF;
	local_g = Math::INF;
	_cx = cell_x;
	_cy = cell_y;
	length = 0;
	g = Math::INF;
}

/**
 * Deconstructor.
 */
Map::CellPath::~CellPath()
{
	if (length > 0)
	{
		delete []x;
		delete []y;
	}
}

/**
 * Get cell x value.
 *
 */
int Map::CellPath::get_cx()
{
	return _cx;
}

/**
 * Get cell y value.
 *
 */
int Map::CellPath::get_cy()
{
	return _cy;
}

/**
 * Hashes cell based on coordinates.
 *
 * @param CellPath*
 * @return size_t
 */
size_t Map::CellPath::Hash::operator()(CellPath *c) const
{
	return CellPath::Hash::C * c->_cy + c->_cx;
}

/**
 * Constructor.
 *
 * @param int x-coordinate
 * @param int y-coordinate
 * @param int [optional] cost of the cell
 */
Map::Cell::Cell(int x, int y, int cost)
{
	_init = false;

	_bptr = nullptr;

	_nbrs = nullptr;
	_cnrs = nullptr;
	_cnr_of = nullptr;

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
	if (_cnr_of != nullptr)
		delete[] _cnr_of;
}

/**
 * Initialize.
 *
 * @param Cell** cell neighbors
 * @param Cell** cell corners
 * @param Cell** cell corners of
 * @return void
 */
void Map::Cell::init(Cell **nbrs, Cell **cnrs, Cell **cnr_of)
{
	if (_init)
		return;

	_init = true;

	_nbrs = nbrs;
	_cnrs = cnrs;
	_cnr_of = cnr_of;
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
		return false;

	return ((_cnrs[0] != nullptr && cnr->x() == _cnrs[0]->x() && cnr->y() == _cnrs[0]->y()) || (_cnrs[1] != nullptr && cnr->x() == _cnrs[1]->x() && cnr->y() == _cnrs[1]->y()) || (_cnrs[2] != nullptr && cnr->x() == _cnrs[2]->x() && cnr->y() == _cnrs[2]->y()) || (_cnrs[3] != nullptr && cnr->x() == _cnrs[3]->x() && cnr->y() == _cnrs[3]->y()));
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
		return nullptr;

	if (_nbrs[0] != nullptr && s1->x() == _nbrs[0]->x() && s1->y() == _nbrs[0]->y())
		return _nbrs[7];
	else if (_nbrs[1] != nullptr && s1->x() == _nbrs[1]->x() && s1->y() == _nbrs[1]->y())
		return _nbrs[0];
	else if (_nbrs[2] != nullptr && s1->x() == _nbrs[2]->x() && s1->y() == _nbrs[2]->y())
		return _nbrs[1];
	else if (_nbrs[3] != nullptr && s1->x() == _nbrs[3]->x() && s1->y() == _nbrs[3]->y())
		return _nbrs[2];
	else if (_nbrs[4] != nullptr && s1->x() == _nbrs[4]->x() && s1->y() == _nbrs[4]->y())
		return _nbrs[3];
	else if (_nbrs[5] != nullptr && s1->x() == _nbrs[5]->x() && s1->y() == _nbrs[5]->y())
		return _nbrs[4];
	else if (_nbrs[6] != nullptr && s1->x() == _nbrs[6]->x() && s1->y() == _nbrs[6]->y())
		return _nbrs[5];
	else if (_nbrs[7] != nullptr && s1->x() == _nbrs[7]->x() && s1->y() == _nbrs[7]->y())
		return _nbrs[6];
	else
		return nullptr;
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
		return nullptr;

	if (_nbrs[0] != nullptr && s1->x() == _nbrs[0]->x() && s1->y() == _nbrs[0]->y())
		return _nbrs[1];
	else if (_nbrs[1] != nullptr && s1->x() == _nbrs[1]->x() && s1->y() == _nbrs[1]->y())
		return _nbrs[2];
	else if (_nbrs[2] != nullptr && s1->x() == _nbrs[2]->x() && s1->y() == _nbrs[2]->y())
		return _nbrs[3];
	else if (_nbrs[3] != nullptr && s1->x() == _nbrs[3]->x() && s1->y() == _nbrs[3]->y())
		return _nbrs[4];
	else if (_nbrs[4] != nullptr && s1->x() == _nbrs[4]->x() && s1->y() == _nbrs[4]->y())
		return _nbrs[5];
	else if (_nbrs[5] != nullptr && s1->x() == _nbrs[5]->x() && s1->y() == _nbrs[5]->y())
		return _nbrs[6];
	else if (_nbrs[6] != nullptr && s1->x() == _nbrs[6]->x() && s1->y() == _nbrs[6]->y())
		return _nbrs[7];
	else if (_nbrs[7] != nullptr && s1->x() == _nbrs[7]->x() && s1->y() == _nbrs[7]->y())
		return _nbrs[0];
	else
		return nullptr;
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
		_bptr = backpointer;
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
 * Gets corners of ->this cell.
 *
 * @return Cell**
 */
Map::Cell **Map::Cell::cnrs()
{
	return _cnrs;
}

/**
 * Get the cells that ->this cell is a corner of
 *
 * @return Cell**
 */
Map::Cell **Map::Cell::cnr_of()
{
	return _cnr_of;
}

/**
 * Gets x-coordinate.
 *
 * @return int
 */
int Map::Cell::x()
{
	return _x;
}

/**
 * Gets y-coordinate.
 *
 * @return int
 */
int Map::Cell::y()
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