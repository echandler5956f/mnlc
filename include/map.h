#ifndef DSTARLITE_MAP_H
#define DSTARLITE_MAP_H

#include <functional>
#include <stdlib.h>
#include <vector>

#include "math.h"

using namespace std;

namespace DStarLite
{
	// map of cells
	class Map
	{
	public:
		// cells used to represent our grid
		class Cell
		{
		public:
			/**
			 * Hash
			 */
			class Hash : public unary_function<Cell *, size_t>
			{
			public:
				/**
				 * @var int hash "constant" (may need to change if width exceeds this value)
				 */
				static const int C;

				/**
				 * Hashes cell based on coordinates.
				 *
				 * @param Cell*
				 * @return size_t
				 */
				size_t operator()(Cell *c) const;
			};

			/**
			 * @var static const int number of cell neighbors
			 */
			static const int NUM_NBRS;

			/**
			 * @var static const int number of cell corners
			 */
			static const int NUM_CNRS;

			/**
			 * @var static const double cost of an unwalkable cell
			 */
			static const double COST_UNWALKABLE;

			/**
			 * @var static const int[] used to find nodes on the corners of a grid cell
			 */
			static const int DELTAY[];

			/**
			 * @var static const int[] used to find nodes on the corners of a grid cell
			 */
			static const int DELTAX[];

			/**
			 * @var double cost of cell
			 */
			int cost;

			/**
			 * @var bool ready to update status of the cell (occurs if the cell cost changes from known to unknown or vice versa)
			 */
			bool unknown_flip;

			/**
			 * Constructor.
			 *
			 * @param int x-coordinate
			 * @param int y-coordinate
			 * @param int [optional] cost of the cell
			 */
			Cell(int x, int y, int cost = 0);

			/**
			 * Deconstructor.
			 */
			virtual ~Cell();

			/**
			 * Initialize.
			 *
			 * @param Cell** cell neighbors
			 * @param Cell** cell corners
			 * @return void
			 */
			void init(Cell **nbrs, Cell **cnrs);

			/**
			 * Check if cnr is a corner of ->this cell
			 *
			 * @param Cell *cnr
			 * @return true or false
			 */
			bool is_corner(Cell *cnr);

			/**
			 * Gets clockwise neighbor relative to some neighbor at neighbor s1.
			 *
			 * @param Cell *
			 * @return Cell*
			 */
			Cell *cknbr(Cell *s1);

			/**
			 * Gets counterclockwise neighbor relative to some neighbor at neighbor s1.
			 *
			 * @param Cell * s1
			 * @return Cell*
			 */
			Cell *ccknbr(Cell *s1);

			/**
			 * Gets/sets cell backpointer from which ->this cell derives its path cost.
			 *
			 * Cell* [optional] backpointer
			 * @return Cell*
			 */
			Cell *bptr(Cell *backpointer = nullptr);

			/**
			 * Gets cell neighbors.
			 *
			 * @return Cell**
			 */
			Cell **nbrs();

			/**
			 * Gets corners of ->this cell.
			 *
			 * @return Cell**
			 */
			Cell **cnrs();

			/**
			 * Get canonical status of the cell.
			 *
			 * @return bool
			 */
			bool is_canon();

			/**
			 * Gets x-coordinate.
			 *
			 * @return int
			 */
			int x();

			/**
			 * Gets y-coordinate.
			 *
			 * @return int
			 */
			int y();

		protected:
			/**
			 * @var bool initialized
			 */
			bool _init;

			/**
			 * @var Cell* points to the cell from which ->this cell derives its path cost
			 */
			Cell *_bptr;

			/**
			 * @var Cell** neighbors
			 */
			Cell **_nbrs;

			/**
			 * @var Cell** corners
			 */
			Cell **_cnrs;

			/**
			 * @var int x-coordinate
			 */
			int _x;

			/**
			 * @var int y-coordinate
			 */
			int _y;

			/**
			 * @var bool is a canonical cell
			 */
			bool _is_canon;
		};

		// class CanonicalCell : public Cell
		// {
		// public:
		// 	/**
		// 	 * Constructor.
		// 	 *
		// 	 * @param int x-coordinate
		// 	 * @param int y-coordinate
		// 	 * @param int [optional] cost of the cell
		// 	 */
		// 	CanonicalCell(int x, int y, int cost = 0);

		// 	/**
		// 	 * Deconstructor.
		// 	 */
		// 	virtual ~CanonicalCell();

		// 	/**
		// 	 * Initialize.
		// 	 *
		// 	 * @param Cell** canonical cell neighbors
		// 	 * @param Cell** canonical cell corners
		// 	 * @return void
		// 	 */
		// 	void init(Cell **nbrs, Cell **cnrs);

		// 	/**
		// 	 * Gets clockwise neighbor relative to some neighbor at neighbor s1.
		// 	 *
		// 	 * @param CanonicalCell *
		// 	 * @return CanonicalCell*
		// 	 */
		// 	CanonicalCell *canon_cknbr(CanonicalCell *s1);

		// 	/**
		// 	 * Gets counterclockwise neighbor relative to some neighbor at neighbor s1.
		// 	 *
		// 	 * @param CanonicalCell * s1
		// 	 * @return CanonicalCell*
		// 	 */
		// 	CanonicalCell *canon_ccknbr(CanonicalCell *s1);

		// 	/**
		// 	 * Gets/sets cell backpointer from which ->this cell derives its path cost.
		// 	 *
		// 	 * CanonicalCell* [optional] backpointer
		// 	 * @return CanonicalCell*
		// 	 */
		// 	CanonicalCell *canon_bptr(CanonicalCell *canon_backpointer = nullptr);

		// 	/**
		// 	 * Gets cell neighbors.
		// 	 *
		// 	 * @return CanonicalCell**
		// 	 */
		// 	CanonicalCell **canon_nbrs();

		// protected:
		// 	/**
		// 	 * @var CanonicalCell* points to the canonical cell from which ->this cell derives its path cost
		// 	 */
		// 	CanonicalCell *_canon_bptr;

		// 	/**
		// 	 * @var CanonicalCell** neighbors
		// 	 */
		// 	CanonicalCell **_canon_nbrs;
		// };

		// local path within a cell
		class CellPath
		{
		public:
			/**
			 * Hash
			 */
			class Hash : public unary_function<CellPath *, size_t>
			{
			public:
				/**
				 * @var int hash "constant" (may need to change if width exceeds this value)
				 */
				static const int C;

				/**
				 * Hashes cell path based on coordinates and g value.
				 *
				 * @param CellPath*
				 * @return size_t
				 */
				size_t operator()(CellPath *c) const;
			};
			/**
			 * Constructor.
			 *
			 * @param int cell y value
			 * @param int cell x value
			 */
			CellPath(int cell_y, int cell_x);

			/**
			 * Deconstructor.
			 */
			~CellPath();

			/**
			 * Get cell x value.
			 *
			 */
			int get_cx();

			/**
			 * Get cell y value.
			 *
			 */
			int get_cy();

			/**
			 *
			 * @var double x the x values that make up the local path
			 */
			double x[3];

			/**
			 *
			 * @var double y the y values that make up the local path
			 */
			double y[3];

			/**
			 *
			 * @var double g the global g value with higher accuracy found via the local path
			 */
			double g;

			/**
			 *
			 * @var double local_g the local component of the overall g value
			 */
			double local_g;

			/**
			 *
			 * @var double g_to_edge the path-to-edge component of the local component of the g value
			 */
			double g_to_edge;

			/**
			 *
			 * @var int length convenience variable to keep track of how many of the three points we are using
			 */
			int length;

		protected:
			/**
			 *
			 * @var int cx the x value of the cell
			 */
			int _cx;

			/**
			 *
			 * @var int cy the y value of the cell
			 */
			int _cy;
		};

		/**
		 * Constructor.
		 *
		 * @param int rows
		 * @param int columns
		 */
		Map(int rows, int cols);

		/**
		 * Deconstructor.
		 */
		~Map();

		/**
		 * Retrieves a cell.
		 *
		 * @param int row
		 * @param int column
		 * @return Map::Cell*
		 */
		Cell *operator()(const int row, const int col);

		/**
		 * Gets number of cols.
		 *
		 * @return int
		 */
		int cols();

		/**
		 * Checks if row/col exists.
		 *
		 * @param int row
		 * @param int col
		 * @return bool
		 */
		bool has(int row, int col);

		/**
		 * Gets number of rows.
		 *
		 * @return int
		 */
		int rows();

	protected:
		/**
		 * @var Cell*** cells of the map
		 */
		Cell ***_cells;

		/**
		 * @var int columns
		 */
		int _cols;

		/**
		 * @var int rows
		 */
		int _rows;
	};
};

#endif // DSTARLITE_MAP_H