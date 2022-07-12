#ifndef DSTARLITE_MAP_H
#define DSTARLITE_MAP_H

#include <functional>
#include <stdlib.h>

#include "math.h"

using namespace std;

namespace DStarLite
{
	class Map
	{
	public:
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
			 * @var static const unsigned int number of cell neighbors
			 */
			static const unsigned int NUM_NBRS;

			/**
			 * @var static const unsigned int number of cell corners
			 */
			static const unsigned int NUM_CNRS;

			/**
			 * @var static const unsigned int number of distinct traversal costs
			 */
			static const unsigned int DIST_TRAV_COSTS;

			/**
			 * @var static const double cost of an unwalkable cell
			 */
			static const double COST_UNWALKABLE;

			/**
			 * @var double cost of cell
			 */
			double cost;

			/**
			 * Constructor.
			 *
			 * @param unsigned int x-coordinate
			 * @param unsigned int y-coordinate
			 * @param double [optional] cost of the cell
			 */
			Cell(unsigned int x, unsigned int y, double cost = 1.0);

			/**
			 * Deconstructor.
			 */
			~Cell();

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
			 * Gets cell corners.
			 *
			 * @return Cell**
			 */
			Cell **cnrs();

			/**
			 * Gets x-coordinate.
			 *
			 * @return unsigned int
			 */
			unsigned int x();

			/**
			 * Gets y-coordinate.
			 *
			 * @return unsigned int
			 */
			unsigned int y();

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
			 * @var unsigned int x-coordinate
			 */
			unsigned int _x;

			/**
			 * @var unsigned int y-coordinate
			 */
			unsigned int _y;
		};

		/**
		 * Constructor.
		 *
		 * @param unsigned int rows
		 * @param unsigned int columns
		 */
		Map(unsigned int rows, unsigned int cols);

		/**
		 * Deconstructor.
		 */
		~Map();

		/**
		 * Retrieves a cell.
		 *
		 * @param unsigned int row
		 * @param unsigned int column
		 * @return Map::Cell*
		 */
		Cell *operator()(const unsigned int row, const unsigned int col);

		/**
		 * Gets number of cols.
		 *
		 * @return unsigned int
		 */
		unsigned int cols();

		/**
		 * Checks if row/col exists.
		 *
		 * @param unsigned int row
		 * @param unsigned int col
		 * @return bool
		 */
		bool has(unsigned int row, unsigned int col);

		/**
		 * Gets number of rows.
		 *
		 * @return unsigned int
		 */
		unsigned int rows();

	protected:
		/**
		 * @var Cell*** cells of the map
		 */
		Cell ***_cells;

		/**
		 * @var unsigned int columns
		 */
		unsigned int _cols;

		/**
		 * @var unsigned int rows
		 */
		unsigned int _rows;
	};
};

#endif // DSTARLITE_MAP_H