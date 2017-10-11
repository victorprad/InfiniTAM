// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

namespace MiniSlamGraph
{
	class SparseBlockMatrix
	{
	public:
		virtual ~SparseBlockMatrix(void) {}

		/** Get number of rows, columns and overall elements in the matrix. */
		virtual void getStats(int & numRows, int & numCols, int & numEntries) const = 0;

		/** For each allocated element, write row index, column index and value
			to the given arrays in triplet format.
		*/
		virtual int toTriplets(int *rowIndices, int *colIndices, double *data) const = 0;

		/** Convert to Compressed Columns format. */
		virtual void toCompressedColumns(int *rowIndices, int *colPointers, double *data) const = 0;

		/** Convert to dense matrix. */
		virtual void densify(double *dest, int rowStride) const = 0;

		/** Add a block of data to the matrix. */
		virtual bool addBlock(int row, int col, int nr, int nc, double *data) = 0;

		/** Transpose a block of data and then add it to the matrix. */
		virtual bool addBlockTranspose(int row, int col, int nr, int nc, double *data)
		{
			std::vector<double> data_t(nr*nc);
			for (int r = 0; r < nr; ++r) for (int c = 0; c < nc; ++c) data_t[r*nc + c] = data[c*nr + r];
			return addBlock(row, col, nc, nr, &(data_t[0]));
		}
	};
}