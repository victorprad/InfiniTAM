// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <map>

#include "SparseBlockMatrix.h"

namespace MiniSlamGraph
{
	template<int BlockSizeRows, int BlockSizeCols>
	class SparseRegularBlockMatrix : public SparseBlockMatrix 
	{
	public:
		static const int bsRows = BlockSizeRows;
		static const int bsCols = BlockSizeCols;

		struct BlockIndex 
		{
			int block_r, block_c;
			BlockIndex(void) {}
			BlockIndex(int r, int c) { block_r = r; block_c = c; }
			bool operator<(const BlockIndex & b) const
			{
				if (this->block_r < b.block_r) return true;
				if (this->block_r > b.block_r) return false;
				if (this->block_c < b.block_c) return true;
				return false;
			}
		};

		struct BlockData 
		{
			double data[BlockSizeRows*BlockSizeCols];
			//BlockData(void) { for (int i = 0; i < BlockSizeRows*BlockSizeCols; ++i) data[i] = 0.0f; }
			double & operator[](int idx) { return data[idx]; }
			const double & operator[](int idx) const { return data[idx]; }
		};
		typedef std::map<BlockIndex, BlockData> MatrixData;

		bool addBlock(int row, int col, int nr, int nc, double *data)
		{
			int block_row = row / BlockSizeRows;
			int block_col = col / BlockSizeCols;
			// TODO: handle ERROR
			if ((nr != BlockSizeRows) || (nc != BlockSizeCols)) return false;
			if ((row != block_row*BlockSizeRows) || (col != block_col*BlockSizeCols)) return false;

			typename MatrixData::iterator it = mData.find(BlockIndex(block_row, block_col));

			if (it != mData.end()) {
				for (int i = 0; i < BlockSizeRows*BlockSizeCols; ++i) it->second[i] += data[i];
			}
			else {
				BlockData d;
				for (int i = 0; i < BlockSizeRows*BlockSizeCols; ++i) d[i] = data[i];
				mData.insert(std::make_pair(BlockIndex(block_row, block_col), d));
			}

			return true;
		}

		void getStats(int & numRows, int & numCols, int & numEntries) const
		{
			numRows = -1;
			numCols = -1;
			numEntries = 0;

			typename MatrixData::const_iterator it = mData.begin();
			for (; it != mData.end(); ++it) {
				int blockPos_r = it->first.block_r;
				int blockPos_c = it->first.block_c;
				if (blockPos_r > numRows) numRows = blockPos_r;
				if (blockPos_c > numCols) numCols = blockPos_c;
				numEntries += BlockSizeRows*BlockSizeCols;
			}
			numRows = (numRows + 1) * BlockSizeRows;
			numCols = (numCols + 1) * BlockSizeCols;
		}

		int toTriplets(int *rowIndices, int *colIndices, double *data) const
		{
			// TODO: untested. should be fine!
			int numEntries = 0;
			typename MatrixData::const_iterator it = mData.begin();
			for (; it != mData.end(); ++it) {
				int blockPos_r = it->first.block_r * BlockSizeRows;
				int blockPos_c = it->first.block_c * BlockSizeCols;
				for (int r = 0; r < BlockSizeRows; ++r) for (int c = 0; c < BlockSizeCols; ++c) {
					rowIndices[numEntries] = blockPos_r + r;
					colIndices[numEntries] = blockPos_c + c;
					data[numEntries] = it->second[r*BlockSizeRows + c];
					++numEntries;
				}
			}
			return numEntries;
		}

		void toCompressedColumns(int *rowIndices, int *colPointers, double *data) const
		{
			typename MatrixData::const_iterator it = mData.begin();
			std::vector<int> entriesPerColumn_blockwise;
			for (; it != mData.end(); ++it) {
				int blockPos_c = it->first.block_c;
				if (entriesPerColumn_blockwise.size() < (size_t)(blockPos_c + 1)) entriesPerColumn_blockwise.resize(blockPos_c + 1, 0);
				entriesPerColumn_blockwise[blockPos_c] += 1;
			}
			int columnOffset = 0;
			for (size_t blockIdx_c = 0; blockIdx_c < entriesPerColumn_blockwise.size(); ++blockIdx_c) {
				for (int i = 0; i < BlockSizeCols; ++i) {
					//fprintf(stderr, "in column %i: %i entries starting at offset %i\n", blockIdx_c*BlockSizeCols+i, entriesPerColumn_blockwise[blockIdx_c] * BlockSizeRows, columnOffset);
					colPointers[blockIdx_c*BlockSizeCols + i] = columnOffset;
					columnOffset += entriesPerColumn_blockwise[blockIdx_c] * BlockSizeRows;
				}
			}
			colPointers[entriesPerColumn_blockwise.size()*BlockSizeCols] = columnOffset;

			int numEntries = 0;
			for (size_t i = 0; i < entriesPerColumn_blockwise.size(); ++i) entriesPerColumn_blockwise[i] = 0;
			for (it = mData.begin(); it != mData.end(); ++it) {
				int blockIdx_c = it->first.block_c;
				int blockPos_r = it->first.block_r * BlockSizeRows;
				int blockPos_c = blockIdx_c * BlockSizeCols;
				//fprintf(stderr, "placing block at %i %i\n", blockPos_r, blockPos_c);
				for (int r = 0; r < BlockSizeRows; ++r) for (int c = 0; c < BlockSizeCols; ++c) {
					int idx = colPointers[blockPos_c + c] + entriesPerColumn_blockwise[blockIdx_c] + r;
					//fprintf(stderr, "      entry %i %i: %i %i\n", r,c, colPointers[blockPos_c+c], idx);
					rowIndices[idx] = blockPos_r + r;
					data[idx] = it->second[r*BlockSizeRows + c];
					++numEntries;
				}
				entriesPerColumn_blockwise[blockIdx_c] += BlockSizeRows;
			}
			//fprintf(stderr, "column pointers:\n");
			//for (size_t i = 0; i < entriesPerColumn_blockwise.size()*BlockSizeCols+1; ++i) fprintf(stderr, "%i ", colPointers[i]);
			//fprintf(stderr, "\nrow indices:\n");
			//for (size_t i = 0; i < numEntries; ++i) fprintf(stderr, "%i ", rowIndices[i]);
		}

		void densify(double *dest, int rowStride) const
		{
			typename MatrixData::const_iterator it = mData.begin();
			for (; it != mData.end(); ++it) {
				int blockPos_r = it->first.block_r * BlockSizeRows;
				int blockPos_c = it->first.block_c * BlockSizeCols;
				//fprintf(stderr, "filling block at %i %i\n", blockPos_r, blockPos_c);
				for (int r = 0; r < BlockSizeRows; ++r) for (int c = 0; c < BlockSizeCols; ++c) {
					dest[(blockPos_r + r)*rowStride + blockPos_c + c] = it->second[r*BlockSizeRows + c];
				}
			}
		}

	private:
		MatrixData mData;
	};
}