// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#ifdef COMPILE_WITH_CSPARSE

#include <suitesparse/cs.h>
#include <string.h>

#include "MatrixWrapper.h"
#include "SparseBlockMatrix.h"

namespace MiniSlamGraph {

	class Matrix_CSparse : public Matrix {
	public:
		typedef cs Data;
		typedef csn Decomposition;
		typedef css Pattern;

		static void freePattern(Pattern *pattern)
		{
			cs_sfree(pattern);
		}

		Matrix_CSparse(const SparseBlockMatrix & src, Pattern * &sparsityPattern)
		{
			mData = createData(src);
			if (sparsityPattern == NULL) sparsityPattern = computePattern();
			mPattern = sparsityPattern;
		}
		Matrix_CSparse(const Matrix_CSparse & src)
		{
			bool isCompressedColumns = (src.mData->nz == -1);
			mData = cs_spalloc(src.mData->m, src.mData->n, src.mData->nzmax, src.mData->nz, !isCompressedColumns);
			if (isCompressedColumns) memcpy(mData->p, src.mData->p, (src.mData->n + 1) * sizeof(int));
			else memcpy(mData->p, src.mData->p, src.mData->nzmax * sizeof(int));
			memcpy(mData->i, src.mData->i, src.mData->nzmax * sizeof(int));
			memcpy(mData->x, src.mData->x, src.mData->nzmax * sizeof(double));
			mPattern = src.mPattern;
		}

		~Matrix_CSparse(void)
		{
			cs_spfree(mData);
		}

		static Data* createData(const SparseBlockMatrix & src)
		{
			int numRows, numCols, numEntries;
			src.getStats(numRows, numCols, numEntries);
			Data *ret = cs_spalloc(numRows, numCols, numEntries, numEntries, 0 /*compressed columns*/);
			src.toCompressedColumns(ret->i, ret->p, ret->x);
			return ret;
		}

		Matrix_CSparse* clone(void) const
		{
			return new Matrix_CSparse(*this);
		}

		void multiply(const double *b, double *x) const
		{
			for (int i = 0; i < numRows(); ++i) x[i] = 0.0f;
			cs_gaxpy(mData, b, x);
		}

		Pattern* computePattern(void) const
		{
			// order 0: "natural", order 1: "cholesky"
			static const int order = 1;
			return cs_schol(order, mData);
		}

		bool solve(const double *b, double *x) const
		{
			bool localPattern = false;
			Pattern *S = mPattern;
			if (S == NULL) {
				S = computePattern();
				localPattern = true;
			}
			csn *N = cs_chol(mData, S);

			int n = mData->n;
			double *y = (double*)cs_malloc(n, sizeof(double));
			cs_ipvec(S->pinv, b, y, n);   /* x = P*b */
			cs_lsolve(N->L, y);           /* x = L\x */
			cs_ltsolve(N->L, y);          /* x = L'\x */
			cs_pvec(S->pinv, y, x, n);    /* b = P'*x */
			cs_free(y);
			cs_nfree(N);
			if (localPattern) freePattern(S);
			return true;
		}

		const double & diag(int i) const
		{
			for (int j = mData->p[i]; j < mData->p[i + 1]; ++j) if (mData->i[j] == i) return mData->x[j];
			return emptyElement;
		}
		double & diag(int i)
		{
			for (int j = mData->p[i]; j < mData->p[i + 1]; ++j) if (mData->i[j] == i) return mData->x[j];
			return emptyElement;
		}

		int numRows(void) const
		{
			return mData->m;
		}

		int numCols(void) const
		{
			return mData->n;
		}

	private:
		Data *mData;
		Pattern *mPattern;

		double emptyElement;
	};
}

#endif

