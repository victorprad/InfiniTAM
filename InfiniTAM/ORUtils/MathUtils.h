// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <math.h>

#ifndef MIN
#define MIN(a,b) ((a < b) ? a : b)
#endif

#ifndef MAX
#define MAX(a,b) ((a < b) ? b : a)
#endif

#ifndef ABS
#define ABS(a) ((a < 0) ? -a : a)
#endif

#ifndef CLAMP
#define CLAMP(x,a,b) MAX((a), MIN((b), (x)))
#endif

#ifndef ROUND
#define ROUND(x) ((x < 0) ? (x - 0.5f) : (x + 0.5f))
#endif

#ifndef PI
#define PI float(3.1415926535897932384626433832795)
#endif

#ifndef DEGTORAD
#define DEGTORAD float(0.017453292519943295769236907684886)
#endif

#ifndef MY_INF
#define MY_INF 0x7f800000
#endif

#ifndef __METALC__

inline bool portable_finite(float a)
{
	volatile float temp = a;
	if (temp != a) return false;
	if ((temp - a) != 0.0) return false;
	return true;
}

inline void matmul(const float *A, const float *b, float *x, int numRows, int numCols)
{
	for (int r = 0; r < numRows; ++r)
	{
		float res = 0.0f;
		for (int c = 0; c < numCols; ++c) res += A[r*numCols + c] * b[c];
		x[r] = res;
	}
}

#endif

/** Solve linear equation system using (non-pivoting!) Gaussian Elimination.
    This implementation does not do pivoting, but assumes that the matrix A
    has a strictly non-zero anti-diagonal. It's going to fail otherwise, and
    on top it's of course numerically unstable.
*/
template<typename T_RHS>
_CPU_AND_GPU_CODE_ inline void solveGaussianEliminationLower(float *A, T_RHS *b, T_RHS *x, int dim)
{
	// transform to lower triangular matrix
	for (int r = 0; r < dim; r++)
	{
		int r_ = dim-r-1;
		for (int r2 = 0; r2 < r_; r2++)
		{
			if (fabs(A[r2*dim+r])<1e-6) continue;
			float v = A[r2*dim+r]/A[r_*dim+r];
			for (int c = 0; c < dim; c++)
			{
				A[r2*dim+c] -= v*A[r_*dim+c];
			}
			b[r2] -= v*b[r_];
		}
	}

	x[dim-1] = b[0] / A[dim-1];

	// perform forward substitution
	for (int r = 1; r < dim; r++)
	{
		T_RHS v = 0.0f;
		for (int c = dim-r; c < dim; c++)
		{
			v += A[r*dim+c]*x[c];
		}
		x[dim-r-1] = (b[r]-v)/A[r*dim+dim-r-1];
	}
}

