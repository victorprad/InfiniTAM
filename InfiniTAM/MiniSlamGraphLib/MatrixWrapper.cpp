// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "MatrixWrapper.h"
#include "../ORUtils/Cholesky.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace MiniSlamGraph;

/** Return true, if a double is almost zero. */
static inline bool almostZero(const double a) {
	return (fabs(a) < 1e-15);
}

void Matrix::addDiagonal(double lambda)
{
	int nr = numRows();
	int nc = numCols();
	if (nc < nr) nr = nc;
	for (int i = 0; i < nr; i++) {
		diag(i) += lambda;
	}
}

void Matrix::multDiagonal(double lambda)
{
	int nr = numRows();
	int nc = numCols();
	if (nc < nr) nr = nc;
	for (int i = 0; i < nr; i++) {
		double & ele = diag(i);
		if (!almostZero(ele)) {
			ele *= (1.0 + lambda);
		}
		else {
			ele = lambda*1e-10;
		}
	}
}

MatrixSymPosDef::MatrixSymPosDef(int _size)
{
	size = _size;
	memory = new double[size*size];
}

MatrixSymPosDef::MatrixSymPosDef(const MatrixSymPosDef & src)
{
	size = src.size;
	memory = new double[size*size];
	memcpy(memory, src.memory, size*size * sizeof(double));
}


MatrixSymPosDef::~MatrixSymPosDef(void)
{
	delete[] memory;
}

void MatrixSymPosDef::multiply(const double *b, double *x) const
{
	for (int i = 0; i < numRows(); i++) {
		x[i] = 0.0;
		for (int k = 0; k < numCols(); k++) {
			x[i] += getMemory()[i*numCols() + k] * b[k];
		}
	}
}

bool MatrixSymPosDef::solve(const double *b, double *x) const
{
	ORUtils::GenericCholesky<double> ch(getMemory(), numRows());
	ch.Backsub(x, b);
	return true;
}

