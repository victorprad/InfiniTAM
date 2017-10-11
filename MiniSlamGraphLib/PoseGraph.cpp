// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "PoseGraph.h"

#include "SparseRegularBlockMatrix.h"

using namespace MiniSlamGraph;

void PoseGraph::allocateGradientAndHessian(VariableLengthVector* & g, SparseBlockMatrix* & H) const
{
	H = new SparseRegularBlockMatrix<6, 6>();
	g = new VariableLengthVector();
}