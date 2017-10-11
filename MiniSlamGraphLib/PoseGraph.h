// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "SlamGraph.h"

namespace MiniSlamGraph
{
	class PoseGraph : public SlamGraph
	{
	protected:
		void allocateGradientAndHessian(VariableLengthVector* & g, SparseBlockMatrix* & H) const;
	};
}

