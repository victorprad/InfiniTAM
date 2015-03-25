// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <math.h>

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMScene.h"
#include "../Objects/ITMMesh.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMMeshingEngine
	{
	public:
		virtual void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel,TIndex> *scene) = 0;

		ITMMeshingEngine(void) { }
		virtual ~ITMMeshingEngine(void) { }
	};
}
