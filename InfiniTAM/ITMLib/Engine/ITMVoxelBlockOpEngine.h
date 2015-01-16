// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <math.h>

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMScene.h"
#include "../Objects/ITMView.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMVoxelBlockOpEngine
		{
		public:
			virtual void SplitAndMerge(ITMScene<TVoxel,TIndex> *scene) = 0;

			ITMVoxelBlockOpEngine(void) { }
			virtual ~ITMVoxelBlockOpEngine(void) { }
		};
	}
}
